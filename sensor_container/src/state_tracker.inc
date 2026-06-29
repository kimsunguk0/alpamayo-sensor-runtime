class StateTracker {
 public:
  StateTracker(const std::string& dev, int baud, UtcMapper* utc_mapper,
               const std::array<double, 3>& gnss_antenna_to_ego_m = {0.0, 0.0, 0.0},
               YawHeadingCsvLogger* yaw_heading_logger = nullptr)
      : dev_(dev),
        baud_(baud),
        utc_mapper_(utc_mapper),
        gnss_antenna_to_ego_m_(gnss_antenna_to_ego_m),
        yaw_heading_logger_(yaw_heading_logger) {}

  ~StateTracker() { stop(); }

  bool start() {
    if (dev_.empty()) {
      has_error_.store(true);
      error_message_ = "no IMU/GNSS device configured";
      return false;
    }
    running_.store(true);
    thread_ = std::thread([this]() { thread_main(); });
    return true;
  }

  void stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
  }

  bool has_error() const { return has_error_.load(); }
  std::string error_message() const { return error_message_; }
  void set_data_callback(std::function<void()> callback) {
    std::lock_guard<std::mutex> lk(callback_mu_);
    data_callback_ = std::move(callback);
  }
  bool imu_ready() const { return imu_count_.load() > 0; }
  bool pose_ready() const { return pose_ring_.size() > 0; }
  uint64_t latest_pose_utc_us() const { return pose_ring_.latest_utc_us(); }
  bool latest_pose_timing(uint64_t& utc_us, uint64_t& received_wall_us, uint64_t& pushed_wall_us) const {
    return pose_ring_.latest_timing(utc_us, received_wall_us, pushed_wall_us);
  }
  bool latest_orientation_timing(uint64_t& utc_us, uint64_t& received_wall_us, uint64_t& pushed_wall_us) const {
    return orientation_ring_.latest_timing(utc_us, received_wall_us, pushed_wall_us);
  }
  const PoseRing& pose_ring() const { return pose_ring_; }
  const OrientationRing& orientation_ring() const { return orientation_ring_; }
  std::string imu_status_string() const {
    std::ostringstream os;
    os << std::fixed << std::setprecision(2)
       << "imu_count=" << imu_count_.load()
       << " imu_sample_hz=" << imu_rate_.hz()
       << " orient_count=" << orientation_count_.load()
       << " orient_hz=" << orientation_rate_.hz();
    std::lock_guard<std::mutex> lk(status_mu_);
    if (last_imu_) {
      os << std::fixed << std::setprecision(4)
         << " acc=[" << last_imu_->acc[0] << "," << last_imu_->acc[1] << "," << last_imu_->acc[2] << "]"
         << " gyro=[" << last_imu_->gyro[0] << "," << last_imu_->gyro[1] << "," << last_imu_->gyro[2] << "]"
         << " euler_deg=[" << last_imu_->euler_deg[0] << "," << last_imu_->euler_deg[1] << "," << last_imu_->euler_deg[2] << "]";
    }
    return os.str();
  }
  std::string gnss_status_string() const {
    std::ostringstream os;
    os << std::fixed << std::setprecision(2)
       << "gnss_frames=" << gnss_count_.load()
       << " gnss_msg_hz=" << gnss_rate_.hz()
       << " pose_count=" << pose_ring_.size()
       << " pose_hz=" << pose_rate_.hz();
    std::lock_guard<std::mutex> lk(status_mu_);
    if (last_gnss_) {
      os << std::fixed << std::setprecision(6)
         << " utc_valid=0x" << std::hex << int(last_gnss_->utc_valid) << std::dec;
      if (last_gnss_->fix_type >= 0) os << " fix_type=" << last_gnss_->fix_type;
      if (last_gnss_->num_sv >= 0) os << " num_sv=" << last_gnss_->num_sv;
    }
    const auto& gnss_for_fix_log =
        (last_gnss_fix_ && last_gnss_fix_->lat && last_gnss_fix_->lon) ? last_gnss_fix_ : last_gnss_;
    if (gnss_for_fix_log) {
      if (gnss_for_fix_log->lat && gnss_for_fix_log->lon) {
        os << " lat=" << *gnss_for_fix_log->lat << " lon=" << *gnss_for_fix_log->lon;
      }
      if (gnss_for_fix_log->alt) os << " alt=" << *gnss_for_fix_log->alt;
      if (gnss_for_fix_log->vel) {
        os << " vel=[" << (*gnss_for_fix_log->vel)[0] << "," << (*gnss_for_fix_log->vel)[1] << "," << (*gnss_for_fix_log->vel)[2] << "]";
      }
      if (gnss_for_fix_log->utm_xyz) {
        os << " utm=[" << (*gnss_for_fix_log->utm_xyz)[0] << "," << (*gnss_for_fix_log->utm_xyz)[1] << "," << (*gnss_for_fix_log->utm_xyz)[2] << "]"
           << " utm_zone=" << gnss_for_fix_log->utm_zone
           << (gnss_for_fix_log->utm_northp ? "N" : "S");
      }
      if (gnss_for_fix_log->covariance_enu_m2) {
        const auto& cov = *gnss_for_fix_log->covariance_enu_m2;
        os << " cov_diag_m2=[" << cov[0] << "," << cov[4] << "," << cov[8] << "]";
      }
      if (last_gnss_fix_ && last_gnss_) {
        uint64_t age_us = (last_gnss_->utc_us > last_gnss_fix_->utc_us)
                              ? (last_gnss_->utc_us - last_gnss_fix_->utc_us)
                              : (last_gnss_fix_->utc_us - last_gnss_->utc_us);
        os << " fix_age_ms=" << std::fixed << std::setprecision(1) << (static_cast<double>(age_us) / 1000.0);
      }
    }
    return os.str();
  }

 private:
  class RecentRateTracker {
   public:
    explicit RecentRateTracker(uint64_t window_ns = 10000000000ULL) : window_ns_(window_ns) {}

    void push(uint64_t ts_ns) {
      std::lock_guard<std::mutex> lk(m_);
      timestamps_.push_back(ts_ns);
      trim_locked(ts_ns);
    }

    double hz() const {
      std::lock_guard<std::mutex> lk(m_);
      if (timestamps_.size() < 2) return 0.0;
      uint64_t newest = timestamps_.back();
      uint64_t oldest = timestamps_.front();
      if (newest <= oldest) return 0.0;
      double span_sec = static_cast<double>(newest - oldest) * 1e-9;
      if (span_sec <= 0.0) return 0.0;
      return static_cast<double>(timestamps_.size() - 1) / span_sec;
    }

   private:
    void trim_locked(uint64_t now_ns) {
      while (!timestamps_.empty() && (now_ns - timestamps_.front()) > window_ns_) timestamps_.pop_front();
    }

    uint64_t window_ns_;
    mutable std::mutex m_;
    std::deque<uint64_t> timestamps_;
  };

  struct LatestImuStatus {
    std::array<float, 3> acc{0.0f, 0.0f, 0.0f};
    std::array<float, 3> gyro{0.0f, 0.0f, 0.0f};
    std::array<double, 3> euler_deg{0.0, 0.0, 0.0};
  };
  struct LatestGnssStatus {
    uint64_t utc_us = 0;
    std::optional<double> lat;
    std::optional<double> lon;
    std::optional<double> alt;
    std::optional<std::array<double, 3>> vel;
    std::optional<std::array<double, 3>> utm_xyz;
    int utm_zone = 0;
    bool utm_northp = true;
    std::optional<std::array<double, 9>> covariance_enu_m2;
    uint8_t utc_valid = 0;
    int fix_type = -1;
    int num_sv = -1;
  };
  struct ParsedAsciiLog {
    std::string name;
    std::vector<std::string> header_fields;
    std::vector<std::string> body_fields;
  };
  struct RawImuScale {
    double rate_hz = 0.0;
    double accel_delta_v_per_lsb = 0.0;
    double gyro_delta_deg_per_lsb_xy = 0.0;
    double gyro_delta_deg_per_lsb_z = 0.0;
    bool valid = false;
  };

  static bool try_parse_double(const std::string& text, double& out) {
    if (text.empty()) return false;
    char* end = nullptr;
    out = std::strtod(text.c_str(), &end);
    return end && *end == '\0' && std::isfinite(out);
  }

  static bool try_parse_long(const std::string& text, long& out) {
    if (text.empty()) return false;
    char* end = nullptr;
    out = std::strtol(text.c_str(), &end, 10);
    return end && *end == '\0';
  }

  static bool try_parse_uint16(const std::string& text, uint16_t& out) {
    long parsed = 0;
    if (!try_parse_long(text, parsed) || parsed < 0 || parsed > 65535) return false;
    out = static_cast<uint16_t>(parsed);
    return true;
  }

  static bool try_parse_int(const std::string& text, int& out) {
    long parsed = 0;
    if (!try_parse_long(text, parsed) ||
        parsed < std::numeric_limits<int>::min() || parsed > std::numeric_limits<int>::max()) {
      return false;
    }
    out = static_cast<int>(parsed);
    return true;
  }

  static bool is_nonzero_vector(const std::array<double, 3>& v) {
    return std::fabs(v[0]) > 1e-9 || std::fabs(v[1]) > 1e-9 || std::fabs(v[2]) > 1e-9;
  }

  static RawImuScale raw_imu_scale_for_type(int imu_type) {
    switch (imu_type) {
      case 3:
        return {100.0, 4.65661287307739e-08, 3.35276126861572e-07, 3.35276126861572e-07, true};
      case 5:
        return {125.0, 2.99275207519531e-08, 2.44140625e-07, 2.44140625e-07, true};
      case 6:
        return {125.0, 5.98550415039063e-08, 2.31193542480469e-07, 2.31193542480469e-07, true};
      default:
        return {};
    }
  }

  bool parse_ascii_log(const std::string& line, ParsedAsciiLog& out) const {
    if (!ascii_xor_checksum_ok(line)) return false;
    size_t star = line.find('*');
    if (star == std::string::npos || star <= 1) return false;
    size_t semi = line.find(';', 1);
    std::string head = line.substr(1, ((semi == std::string::npos) ? star : semi) - 1);
    out.header_fields = split_csv(head);
    if (out.header_fields.empty()) return false;
    out.name = out.header_fields[0];
    if (semi != std::string::npos && semi < star) {
      out.body_fields = split_csv(line.substr(semi + 1, star - semi - 1));
    } else {
      out.body_fields.clear();
    }
    return true;
  }

	  bool parse_proprietary_time_from_week_tow(uint16_t gps_week, double tow_sec,
	                                            uint64_t t_rx_ns, uint64_t& utc_ns,
	                                            uint8_t& utc_valid) const {
	    utc_valid = 0;
	    if (gps_week_tow_to_unix_ns(gps_week, tow_sec, utc_ns)) {
	      if (plausible_utc_ns(utc_ns)) {
	        utc_valid = 0x03;
	        utc_mapper_->update(t_rx_ns, utc_ns);
	        return true;
	      }
	    }
	    if (utc_mapper_->map(t_rx_ns, utc_ns)) return true;
	    utc_ns = utc_mapper_->fallback_from_anchor(t_rx_ns);
	    return true;
	  }

	  bool parse_proprietary_header_week_tow(const std::vector<std::string>& header_fields,
	                                         uint16_t& gps_week, double& tow_sec) const {
	    auto try_candidate = [&](size_t week_idx, size_t tow_idx) -> bool {
	      if (week_idx >= header_fields.size() || tow_idx >= header_fields.size()) return false;
	      uint16_t maybe_week = 0;
	      double maybe_tow_raw = 0.0;
	      if (!try_parse_uint16(header_fields[week_idx], maybe_week) || !plausible_gps_week(maybe_week)) return false;
	      if (!try_parse_double(header_fields[tow_idx], maybe_tow_raw) || maybe_tow_raw < 0.0) return false;
	      double maybe_tow_sec = maybe_tow_raw;
	      if (maybe_tow_sec > 7.0 * 86400.0 + 1.0 && maybe_tow_sec <= 7.0 * 86400.0 * 1000.0 + 1000.0) {
	        maybe_tow_sec *= 1e-3;
	      }
	      if (maybe_tow_sec < 0.0 || maybe_tow_sec > 7.0 * 86400.0 + 1.0) return false;
	      gps_week = maybe_week;
	      tow_sec = maybe_tow_sec;
	      return true;
	    };

	    // NovAtel ASCII long headers are:
	    // LOG,PORT,SEQ,IDLE,TIMESTATUS,WEEK,SECONDS,RECEIVERSTATUS,...
	    if (try_candidate(5, 6)) return true;
	    for (size_t i = 1; i + 1 < header_fields.size(); ++i) {
	      if (try_candidate(i, i + 1)) return true;
	    }
	    return false;
	  }

  uint64_t master_to_wall_us(uint64_t master_ns) const {
    return utc_mapper_->fallback_from_anchor(master_ns) / 1000ULL;
  }

  void notify_data_callback() {
    std::function<void()> callback;
    {
      std::lock_guard<std::mutex> lk(callback_mu_);
      callback = data_callback_;
    }
    if (callback) callback();
  }

  void update_raw_imu_status(uint64_t utc_ns, const std::array<float, 3>& acc,
                             const std::array<float, 3>& gyro) {
    imu_count_.fetch_add(1);
    imu_rate_.push(now_master_ns());
    std::array<double, 3> euler_deg{0.0, 0.0, 0.0};
    OrientationSample orientation{};
    uint64_t orient_err_us = 0;
    if (orientation_ring_.nearest(utc_ns / 1000ULL, orientation, orient_err_us) && orient_err_us <= 500000ULL) {
      euler_deg = {orientation.roll_deg, orientation.pitch_deg, orientation.yaw_deg};
    }
    std::lock_guard<std::mutex> lk(status_mu_);
    last_imu_ = LatestImuStatus{acc, gyro, euler_deg};
  }

  void mark_error(const std::string& message) {
    error_message_ = message;
    has_error_.store(true);
    log_error(message);
  }

  void push_orientation(uint64_t utc_ns, uint16_t pc, const std::array<double, 3>& euler_deg,
                        const std::array<float, 3>& acc, const std::array<float, 3>& gyro,
                        uint64_t received_wall_us = 0) {
    OrientationSample sample;
    sample.utc_us = utc_ns / 1000ULL;
    sample.received_wall_us = received_wall_us;
    sample.pc = pc;
    sample.roll_deg = euler_deg[0];
    sample.pitch_deg = euler_deg[1];
    sample.yaw_deg = euler_deg[2];
    orientation_ring_.push(sample);
    notify_data_callback();
    imu_count_.fetch_add(1);
    orientation_count_.fetch_add(1);
    uint64_t now_ns = now_master_ns();
    imu_rate_.push(now_ns);
    orientation_rate_.push(now_ns);
    std::lock_guard<std::mutex> lk(status_mu_);
    last_imu_ = LatestImuStatus{acc, gyro, euler_deg};
  }

  void push_gnss_pose(uint64_t utc_ns, uint16_t pc,
                      std::optional<double> lat, std::optional<double> lon, std::optional<double> alt,
                      std::optional<std::array<double, 3>> vel, uint8_t utc_valid,
                      int fix_type = -1, int num_sv = -1,
                      std::optional<std::array<double, 9>> covariance_enu_m2 = std::nullopt,
                      uint64_t received_wall_us = 0) {
    gnss_count_.fetch_add(1);
    uint64_t now_ns = now_master_ns();
    gnss_rate_.push(now_ns);
    std::optional<double> alt_for_pose = alt;
    std::optional<std::array<double, 9>> covariance_for_pose = covariance_enu_m2;
    std::optional<std::array<double, 3>> utm_xyz_for_status;
    int utm_zone_for_status = 0;
    bool utm_northp_for_status = true;
    if (lat && lon && alt_for_pose) {
      UtmCoord utm{};
      if (lla_to_utm(*lat, *lon, utm)) {
        utm_xyz_for_status = std::array<double, 3>{utm.easting_m, utm.northing_m, *alt_for_pose};
        utm_zone_for_status = utm.zone;
        utm_northp_for_status = utm.northp;
      }
    }
    {
      std::lock_guard<std::mutex> lk(status_mu_);
      last_gnss_ = LatestGnssStatus{
          utc_ns / 1000ULL, lat, lon, alt, vel,
          utm_xyz_for_status, utm_zone_for_status, utm_northp_for_status,
          covariance_enu_m2, utc_valid, fix_type, num_sv};
      if (lat && lon) {
        LatestGnssStatus merged = last_gnss_fix_.value_or(LatestGnssStatus{});
        merged.utc_us = utc_ns / 1000ULL;
        merged.lat = lat;
        merged.lon = lon;
        if (alt) merged.alt = alt;
        if (vel) merged.vel = vel;
        if (utm_xyz_for_status) {
          merged.utm_xyz = utm_xyz_for_status;
          merged.utm_zone = utm_zone_for_status;
          merged.utm_northp = utm_northp_for_status;
        }
        if (covariance_enu_m2) merged.covariance_enu_m2 = covariance_enu_m2;
        merged.utc_valid = utc_valid;
        if (fix_type >= 0) merged.fix_type = fix_type;
        if (num_sv >= 0) merged.num_sv = num_sv;
        last_gnss_fix_ = merged;
      }
      if (!alt_for_pose && last_gnss_fix_ && last_gnss_fix_->alt) alt_for_pose = last_gnss_fix_->alt;
      if (!covariance_for_pose && last_gnss_fix_ && last_gnss_fix_->covariance_enu_m2) {
        covariance_for_pose = last_gnss_fix_->covariance_enu_m2;
      }
    }
    OrientationSample orientation{};
    uint64_t orient_err_us = 0;
    const bool have_orientation =
        orientation_ring_.nearest(utc_ns / 1000ULL, orientation, orient_err_us) && orient_err_us <= 200000ULL;
    if (yaw_heading_logger_ && (have_orientation || vel)) {
      yaw_heading_logger_->log_sensor(
          utc_ns / 1000ULL, now_ns,
          have_orientation, have_orientation ? orientation.yaw_deg : 0.0,
          vel, fix_type, num_sv, utc_valid);
    }

    if (!lat || !lon || !alt_for_pose) return;
    const std::array<double, 3> lla{*lat, *lon, *alt_for_pose};
    const std::array<double, 3> ecef = lla_to_ecef(*lat, *lon, *alt_for_pose);
    if (!origin_lla_) {
      origin_lla_ = lla;
      origin_ecef_ = ecef;
      log_info("initialized GNSS origin for pose ring at lat=" + std::to_string(*lat) + " lon=" + std::to_string(*lon));
    }
    PoseSample pose;
    pose.utc_us = utc_ns / 1000ULL;
    pose.received_wall_us = received_wall_us;
    pose.pc = pc;
    pose.lla = lla;
    pose.enu_xyz = ecef_to_enu(ecef, *origin_lla_, origin_ecef_);
    if (vel) {
      pose.vel_enu_mps = *vel;
      pose.velocity_valid = true;
    }
    UtmCoord utm{};
    if (lla_to_utm(*lat, *lon, utm)) {
      pose.utm_xyz = {utm.easting_m, utm.northing_m, *alt_for_pose};
      pose.utm_zone = utm.zone;
      pose.utm_northp = utm.northp;
    }
    if (covariance_for_pose) {
      pose.covariance_enu_m2 = *covariance_for_pose;
      pose.covariance_valid = true;
    }
    if (have_orientation) {
      pose.world_from_ego = rotation_matrix_from_ypr_rad(
          orientation.yaw_deg * M_PI / 180.0,
          orientation.pitch_deg * M_PI / 180.0,
          orientation.roll_deg * M_PI / 180.0);
      if (is_nonzero_vector(gnss_antenna_to_ego_m_)) {
        const auto lever_arm_enu = matvec33(pose.world_from_ego, gnss_antenna_to_ego_m_);
        for (int i = 0; i < 3; ++i) pose.enu_xyz[i] += lever_arm_enu[i];
      }
    }
    pose_ring_.push(pose);
    notify_data_callback();
    pose_rate_.push(now_ns);
  }

  void process_nmea_sentence(const std::string& line, uint64_t t_rx_ns, uint16_t pc_last) {
    if (!ascii_xor_checksum_ok(line)) return;
    const uint64_t received_wall_us = master_to_wall_us(t_rx_ns);
    size_t star = line.find('*');
    if (star == std::string::npos || star < 1) return;
    std::string body = line.substr(1, star - 1);
    auto fields = split_csv(body);
    if (fields.empty()) return;

    if (fields[0].size() >= 5 && fields[0].substr(fields[0].size() - 3) == "RMC") {
      if (fields.size() < 10 || fields[2] != "A") return;
      double lat = 0.0, lon = 0.0;
      if (!parse_nmea_latlon(fields[3], fields[4], true, lat)) return;
      if (!parse_nmea_latlon(fields[5], fields[6], false, lon)) return;
      uint8_t hour = 0, min = 0, sec = 0;
      uint32_t nanos = 0;
      uint16_t year = 0;
      uint8_t mon = 0, day = 0;
      if (!parse_nmea_hms(fields[1], hour, min, sec, nanos)) return;
      if (!parse_nmea_date_ddmmyy(fields[9], year, mon, day)) return;
      uint64_t utc_ns = 0;
      if (!xsens_utc_to_ns(year, mon, day, hour, min, sec, nanos, utc_ns)) return;
      utc_mapper_->update(t_rx_ns, utc_ns);
      nmea_have_date_ = true;
      nmea_year_ = year;
      nmea_mon_ = mon;
      nmea_day_ = day;

      std::optional<double> speed_mps;
      if (fields.size() > 7 && !fields[7].empty()) {
        speed_mps = std::strtod(fields[7].c_str(), nullptr) * 0.514444;
      }

      std::optional<double> course_deg;
      if (fields.size() > 8 && !fields[8].empty()) {
        course_deg = std::strtod(fields[8].c_str(), nullptr);
      }

      std::optional<std::array<double, 3>> vel;
      if (speed_mps && course_deg) {
        const double course_rad = *course_deg * M_PI / 180.0;
        vel = std::array<double, 3>{
            *speed_mps * std::sin(course_rad),
            *speed_mps * std::cos(course_rad),
            0.0,
        };
      }

      double yaw_deg = 0.0;
      if (course_deg) {
        yaw_deg = wrap_angle_deg(90.0 - *course_deg);
      } else {
        OrientationSample prior{};
        uint64_t prior_err_us = 0;
        if (orientation_ring_.nearest(utc_ns / 1000ULL, prior, prior_err_us) && prior_err_us <= 1000000ULL) {
          yaw_deg = prior.yaw_deg;
        }
      }
      push_orientation(
          utc_ns, pc_last, {0.0, 0.0, yaw_deg},
          {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f},
          received_wall_us);
      push_gnss_pose(utc_ns, pc_last, lat, lon, std::nullopt, vel, 0x03, 1, -1, std::nullopt, received_wall_us);
      return;
    }

    if (fields[0].size() >= 5 && fields[0].substr(fields[0].size() - 3) == "GGA") {
      if (fields.size() < 10) return;
      int fix_quality = fields[6].empty() ? 0 : std::atoi(fields[6].c_str());
      if (fix_quality <= 0) return;
      int num_sv = (fields.size() > 7 && !fields[7].empty()) ? std::atoi(fields[7].c_str()) : -1;
      double lat = 0.0, lon = 0.0;
      if (!parse_nmea_latlon(fields[2], fields[3], true, lat)) return;
      if (!parse_nmea_latlon(fields[4], fields[5], false, lon)) return;
      std::optional<double> alt;
      if (!fields[9].empty()) {
        double orthometric_hgt = std::strtod(fields[9].c_str(), nullptr);
        alt = orthometric_hgt;
        if (fields.size() > 11 && !fields[11].empty()) {
          double undulation = 0.0;
          if (try_parse_double(fields[11], undulation)) alt = orthometric_hgt + undulation;
        }
      }
      uint64_t utc_ns = 0;
      uint8_t utc_valid = 0;
      if (nmea_have_date_) {
        uint8_t hour = 0, min = 0, sec = 0;
        uint32_t nanos = 0;
        if (parse_nmea_hms(fields[1], hour, min, sec, nanos) &&
            xsens_utc_to_ns(nmea_year_, nmea_mon_, nmea_day_, hour, min, sec, nanos, utc_ns)) {
          utc_valid = 0x03;
          utc_mapper_->update(t_rx_ns, utc_ns);
          push_gnss_pose(utc_ns, pc_last, lat, lon, alt, std::nullopt, utc_valid, fix_quality, num_sv, std::nullopt, received_wall_us);
        }
      }
    }
  }

  void process_pppnava_sentence(const ParsedAsciiLog& log, uint64_t t_rx_ns, uint16_t pc_last) {
    if (log.body_fields.size() < 6) return;
    if (log.body_fields[0] != "SOL_COMPUTED") return;
    const uint64_t received_wall_us = master_to_wall_us(t_rx_ns);

    double lat = 0.0, lon = 0.0, hgt = 0.0;
    if (!try_parse_double(log.body_fields[2], lat) ||
        !try_parse_double(log.body_fields[3], lon) ||
        !try_parse_double(log.body_fields[4], hgt)) {
      return;
    }

    double alt_m = hgt;
    if (log.body_fields.size() > 5) {
      double undulation = 0.0;
      if (try_parse_double(log.body_fields[5], undulation)) alt_m = hgt + undulation;
    }

    uint16_t gps_week = 0;
    double tow_sec = 0.0;
    uint64_t utc_ns = 0;
    uint8_t utc_valid = 0;
    if (parse_proprietary_header_week_tow(log.header_fields, gps_week, tow_sec) &&
        parse_proprietary_time_from_week_tow(gps_week, tow_sec, t_rx_ns, utc_ns, utc_valid)) {
      push_gnss_pose(utc_ns, pc_last, lat, lon, alt_m, std::nullopt, utc_valid, 4, -1, std::nullopt, received_wall_us);
      return;
    }

    parse_proprietary_time_from_week_tow(0, -1.0, t_rx_ns, utc_ns, utc_valid);
    push_gnss_pose(utc_ns, pc_last, lat, lon, alt_m, std::nullopt, utc_valid, 4, -1, std::nullopt, received_wall_us);
  }

  void process_inspva_sentence(const ParsedAsciiLog& log, uint64_t t_rx_ns, uint16_t pc_last) {
    const uint64_t received_wall_us = master_to_wall_us(t_rx_ns);
    if (log.name == "INSPVAA" || log.name == "INSPVASA") {
      if (log.body_fields.size() < 12) return;
      uint16_t gps_week = 0;
      double tow_sec = 0.0;
      if (!try_parse_uint16(log.body_fields[0], gps_week) || !try_parse_double(log.body_fields[1], tow_sec)) return;
      double lat = 0.0, lon = 0.0, alt_m = 0.0;
      double vn = 0.0, ve = 0.0, vu = 0.0;
      double roll_deg = 0.0, pitch_up_deg = 0.0, azimuth_deg = 0.0;
      if (!try_parse_double(log.body_fields[2], lat) ||
          !try_parse_double(log.body_fields[3], lon) ||
          !try_parse_double(log.body_fields[4], alt_m) ||
          !try_parse_double(log.body_fields[5], vn) ||
          !try_parse_double(log.body_fields[6], ve) ||
          !try_parse_double(log.body_fields[7], vu) ||
          !try_parse_double(log.body_fields[8], roll_deg) ||
          !try_parse_double(log.body_fields[9], pitch_up_deg) ||
          !try_parse_double(log.body_fields[10], azimuth_deg)) {
        return;
      }

      uint64_t utc_ns = 0;
      uint8_t utc_valid = 0;
      parse_proprietary_time_from_week_tow(gps_week, tow_sec, t_rx_ns, utc_ns, utc_valid);
      const double yaw_deg = wrap_angle_deg(90.0 - azimuth_deg);
      const double pitch_deg = -pitch_up_deg;
      push_orientation(
          utc_ns, pc_last, {roll_deg, pitch_deg, yaw_deg},
          {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f},
          received_wall_us);
      push_gnss_pose(utc_ns, pc_last, lat, lon, alt_m, std::array<double, 3>{ve, vn, vu}, utc_valid, 4, -1, std::nullopt, received_wall_us);
      return;
    }

    if (log.name == "INSPVAXA") {
      if (log.body_fields.size() < 12) return;
      double lat = 0.0, lon = 0.0, hgt = 0.0;
      double undulation = 0.0, vn = 0.0, ve = 0.0, vu = 0.0;
      double azimuth_deg = 0.0, pitch_up_deg = 0.0, roll_deg = 0.0;
      if (!try_parse_double(log.body_fields[2], lat) ||
          !try_parse_double(log.body_fields[3], lon) ||
          !try_parse_double(log.body_fields[4], hgt) ||
          !try_parse_double(log.body_fields[5], undulation) ||
          !try_parse_double(log.body_fields[6], vn) ||
          !try_parse_double(log.body_fields[7], ve) ||
          !try_parse_double(log.body_fields[8], vu) ||
          !try_parse_double(log.body_fields[9], azimuth_deg) ||
          !try_parse_double(log.body_fields[10], pitch_up_deg) ||
          !try_parse_double(log.body_fields[11], roll_deg)) {
        return;
      }

      uint16_t gps_week = 0;
      double tow_sec = 0.0;
      uint64_t utc_ns = 0;
      uint8_t utc_valid = 0;
      if (parse_proprietary_header_week_tow(log.header_fields, gps_week, tow_sec)) {
        parse_proprietary_time_from_week_tow(gps_week, tow_sec, t_rx_ns, utc_ns, utc_valid);
      } else {
        parse_proprietary_time_from_week_tow(0, -1.0, t_rx_ns, utc_ns, utc_valid);
      }

      const double yaw_deg = wrap_angle_deg(90.0 - azimuth_deg);
      const double pitch_deg = -pitch_up_deg;
      push_orientation(
          utc_ns, pc_last, {roll_deg, pitch_deg, yaw_deg},
          {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f},
          received_wall_us);
      push_gnss_pose(utc_ns, pc_last, lat, lon, hgt + undulation, std::array<double, 3>{ve, vn, vu}, utc_valid, 4, -1, std::nullopt, received_wall_us);
    }
  }

  void process_drpvaa_sentence(const ParsedAsciiLog& log, uint64_t t_rx_ns, uint16_t pc_last) {
    if (log.body_fields.size() < 28) return;
    const uint64_t received_wall_us = master_to_wall_us(t_rx_ns);

    double lat = 0.0, lon = 0.0, hgt = 0.0, undulation = 0.0;
    double ve = 0.0, vn = 0.0, vu = 0.0;
    double heading_deg = 0.0, pitch_up_deg = 0.0, roll_deg = 0.0;
    double lat_sigma_m = 0.0, lon_sigma_m = 0.0, hgt_sigma_m = 0.0;
    if (!try_parse_double(log.body_fields[9], lat) ||
        !try_parse_double(log.body_fields[10], lon) ||
        !try_parse_double(log.body_fields[11], hgt) ||
        !try_parse_double(log.body_fields[12], undulation) ||
        !try_parse_double(log.body_fields[13], lat_sigma_m) ||
        !try_parse_double(log.body_fields[14], lon_sigma_m) ||
        !try_parse_double(log.body_fields[15], hgt_sigma_m) ||
        !try_parse_double(log.body_fields[19], ve) ||
        !try_parse_double(log.body_fields[20], vn) ||
        !try_parse_double(log.body_fields[21], vu) ||
        !try_parse_double(log.body_fields[22], heading_deg) ||
        !try_parse_double(log.body_fields[23], pitch_up_deg) ||
        !try_parse_double(log.body_fields[24], roll_deg)) {
      return;
    }

    uint16_t gps_week = 0;
    double tow_sec = 0.0;
    uint64_t utc_ns = 0;
    uint8_t utc_valid = 0;
    if (parse_proprietary_header_week_tow(log.header_fields, gps_week, tow_sec)) {
      parse_proprietary_time_from_week_tow(gps_week, tow_sec, t_rx_ns, utc_ns, utc_valid);
    } else {
      parse_proprietary_time_from_week_tow(0, -1.0, t_rx_ns, utc_ns, utc_valid);
    }

    const double yaw_deg = wrap_angle_deg(90.0 - heading_deg);
    const double pitch_deg = -pitch_up_deg;
    push_orientation(
        utc_ns, pc_last, {roll_deg, pitch_deg, yaw_deg},
        {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f},
        received_wall_us);

    if (log.body_fields[0] != "INSUFFICIENT_OBS" && std::fabs(lat) > 1e-9 && std::fabs(lon) > 1e-9) {
      std::optional<std::array<double, 9>> covariance_enu_m2;
      if (lat_sigma_m >= 0.0 && lon_sigma_m >= 0.0 && hgt_sigma_m >= 0.0 &&
          std::isfinite(lat_sigma_m) && std::isfinite(lon_sigma_m) && std::isfinite(hgt_sigma_m)) {
        // DRPVAA position standard deviations are latitude, longitude, height.
        // Publish covariance in ENU order: east, north, up.
        covariance_enu_m2 = std::array<double, 9>{
            lon_sigma_m * lon_sigma_m, 0.0, 0.0,
            0.0, lat_sigma_m * lat_sigma_m, 0.0,
            0.0, 0.0, hgt_sigma_m * hgt_sigma_m,
        };
      }
      push_gnss_pose(
          utc_ns, pc_last, lat, lon, hgt + undulation,
          std::array<double, 3>{ve, vn, vu}, utc_valid, 4, -1,
          covariance_enu_m2, received_wall_us);
    }
  }

  void process_rawimuxa_sentence(const ParsedAsciiLog& log, uint64_t t_rx_ns) {
    if (log.body_fields.size() < 11) return;
    int imu_info = 0;
    int imu_type = 0;
    uint16_t gps_week = 0;
    double tow_sec = 0.0;
    long z_acc_lsb = 0, neg_y_acc_lsb = 0, x_acc_lsb = 0;
    long z_gyro_lsb = 0, neg_y_gyro_lsb = 0, x_gyro_lsb = 0;
    if (!try_parse_int(log.body_fields[0], imu_info) ||
        !try_parse_int(log.body_fields[1], imu_type) ||
        !try_parse_uint16(log.body_fields[2], gps_week) ||
        !try_parse_double(log.body_fields[3], tow_sec) ||
        !try_parse_long(log.body_fields[5], z_acc_lsb) ||
        !try_parse_long(log.body_fields[6], neg_y_acc_lsb) ||
        !try_parse_long(log.body_fields[7], x_acc_lsb) ||
        !try_parse_long(log.body_fields[8], z_gyro_lsb) ||
        !try_parse_long(log.body_fields[9], neg_y_gyro_lsb) ||
        !try_parse_long(log.body_fields[10], x_gyro_lsb)) {
      return;
    }
    if ((imu_info & 0x03) != 0) return;

    RawImuScale scale = raw_imu_scale_for_type(imu_type);
    if (!scale.valid || scale.rate_hz <= 0.0) return;

    const float ax = static_cast<float>(static_cast<double>(x_acc_lsb) * scale.accel_delta_v_per_lsb * scale.rate_hz);
    const float ay = static_cast<float>(-static_cast<double>(neg_y_acc_lsb) * scale.accel_delta_v_per_lsb * scale.rate_hz);
    const float az = static_cast<float>(static_cast<double>(z_acc_lsb) * scale.accel_delta_v_per_lsb * scale.rate_hz);
    const float gx = static_cast<float>(static_cast<double>(x_gyro_lsb) * scale.gyro_delta_deg_per_lsb_xy * scale.rate_hz);
    const float gy = static_cast<float>(-static_cast<double>(neg_y_gyro_lsb) * scale.gyro_delta_deg_per_lsb_xy * scale.rate_hz);
    const float gz = static_cast<float>(static_cast<double>(z_gyro_lsb) * scale.gyro_delta_deg_per_lsb_z * scale.rate_hz);

    uint64_t utc_ns = 0;
    uint8_t utc_valid = 0;
    parse_proprietary_time_from_week_tow(gps_week, tow_sec, t_rx_ns, utc_ns, utc_valid);
    update_raw_imu_status(utc_ns, {ax, ay, az}, {gx, gy, gz});
  }

  void process_ascii_sentence(const std::string& line, uint64_t t_rx_ns, uint16_t pc_last) {
    if (line.empty()) return;
    if (line[0] == '$') {
      process_nmea_sentence(line, t_rx_ns, pc_last);
      return;
    }

    ParsedAsciiLog log;
    if (!parse_ascii_log(line, log)) return;
    if (log.name == "PPPNAVA") {
      process_pppnava_sentence(log, t_rx_ns, pc_last);
    } else if (log.name == "DRPVAA") {
      process_drpvaa_sentence(log, t_rx_ns, pc_last);
    } else if (log.name == "INSPVAA" || log.name == "INSPVASA" || log.name == "INSPVAXA") {
      process_inspva_sentence(log, t_rx_ns, pc_last);
    } else if (log.name == "RAWIMUXA" || log.name == "RAWIMUSXA") {
      process_rawimuxa_sentence(log, t_rx_ns);
    }
  }

  void thread_main() {
    const double bits_per_byte = 10.0;
    const uint64_t byte_time_ns = static_cast<uint64_t>(llround(1e9 * bits_per_byte / static_cast<double>(baud_)));
    int fd = ::open(dev_.c_str(), O_RDONLY | O_NOCTTY);
    if (fd < 0) {
      mark_error("IMU open failed: " + dev_ + " errno=" + std::to_string(errno));
      running_.store(false);
      return;
    }
    if (!set_serial(fd, baud_)) {
      mark_error("failed to configure serial: " + dev_);
      ::close(fd);
      running_.store(false);
      return;
    }
    tcflush(fd, TCIFLUSH);

    std::deque<uint8_t> buf;
    std::deque<uint64_t> tbuf;
    std::vector<uint8_t> tmp(4096);
    bool have_stf = false;
    uint32_t last_stf32 = 0;
    uint64_t stf_ext = 0;
    LinFitWindow fit;
    fit.window_sec = 10.0;
    double a = 1.0, b = 0.0;
    bool have_ab = false;
    uint16_t pc_last = 0;
    bool ascii_collect = false;
    std::string ascii_line;

    while (running_.load() && !g_stop.load()) {
      int n = ::read(fd, tmp.data(), static_cast<int>(tmp.size()));
      if (n > 0) {
        uint64_t t_read_ns = now_master_ns();
        for (int i = 0; i < n; ++i) {
          uint64_t t_i = t_read_ns - static_cast<uint64_t>((n - 1 - i) * byte_time_ns);
          uint8_t ch = tmp[i];
          if (!ascii_collect) {
            if (ch == '$' || ch == '#' || ch == '%') {
              ascii_collect = true;
              ascii_line.clear();
              ascii_line.push_back(static_cast<char>(ch));
            }
          } else {
            if (ch == '$' || ch == '#' || ch == '%') {
              ascii_line.clear();
              ascii_line.push_back(static_cast<char>(ch));
            } else if (ch == '\n') {
              process_ascii_sentence(ascii_line, t_i, pc_last);
              ascii_collect = false;
              ascii_line.clear();
            } else if (ch != '\r') {
              if (ascii_line.size() < 512) {
                ascii_line.push_back(static_cast<char>(ch));
              } else {
                ascii_collect = false;
                ascii_line.clear();
              }
            }
          }
          buf.push_back(ch);
          tbuf.push_back(t_i);
        }
      } else {
        usleep(1000);
      }

      if (buf.size() > (1u << 20)) {
        buf.clear();
        tbuf.clear();
      }

      while (true) {
        while (!buf.empty() && buf.front() != 0xFA) {
          buf.pop_front();
          tbuf.pop_front();
        }
        if (buf.size() < 5) break;
        uint8_t mid = buf[2];
        uint8_t len = buf[3];
        size_t hdr = 4;
        uint16_t data_len = len;
        if (len == 0xFF) {
          if (buf.size() < 7) break;
          data_len = (uint16_t(buf[4]) << 8) | uint16_t(buf[5]);
          hdr = 6;
        }
        size_t frame_len = hdr + data_len + 1;
        if (buf.size() < frame_len) break;
        if (!checksum_ok_deque(buf, frame_len)) {
          buf.pop_front();
          tbuf.pop_front();
          continue;
        }

        uint64_t t_rx_ns = tbuf[frame_len - 1];
        std::vector<uint8_t> frame(frame_len);
        for (size_t i = 0; i < frame_len; ++i) frame[i] = buf[i];
        for (size_t i = 0; i < frame_len; ++i) {
          buf.pop_front();
          tbuf.pop_front();
        }
        if (mid != 0x36) continue;

        const uint8_t* payload = frame.data() + hdr;
        size_t ppos = 0;

        bool got_stf = false, got_pc = false, got_acc = false, got_gyro = false, got_euler = false;
        bool got_latlon = false, got_alt = false, got_vel = false, got_utc = false;
        uint32_t stf32 = 0;
        uint16_t pc = 0;
        std::array<float, 3> acc{0.0f, 0.0f, 0.0f};
        std::array<float, 3> gyro{0.0f, 0.0f, 0.0f};
        std::array<double, 3> euler_deg{0.0, 0.0, 0.0};
        std::optional<double> lat, lon, alt;
        std::optional<std::array<double, 3>> vel;
        int gnss_fix_type = -1;
        int gnss_num_sv = -1;
        uint16_t utc_year = 0;
        uint8_t utc_mon = 0, utc_day = 0, utc_hour = 0, utc_min = 0, utc_sec = 0, utc_valid = 0;
        uint32_t utc_nanos = 0;

        while (ppos + 3 <= data_len) {
          uint16_t data_id = be_u16(payload + ppos);
          uint8_t sz = payload[ppos + 2];
          ppos += 3;
          if (ppos + sz > data_len) break;
          uint16_t id_mask = data_id & 0xFFF0;
          if (data_id == 0x1060 && sz == 4) {
            stf32 = be_u32(payload + ppos);
            got_stf = true;
          } else if (data_id == 0x1020 && sz == 2) {
            pc = be_u16(payload + ppos);
            got_pc = true;
          } else if (id_mask == 0x4020 && sz == 12) {
            for (int i = 0; i < 3; ++i) acc[i] = be_f32(payload + ppos + 4 * i);
            got_acc = true;
          } else if (id_mask == 0x8020 && sz == 12) {
            for (int i = 0; i < 3; ++i) gyro[i] = be_f32(payload + ppos + 4 * i);
            got_gyro = true;
          } else if (id_mask == 0x2030) {
            const uint16_t precision = data_id & 0x0003;
            bool parsed_euler = false;
            if (precision == 0x0 && sz == 12) {
              for (int i = 0; i < 3; ++i) euler_deg[i] = static_cast<double>(be_f32(payload + ppos + 4 * i));
              parsed_euler = true;
            } else if (precision == 0x1 && sz == 12) {
              for (int i = 0; i < 3; ++i) euler_deg[i] = static_cast<double>(be_i32(payload + ppos + 4 * i)) / 1048576.0;
              parsed_euler = true;
            } else if (precision == 0x2 && sz == 18) {
              for (int i = 0; i < 3; ++i) euler_deg[i] = static_cast<double>(xsens_fp1632_i64(payload + ppos + 6 * i)) / 4294967296.0;
              parsed_euler = true;
            } else if (precision == 0x3 && sz == 24) {
              for (int i = 0; i < 3; ++i) euler_deg[i] = be_f64(payload + ppos + 8 * i);
              parsed_euler = true;
            }
            got_euler = got_euler || parsed_euler;
          } else if (data_id == 0x1010 && sz >= 12) {
            utc_nanos = be_u32(payload + ppos);
            utc_year = be_u16(payload + ppos + 4);
            utc_mon = payload[ppos + 6];
            utc_day = payload[ppos + 7];
            utc_hour = payload[ppos + 8];
            utc_min = payload[ppos + 9];
            utc_sec = payload[ppos + 10];
            utc_valid = payload[ppos + 11];
            got_utc = true;
          } else if (data_id == 0x5040 && sz == 16) {
            lat = be_f64(payload + ppos);
            lon = be_f64(payload + ppos + 8);
            got_latlon = true;
          } else if (data_id == 0x5041 && sz == 8) {
            lat = static_cast<double>(be_i32(payload + ppos)) / 1048576.0;
            lon = static_cast<double>(be_i32(payload + ppos + 4)) / 1048576.0;
            got_latlon = true;
          } else if (data_id == 0x5042 && sz == 12) {
            lat = static_cast<double>(xsens_fp1632_i64(payload + ppos)) / 4294967296.0;
            lon = static_cast<double>(xsens_fp1632_i64(payload + ppos + 6)) / 4294967296.0;
            got_latlon = true;
          } else if (data_id == 0x5020 && sz == 8) {
            alt = be_f64(payload + ppos);
            got_alt = true;
          } else if (data_id == 0x5021 && sz == 4) {
            alt = static_cast<double>(be_f32(payload + ppos));
            got_alt = true;
          } else if (data_id == 0x5022 && sz == 4) {
            alt = static_cast<double>(be_i32(payload + ppos)) / 1048576.0;
            got_alt = true;
          } else if (data_id == 0xD010 && sz == 12) {
            vel = std::array<double, 3>{
                static_cast<double>(be_f32(payload + ppos)),
                static_cast<double>(be_f32(payload + ppos + 4)),
                static_cast<double>(be_f32(payload + ppos + 8)),
            };
            got_vel = true;
          } else if (data_id == 0xD011 && sz == 24) {
            vel = std::array<double, 3>{
                be_f64(payload + ppos),
                be_f64(payload + ppos + 8),
                be_f64(payload + ppos + 16),
            };
            got_vel = true;
          } else if (data_id == 0xD012 && sz == 18) {
            vel = std::array<double, 3>{
                static_cast<double>(xsens_fp1632_i64(payload + ppos)) / 4294967296.0,
                static_cast<double>(xsens_fp1632_i64(payload + ppos + 6)) / 4294967296.0,
                static_cast<double>(xsens_fp1632_i64(payload + ppos + 12)) / 4294967296.0,
            };
            got_vel = true;
          } else if (data_id == 0x7010 && sz >= 60) {
            uint8_t pvt_valid = payload[ppos + 11];
            gnss_fix_type = payload[ppos + 20];
            gnss_num_sv = payload[ppos + 23];
            if (!got_utc && (pvt_valid & 0x03) == 0x03) {
              utc_year = be_u16(payload + ppos + 4);
              utc_mon = payload[ppos + 6];
              utc_day = payload[ppos + 7];
              utc_hour = payload[ppos + 8];
              utc_min = payload[ppos + 9];
              utc_sec = payload[ppos + 10];
              int64_t nano_signed = static_cast<int64_t>(be_i32(payload + ppos + 16));
              if (nano_signed >= 0 && nano_signed < 1000000000LL) {
                utc_nanos = static_cast<uint32_t>(nano_signed);
                utc_valid = pvt_valid;
                got_utc = true;
              }
            }
            if (gnss_fix_type >= 2) {
              lon = static_cast<double>(be_i32(payload + ppos + 24)) * 1e-7;
              lat = static_cast<double>(be_i32(payload + ppos + 28)) * 1e-7;
              alt = static_cast<double>(be_i32(payload + ppos + 36)) * 1e-3;
              vel = std::array<double, 3>{
                  static_cast<double>(be_i32(payload + ppos + 48)) * 1e-3,
                  static_cast<double>(be_i32(payload + ppos + 52)) * 1e-3,
                  static_cast<double>(be_i32(payload + ppos + 56)) * 1e-3,
              };
              got_latlon = true;
              got_alt = true;
              got_vel = true;
            }
          }
          ppos += sz;
        }

        if (!got_stf) continue;
        if (!have_stf) {
          have_stf = true;
          last_stf32 = stf32;
          stf_ext = stf32;
        } else {
          uint32_t delta = stf32 - last_stf32;
          stf_ext += static_cast<uint64_t>(delta);
          last_stf32 = stf32;
        }
        if (got_pc) pc_last = pc;
        double t_sensor = static_cast<double>(stf_ext) / 10000.0;
        double t_host = static_cast<double>(t_rx_ns) * 1e-9;
        fit.push(t_sensor, t_host);
        double aa = 0.0, bb = 0.0;
        if (fit.estimate(aa, bb)) {
          a = aa;
          b = bb;
          have_ab = true;
        }
        double t_est = have_ab ? (a * t_sensor + b) : t_host;
        uint64_t t_master_ns = static_cast<uint64_t>(llround(t_est * 1e9));
        uint64_t t_utc_ns = 0;
        if (got_utc && (utc_valid & 0x03) == 0x03) {
          if (xsens_utc_to_ns(utc_year, utc_mon, utc_day, utc_hour, utc_min, utc_sec, utc_nanos, t_utc_ns)) {
            utc_mapper_->update(t_master_ns, t_utc_ns);
            nmea_have_date_ = true;
            nmea_year_ = utc_year;
            nmea_mon_ = utc_mon;
            nmea_day_ = utc_day;
          }
        } else if (!utc_mapper_->map(t_master_ns, t_utc_ns)) {
          t_utc_ns = utc_mapper_->fallback_from_anchor(t_master_ns);
        }

        const uint64_t received_wall_us = master_to_wall_us(t_rx_ns);
        if (got_euler) {
          push_orientation(t_utc_ns, pc_last, euler_deg, acc, gyro, received_wall_us);
        } else if (got_acc || got_gyro) {
          imu_count_.fetch_add(1);
          std::lock_guard<std::mutex> lk(status_mu_);
          last_imu_ = LatestImuStatus{acc, gyro, {0.0, 0.0, 0.0}};
        }

        if (got_latlon || got_alt || got_vel || got_utc || gnss_fix_type >= 0 || gnss_num_sv >= 0) {
          push_gnss_pose(t_utc_ns, pc_last, lat, lon, alt, vel, utc_valid, gnss_fix_type, gnss_num_sv, std::nullopt, received_wall_us);
        }
      }
    }

    ::close(fd);
    running_.store(false);
  }

  std::string dev_;
  int baud_;
  UtcMapper* utc_mapper_;
  std::array<double, 3> gnss_antenna_to_ego_m_{0.0, 0.0, 0.0};
  YawHeadingCsvLogger* yaw_heading_logger_{nullptr};
  std::atomic<bool> running_{false};
  std::atomic<bool> has_error_{false};
  std::string error_message_;
  std::thread thread_;
  OrientationRing orientation_ring_;
  PoseRing pose_ring_;
  std::atomic<uint64_t> imu_count_{0};
  std::atomic<uint64_t> orientation_count_{0};
  std::atomic<uint64_t> gnss_count_{0};
  RecentRateTracker imu_rate_;
  RecentRateTracker orientation_rate_;
  RecentRateTracker gnss_rate_;
  RecentRateTracker pose_rate_;
  mutable std::mutex status_mu_;
  mutable std::mutex callback_mu_;
  std::function<void()> data_callback_;
  std::optional<LatestImuStatus> last_imu_;
  std::optional<LatestGnssStatus> last_gnss_;
  std::optional<LatestGnssStatus> last_gnss_fix_;
  std::optional<std::array<double, 3>> origin_lla_;
  std::array<double, 3> origin_ecef_{0.0, 0.0, 0.0};
  bool nmea_have_date_{false};
  uint16_t nmea_year_{0};
  uint8_t nmea_mon_{0};
  uint8_t nmea_day_{0};
};
