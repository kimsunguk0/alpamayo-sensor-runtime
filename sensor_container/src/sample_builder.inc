static void append_u16_le(std::vector<uint8_t>& out, uint16_t value) {
  out.push_back(static_cast<uint8_t>(value & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}
static void append_u32_le(std::vector<uint8_t>& out, uint32_t value) {
  out.push_back(static_cast<uint8_t>(value & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

template <typename T>
static void append_pod_bytes(std::vector<uint8_t>& out, const T& value) {
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&value);
  out.insert(out.end(), p, p + sizeof(T));
}

template <typename T>
static std::vector<uint8_t> make_npy_numeric(const std::string& descr, const std::vector<size_t>& shape, const std::vector<T>& data) {
  std::ostringstream header;
  header << "{'descr': '" << descr << "', 'fortran_order': False, 'shape': (";
  for (size_t i = 0; i < shape.size(); ++i) {
    if (i) header << ", ";
    header << shape[i];
  }
  if (shape.size() == 1) header << ",";
  header << "), }";
  std::string header_str = header.str();
  size_t preamble = 10;
  size_t padding = 16 - ((preamble + header_str.size() + 1) % 16);
  if (padding == 16) padding = 0;
  header_str.append(padding, ' ');
  header_str.push_back('\n');

  std::vector<uint8_t> out;
  out.push_back(0x93);
  out.insert(out.end(), {'N', 'U', 'M', 'P', 'Y'});
  out.push_back(1);
  out.push_back(0);
  append_u16_le(out, static_cast<uint16_t>(header_str.size()));
  out.insert(out.end(), header_str.begin(), header_str.end());
  const uint8_t* ptr = reinterpret_cast<const uint8_t*>(data.data());
  out.insert(out.end(), ptr, ptr + data.size() * sizeof(T));
  return out;
}

static std::vector<uint8_t> make_npy_unicode(const std::vector<size_t>& shape, const std::vector<std::string>& strings) {
  size_t max_chars = 1;
  for (const auto& s : strings) max_chars = std::max(max_chars, s.size());
  std::ostringstream descr;
  descr << "<U" << max_chars;

  std::ostringstream header;
  header << "{'descr': '" << descr.str() << "', 'fortran_order': False, 'shape': (";
  for (size_t i = 0; i < shape.size(); ++i) {
    if (i) header << ", ";
    header << shape[i];
  }
  if (shape.size() == 1) header << ",";
  header << "), }";
  std::string header_str = header.str();
  size_t preamble = 10;
  size_t padding = 16 - ((preamble + header_str.size() + 1) % 16);
  if (padding == 16) padding = 0;
  header_str.append(padding, ' ');
  header_str.push_back('\n');

  std::vector<uint8_t> out;
  out.push_back(0x93);
  out.insert(out.end(), {'N', 'U', 'M', 'P', 'Y'});
  out.push_back(1);
  out.push_back(0);
  append_u16_le(out, static_cast<uint16_t>(header_str.size()));
  out.insert(out.end(), header_str.begin(), header_str.end());
  for (const auto& s : strings) {
    for (size_t i = 0; i < max_chars; ++i) {
      uint32_t cp = (i < s.size()) ? static_cast<unsigned char>(s[i]) : 0;
      append_pod_bytes(out, cp);
    }
  }
  return out;
}

struct NpzEntry {
  std::string name;
  std::vector<uint8_t> bytes;
};

static std::vector<uint8_t> build_npz(const std::vector<NpzEntry>& entries) {
  std::vector<uint8_t> out;
  struct CentralItem {
    std::string name;
    uint32_t crc = 0;
    uint32_t size = 0;
    uint32_t offset = 0;
  };
  std::vector<CentralItem> central;
  central.reserve(entries.size());

  for (const auto& entry : entries) {
    CentralItem item;
    item.name = entry.name;
    item.size = static_cast<uint32_t>(entry.bytes.size());
    item.crc = static_cast<uint32_t>(crc32(0L, reinterpret_cast<const Bytef*>(entry.bytes.data()), entry.bytes.size()));
    item.offset = static_cast<uint32_t>(out.size());

    append_u32_le(out, 0x04034b50);
    append_u16_le(out, 20);
    append_u16_le(out, 0);
    append_u16_le(out, 0);
    append_u16_le(out, 0);
    append_u16_le(out, 0);
    append_u32_le(out, item.crc);
    append_u32_le(out, item.size);
    append_u32_le(out, item.size);
    append_u16_le(out, static_cast<uint16_t>(item.name.size()));
    append_u16_le(out, 0);
    out.insert(out.end(), item.name.begin(), item.name.end());
    out.insert(out.end(), entry.bytes.begin(), entry.bytes.end());
    central.push_back(item);
  }

  const uint32_t central_offset = static_cast<uint32_t>(out.size());
  for (const auto& item : central) {
    append_u32_le(out, 0x02014b50);
    append_u16_le(out, 20);
    append_u16_le(out, 20);
    append_u16_le(out, 0);
    append_u16_le(out, 0);
    append_u16_le(out, 0);
    append_u16_le(out, 0);
    append_u32_le(out, item.crc);
    append_u32_le(out, item.size);
    append_u32_le(out, item.size);
    append_u16_le(out, static_cast<uint16_t>(item.name.size()));
    append_u16_le(out, 0);
    append_u16_le(out, 0);
    append_u16_le(out, 0);
    append_u16_le(out, 0);
    append_u32_le(out, 0);
    append_u32_le(out, item.offset);
    out.insert(out.end(), item.name.begin(), item.name.end());
  }
  const uint32_t central_size = static_cast<uint32_t>(out.size()) - central_offset;
  append_u32_le(out, 0x06054b50);
  append_u16_le(out, 0);
  append_u16_le(out, 0);
  append_u16_le(out, static_cast<uint16_t>(central.size()));
  append_u16_le(out, static_cast<uint16_t>(central.size()));
  append_u32_le(out, central_size);
  append_u32_le(out, central_offset);
  append_u16_le(out, 0);
  return out;
}

struct PlannerSample {
  uint64_t seq = 0;
  uint64_t t0_us = 0;
  std::string clip_id;
  std::vector<uint8_t> npz_bytes;
};

class LatestSampleStore {
 public:
  void set_valid(PlannerSample sample, std::string summary_json) {
    std::lock_guard<std::mutex> lk(m_);
    latest_ = std::move(sample);
    have_valid_ = true;
    summary_json_ = std::move(summary_json);
  }

  void set_invalid_attempt(const std::string& reason) {
    std::lock_guard<std::mutex> lk(m_);
    last_invalid_reason_ = reason;
  }

  bool get_latest(PlannerSample& out) const {
    std::lock_guard<std::mutex> lk(m_);
    if (!have_valid_) return false;
    out = latest_;
    return true;
  }

  std::string health_json() const {
    std::lock_guard<std::mutex> lk(m_);
    std::ostringstream os;
    os << "{\"have_valid_sample\":" << (have_valid_ ? "true" : "false")
       << ",\"latest_seq\":" << latest_.seq
       << ",\"latest_t0_us\":" << latest_.t0_us
       << ",\"last_invalid_reason\":\"" << json_escape(last_invalid_reason_) << "\"";
    if (!summary_json_.empty()) os << ",\"latest_sample_summary\":" << summary_json_;
    os << "}";
    return os.str();
  }

 private:
  static std::string json_escape(const std::string& s) {
    std::ostringstream os;
    for (char ch : s) {
      switch (ch) {
        case '\\': os << "\\\\"; break;
        case '"': os << "\\\""; break;
        case '\n': os << "\\n"; break;
        case '\r': os << "\\r"; break;
        case '\t': os << "\\t"; break;
        default: os << ch; break;
      }
    }
    return os.str();
  }

  mutable std::mutex m_;
  PlannerSample latest_;
  bool have_valid_{false};
  std::string last_invalid_reason_;
  std::string summary_json_;
};

class SampleBuilder {
 public:
  SampleBuilder(const SensorContainerConfig& cfg,
                const std::vector<std::shared_ptr<CameraSource>>& cameras,
                const StateTracker* state,
                LatestSampleStore* store)
      : cfg_(cfg), cameras_(cameras), state_(state), store_(store), resized_cache_(cameras.size()) {}

  ~SampleBuilder() {
    stop();
    if (sws_ctx_) sws_freeContext(sws_ctx_);
  }

  void start() {
    running_.store(true);
    thread_ = std::thread([this]() { thread_main(); });
  }

  void stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
  }

 private:
  struct CachedRgbFrame {
    uint64_t seq = 0;
    std::shared_ptr<std::vector<uint8_t>> rgb;
  };

  std::shared_ptr<std::vector<uint8_t>> get_cached_rgb(size_t cam_idx, uint64_t seq) const {
    for (const auto& item : resized_cache_[cam_idx]) {
      if (item.seq == seq) return item.rgb;
    }
    return nullptr;
  }

  void put_cached_rgb(size_t cam_idx, uint64_t seq, std::shared_ptr<std::vector<uint8_t>> rgb) {
    auto& cache = resized_cache_[cam_idx];
    for (auto it = cache.begin(); it != cache.end(); ++it) {
      if (it->seq == seq) {
        it->rgb = std::move(rgb);
        return;
      }
    }
    cache.push_back({seq, std::move(rgb)});
    while (cache.size() > 16) cache.pop_front();
  }

  bool append_rgb_hwc_to_chw(const std::vector<uint8_t>& rgb_hwc, std::vector<uint8_t>& image_frames) {
    const int width = cfg_.cameras[0].planner_width;
    const int height = cfg_.cameras[0].planner_height;
    const size_t expected = static_cast<size_t>(width) * height * 3;
    if (rgb_hwc.size() != expected) return false;

    const size_t base = image_frames.size();
    image_frames.resize(base + expected);
    uint8_t* dst = image_frames.data() + base;
    const uint8_t* src = rgb_hwc.data();
    const size_t plane = static_cast<size_t>(width) * height;

    for (size_t i = 0; i < plane; ++i) {
      dst[i] = src[i * 3 + 0];
      dst[plane + i] = src[i * 3 + 1];
      dst[plane * 2 + i] = src[i * 3 + 2];
    }

    if (!layout_self_check_done_) {
      std::vector<uint8_t> decoded_hwc(expected, 0);
      for (size_t i = 0; i < plane; ++i) {
        decoded_hwc[i * 3 + 0] = dst[i];
        decoded_hwc[i * 3 + 1] = dst[plane + i];
        decoded_hwc[i * 3 + 2] = dst[plane * 2 + i];
      }
      if (decoded_hwc != rgb_hwc) {
        store_->set_invalid_attempt("image_frames CHW packing self-check failed");
        log_error("image_frames CHW packing self-check failed");
        return false;
      }
      layout_self_check_done_ = true;
      log_info("image_frames layout self-check passed (RGB HWC -> CHW contiguous)");
    }
    return true;
  }

  std::shared_ptr<std::vector<uint8_t>> resize_frame_to_planner_rgb(size_t cam_idx, const ImageFrame& frame) {
    if (!frame.data) return nullptr;
    if (frame.format == ImagePixelFormat::kRgb24 &&
        frame.width == cfg_.cameras[0].planner_width &&
        frame.height == cfg_.cameras[0].planner_height) {
      if (frame.bytes) return frame.bytes;
      auto rgb = std::make_shared<std::vector<uint8_t>>(frame.data, frame.data + frame.data_size);
      put_cached_rgb(cam_idx, frame.seq, rgb);
      return rgb;
    }
    if (auto cached = get_cached_rgb(cam_idx, frame.seq)) return cached;

    AVPixelFormat src_fmt = AV_PIX_FMT_NONE;
    switch (frame.format) {
      case ImagePixelFormat::kUyvy422:
        src_fmt = AV_PIX_FMT_UYVY422;
        break;
      case ImagePixelFormat::kRgba32:
        src_fmt = AV_PIX_FMT_RGBA;
        break;
      case ImagePixelFormat::kRgb24:
        src_fmt = AV_PIX_FMT_RGB24;
        break;
      default:
        return nullptr;
    }

    sws_ctx_ = sws_getCachedContext(
        sws_ctx_,
        frame.width,
        frame.height,
        src_fmt,
        cfg_.cameras[0].planner_width,
        cfg_.cameras[0].planner_height,
        AV_PIX_FMT_RGB24,
        SWS_BICUBIC,
        nullptr,
        nullptr,
        nullptr);
    if (!sws_ctx_) return nullptr;

    auto rgb = std::make_shared<std::vector<uint8_t>>(
        static_cast<size_t>(cfg_.cameras[0].planner_width) * cfg_.cameras[0].planner_height * 3);
    const uint8_t* src_slices[4] = {frame.data, nullptr, nullptr, nullptr};
    int src_stride[4] = {frame.stride, 0, 0, 0};
    uint8_t* dst_slices[4] = {rgb->data(), nullptr, nullptr, nullptr};
    int dst_stride[4] = {cfg_.cameras[0].planner_width * 3, 0, 0, 0};
    int out_rows = sws_scale(sws_ctx_, src_slices, src_stride, 0, frame.height, dst_slices, dst_stride);
    if (out_rows != cfg_.cameras[0].planner_height) return nullptr;

    put_cached_rgb(cam_idx, frame.seq, rgb);
    return rgb;
  }

  void thread_main() {
    uint64_t last_log_ns = 0;
    while (running_.load() && !g_stop.load()) {
      uint64_t min_latest_camera_us = std::numeric_limits<uint64_t>::max();
      bool cameras_ready = true;
      for (const auto& cam : cameras_) {
        min_latest_camera_us = std::min(min_latest_camera_us, cam->latest_utc_us());
        cameras_ready = cameras_ready && (cam->frame_count() >= 30);
      }
      uint64_t latest_pose_us = state_->latest_pose_utc_us();
      bool pose_ready = latest_pose_us != 0;
      uint64_t now_ns = now_master_ns();
      if (!cameras_ready || (!pose_ready && !cfg_.allow_poseless_test_mode) ||
          min_latest_camera_us == std::numeric_limits<uint64_t>::max()) {
        if (last_log_ns == 0 || now_ns - last_log_ns > 2000000000ULL) {
          last_log_ns = now_ns;
          std::ostringstream os;
          os << "sample builder waiting: cameras_ready=" << (cameras_ready ? "yes" : "no")
             << " latest_pose_us=" << latest_pose_us
             << " poseless_test_mode=" << (cfg_.allow_poseless_test_mode ? "on" : "off");
          store_->set_invalid_attempt(os.str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }

      uint64_t newest_possible_us = pose_ready ? std::min(min_latest_camera_us, latest_pose_us) : min_latest_camera_us;
      uint64_t t0_us = (newest_possible_us / 100000ULL) * 100000ULL;
      if (t0_us == 0 || t0_us <= last_built_t0_us_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }
      if (build_one(t0_us)) {
        last_built_t0_us_ = t0_us;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  bool build_one(uint64_t t0_us) {
    static const std::array<int64_t, 4> kFrameOffsetsUs{-300000, -200000, -100000, 0};
    const size_t camera_count = cameras_.size();
    if (camera_count == 0 || cfg_.cameras.empty()) {
      store_->set_invalid_attempt("no cameras configured");
      return false;
    }

    std::vector<uint8_t> image_frames;
    image_frames.reserve(camera_count * kFrameOffsetsUs.size() * 3 * cfg_.cameras[0].planner_width * cfg_.cameras[0].planner_height);
    std::vector<float> relative_timestamps;
    std::vector<int64_t> absolute_timestamps;
    relative_timestamps.reserve(camera_count * kFrameOffsetsUs.size());
    absolute_timestamps.reserve(camera_count * kFrameOffsetsUs.size());

    PoseSample pose_t0{};
    uint64_t pose_t0_err_us = 0;
    const bool have_pose_t0 =
        state_->pose_ring().interpolate_xyz(t0_us, pose_t0, pose_t0_err_us) &&
        pose_t0_err_us <= cfg_.localization_stale_us;
    if (!have_pose_t0) {
      if (!cfg_.allow_poseless_test_mode) {
        store_->set_invalid_attempt("missing pose at t0 or pose stale");
        return false;
      }
      pose_t0.utc_us = t0_us;
    }

    OrientationSample orientation_t0{};
    uint64_t orientation_t0_err_us = 0;
    const bool have_orientation_t0 =
        state_->orientation_ring().interpolate(t0_us, orientation_t0, orientation_t0_err_us) &&
        orientation_t0_err_us <= cfg_.localization_stale_us;
    if (!have_orientation_t0 && !cfg_.allow_poseless_test_mode) {
      store_->set_invalid_attempt("missing orientation at t0 or orientation stale");
      return false;
    }
    const std::array<double, 9> world_from_ego_t0 =
        have_orientation_t0
            ? rotation_matrix_from_ypr_rad(
                  orientation_t0.yaw_deg * M_PI / 180.0,
                  orientation_t0.pitch_deg * M_PI / 180.0,
                  orientation_t0.roll_deg * M_PI / 180.0)
            : std::array<double, 9>{
                  1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0,
              };

    for (size_t cam_idx = 0; cam_idx < camera_count; ++cam_idx) {
      for (size_t t_idx = 0; t_idx < kFrameOffsetsUs.size(); ++t_idx) {
        uint64_t target_us = static_cast<uint64_t>(static_cast<int64_t>(t0_us) + kFrameOffsetsUs[t_idx]);
        ImageFrame frame;
        uint64_t err_us = 0;
        if (!cameras_[cam_idx]->ring().nearest(target_us, frame, err_us) || err_us > cfg_.camera_target_mismatch_us) {
          std::ostringstream os;
          os << "camera " << cameras_[cam_idx]->config().name << " missing frame within threshold at offset " << kFrameOffsetsUs[t_idx] << "us";
          store_->set_invalid_attempt(os.str());
          return false;
        }
        auto rgb = resize_frame_to_planner_rgb(cam_idx, frame);
        if (!rgb) {
          std::ostringstream os;
          os << "camera " << cameras_[cam_idx]->config().name << " resize failed for frame seq=" << frame.seq;
          store_->set_invalid_attempt(os.str());
          return false;
        }
        relative_timestamps.push_back(static_cast<float>((static_cast<int64_t>(frame.utc_us) - static_cast<int64_t>(t0_us)) / 1e6));
        absolute_timestamps.push_back(static_cast<int64_t>(frame.utc_us));
        if (!append_rgb_hwc_to_chw(*rgb, image_frames)) {
          std::ostringstream os;
          os << "camera " << cameras_[cam_idx]->config().name << " CHW pack failed for frame seq=" << frame.seq;
          store_->set_invalid_attempt(os.str());
          return false;
        }
      }
    }

    size_t reference_camera_idx = 0;
    for (size_t cam_idx = 0; cam_idx < camera_count; ++cam_idx) {
      if (cameras_[cam_idx]->config().name == "front") {
        reference_camera_idx = cam_idx;
        break;
      }
    }
    uint64_t reference_frame_err_us = 0;
    ImageFrame reference_t0{};
    if (!cameras_[reference_camera_idx]->ring().nearest(t0_us, reference_t0, reference_frame_err_us)) {
      store_->set_invalid_attempt("reference camera t0 frame unavailable");
      return false;
    }
    uint64_t skew_us =
        (reference_t0.utc_us > t0_us) ? (reference_t0.utc_us - t0_us) : (t0_us - reference_t0.utc_us);
    if ((have_pose_t0 || have_orientation_t0) && skew_us > cfg_.camera_state_skew_us) {
      store_->set_invalid_attempt("camera-state skew at t0 exceeds threshold");
      return false;
    }

    std::vector<float> ego_history_xyz;
    std::vector<float> ego_history_rot;
    ego_history_xyz.reserve(16 * 3);
    ego_history_rot.reserve(16 * 9);

    const auto R0T = transpose33(world_from_ego_t0);
    for (int i = 0; i < 16; ++i) {
      uint64_t query_us = t0_us - static_cast<uint64_t>((15 - i) * 100000);
      PoseSample pose{};
      uint64_t pose_err_us = 0;
      const bool have_pose_hist =
          state_->pose_ring().interpolate_xyz(query_us, pose, pose_err_us) &&
          pose_err_us <= cfg_.localization_stale_us;
      if (!have_pose_hist) {
        if (!cfg_.allow_poseless_test_mode) {
          store_->set_invalid_attempt("pose history missing or stale");
          return false;
        }
        pose.utc_us = query_us;
      }

      OrientationSample orientation{};
      uint64_t orientation_err_us = 0;
      const bool have_orientation_hist =
          state_->orientation_ring().interpolate(query_us, orientation, orientation_err_us) &&
          orientation_err_us <= cfg_.localization_stale_us;
      if (!have_orientation_hist && !cfg_.allow_poseless_test_mode) {
        store_->set_invalid_attempt("orientation history missing or stale");
        return false;
      }

      std::array<double, 3> delta_world{
          pose.enu_xyz[0] - pose_t0.enu_xyz[0],
          pose.enu_xyz[1] - pose_t0.enu_xyz[1],
          pose.enu_xyz[2] - pose_t0.enu_xyz[2],
      };
      std::array<double, 3> delta_local = matvec33(R0T, delta_world);
      ego_history_xyz.push_back(static_cast<float>(delta_local[0]));
      ego_history_xyz.push_back(static_cast<float>(delta_local[1]));
      ego_history_xyz.push_back(static_cast<float>(delta_local[2]));

      std::array<double, 9> world_from_ego_hist{
          1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0,
      };
      if (have_orientation_hist) {
        world_from_ego_hist = rotation_matrix_from_ypr_rad(
            orientation.yaw_deg * M_PI / 180.0,
            orientation.pitch_deg * M_PI / 180.0,
            orientation.roll_deg * M_PI / 180.0);
      }
      std::array<double, 9> local_rot = matmul33(R0T, world_from_ego_hist);
      for (double v : local_rot) ego_history_rot.push_back(static_cast<float>(v));
    }

    std::vector<int32_t> camera_indices;
    std::vector<std::string> camera_order_arr;
    camera_indices.reserve(camera_count);
    camera_order_arr.reserve(camera_count);
    for (const auto& camera : cameras_) {
      camera_indices.push_back(camera->config().planner_camera_id);
      camera_order_arr.push_back(camera->config().name);
    }
    std::vector<int64_t> t0_arr{static_cast<int64_t>(t0_us)};
    std::vector<float> fixed_delta_arr{static_cast<float>(cfg_.fixed_delta_seconds)};
    std::vector<std::string> clip_id_arr{cfg_.clip_id};

    std::vector<NpzEntry> entries;
    entries.push_back({"image_frames.npy", make_npy_numeric<uint8_t>("|u1", {camera_count, 4, 3, static_cast<size_t>(cfg_.cameras[0].planner_height), static_cast<size_t>(cfg_.cameras[0].planner_width)}, image_frames)});
    entries.push_back({"camera_indices.npy", make_npy_numeric<int32_t>("<i4", {camera_count}, camera_indices)});
    entries.push_back({"ego_history_xyz.npy", make_npy_numeric<float>("<f4", {1, 1, 16, 3}, ego_history_xyz)});
    entries.push_back({"ego_history_rot.npy", make_npy_numeric<float>("<f4", {1, 1, 16, 3, 3}, ego_history_rot)});
    entries.push_back({"relative_timestamps.npy", make_npy_numeric<float>("<f4", {camera_count, 4}, relative_timestamps)});
    entries.push_back({"absolute_timestamps.npy", make_npy_numeric<int64_t>("<i8", {camera_count, 4}, absolute_timestamps)});
    entries.push_back({"t0_us.npy", make_npy_numeric<int64_t>("<i8", {1}, t0_arr)});
    entries.push_back({"fixed_delta_seconds.npy", make_npy_numeric<float>("<f4", {1}, fixed_delta_arr)});
    entries.push_back({"clip_id.npy", make_npy_unicode({1}, clip_id_arr)});
    entries.push_back({"camera_order.npy", make_npy_unicode({camera_count}, camera_order_arr)});

    PlannerSample sample;
    sample.seq = sample_seq_.fetch_add(1) + 1;
    sample.t0_us = t0_us;
    sample.clip_id = cfg_.clip_id;
    sample.npz_bytes = build_npz(entries);

    std::ostringstream summary;
    summary << "{\"camera_state_skew_us\":" << skew_us
            << ",\"pose_t0_err_us\":" << pose_t0_err_us
            << ",\"orientation_t0_err_us\":" << orientation_t0_err_us
            << ",\"npz_bytes\":" << sample.npz_bytes.size() << "}";
    store_->set_valid(std::move(sample), summary.str());
    return true;
  }

  SensorContainerConfig cfg_;
  std::vector<std::shared_ptr<CameraSource>> cameras_;
  const StateTracker* state_;
  LatestSampleStore* store_;
  std::atomic<bool> running_{false};
  std::thread thread_;
  std::atomic<uint64_t> sample_seq_{0};
  uint64_t last_built_t0_us_{0};
  SwsContext* sws_ctx_{nullptr};
  bool layout_self_check_done_{false};
  std::vector<std::deque<CachedRgbFrame>> resized_cache_;
};
