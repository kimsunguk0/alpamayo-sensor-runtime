class HttpServer {
 public:
  HttpServer(const SensorContainerConfig& cfg,
             const std::vector<std::shared_ptr<CameraSource>>* cameras,
             const StateTracker* state,
             const LatestSampleStore* latest_store)
      : cfg_(cfg), cameras_(cameras), state_(state), latest_store_(latest_store) {}

  ~HttpServer() { stop(); }

  bool start() {
    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
      log_error("failed to create HTTP socket");
      return false;
    }
    int yes = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(cfg_.bind_port));
    if (cfg_.bind_host == "0.0.0.0") {
      addr.sin_addr.s_addr = INADDR_ANY;
    } else {
      if (::inet_pton(AF_INET, cfg_.bind_host.c_str(), &addr.sin_addr) != 1) {
        log_error("invalid bind_host: " + cfg_.bind_host);
        ::close(server_fd_);
        server_fd_ = -1;
        return false;
      }
    }
    if (::bind(server_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
      log_error("bind failed on port " + std::to_string(cfg_.bind_port) + " errno=" + std::to_string(errno));
      ::close(server_fd_);
      server_fd_ = -1;
      return false;
    }
    if (::listen(server_fd_, 16) != 0) {
      log_error("listen failed errno=" + std::to_string(errno));
      ::close(server_fd_);
      server_fd_ = -1;
      return false;
    }
    running_.store(true);
    thread_ = std::thread([this]() { thread_main(); });
    log_ready("HTTP server listening on " + cfg_.bind_host + ":" + std::to_string(cfg_.bind_port));
    return true;
  }

  void stop() {
    running_.store(false);
    if (server_fd_ >= 0) {
      ::shutdown(server_fd_, SHUT_RDWR);
      ::close(server_fd_);
      server_fd_ = -1;
    }
    if (thread_.joinable()) thread_.join();
  }

 private:
  static void send_all(int fd, const std::vector<uint8_t>& bytes) {
    size_t sent = 0;
    while (sent < bytes.size()) {
      ssize_t n = ::send(fd, bytes.data() + sent, bytes.size() - sent, 0);
      if (n <= 0) break;
      sent += static_cast<size_t>(n);
    }
  }

  static std::vector<uint8_t> make_response(const std::string& status,
                                            const std::string& content_type,
                                            const std::vector<uint8_t>& body,
                                            const std::vector<std::pair<std::string, std::string>>& headers = {}) {
    std::ostringstream os;
    os << "HTTP/1.1 " << status << "\r\n";
    os << "Content-Type: " << content_type << "\r\n";
    os << "Content-Length: " << body.size() << "\r\n";
    os << "Connection: close\r\n";
    for (const auto& [k, v] : headers) os << k << ": " << v << "\r\n";
    os << "\r\n";
    std::string head = os.str();
    std::vector<uint8_t> out(head.begin(), head.end());
    out.insert(out.end(), body.begin(), body.end());
    return out;
  }

  std::string build_health_json() const {
    std::ostringstream os;
    os << "{\"clip_id\":\"" << cfg_.clip_id << "\",";
    os << "\"imu\":\"" << state_->imu_status_string() << "\",";
    os << "\"gnss\":\"" << state_->gnss_status_string() << "\",";
    os << "\"cameras\":[";
    for (size_t i = 0; i < cameras_->size(); ++i) {
      if (i) os << ",";
      const auto& cam = (*cameras_)[i];
      os << "{\"name\":\"" << cam->config().name << "\",\"frames\":" << cam->frame_count()
         << ",\"fps\":" << std::fixed << std::setprecision(2) << cam->fps_estimate()
         << ",\"latest_utc_us\":" << cam->latest_utc_us() << "}";
    }
    os << "],\"latest_store\":" << latest_store_->health_json(cfg_.debug_diagnostics_enabled) << "}";
    return os.str();
  }

  void handle_client(int client_fd) {
    char buf[4096];
    ssize_t n = ::recv(client_fd, buf, sizeof(buf) - 1, 0);
    if (n <= 0) return;
    buf[n] = '\0';
    std::string req(buf);
    std::istringstream iss(req);
    std::string method, path;
    iss >> method >> path;
    if (method != "GET") {
      std::string body = "method not allowed\n";
      send_all(client_fd, make_response("405 Method Not Allowed", "text/plain", std::vector<uint8_t>(body.begin(), body.end())));
      return;
    }
    if (path == "/healthz") {
      std::string body = build_health_json();
      send_all(client_fd, make_response("200 OK", "application/json", std::vector<uint8_t>(body.begin(), body.end())));
      return;
    }
    if (path == "/latest") {
      PlannerSample sample;
      if (!latest_store_->get_latest(sample)) {
        std::string body = "no valid sample yet\n";
        send_all(client_fd, make_response("503 Service Unavailable", "text/plain", std::vector<uint8_t>(body.begin(), body.end())));
        return;
      }
      std::vector<std::pair<std::string, std::string>> headers{
          {"X-Sample-Sequence", std::to_string(sample.seq)},
          {"X-T0-US", std::to_string(sample.t0_us)},
          {"X-Clip-ID", sample.clip_id},
          {"X-Sample-Valid", "1"},
      };
      if (cfg_.debug_diagnostics_enabled) {
        const uint64_t now_us = now_utc_us_for_diagnostics();
        headers.push_back({"X-Debug-Diagnostics", "1"});
        headers.push_back({"X-Now-US", std::to_string(now_us)});
        headers.push_back({"X-Latest-Camera-UTC-US", std::to_string(sample.latest_camera_utc_us)});
        headers.push_back({"X-Latest-Camera-Pushed-Wall-US", std::to_string(sample.latest_camera_pushed_wall_us)});
        headers.push_back({"X-Latest-Camera-Pushed-Minus-UTC-US", std::to_string(signed_delta_us(sample.latest_camera_pushed_wall_us, sample.latest_camera_utc_us))});
        headers.push_back({"X-Latest-Pose-US", std::to_string(sample.latest_pose_us)});
        headers.push_back({"X-Latest-Pose-Received-Wall-US", std::to_string(sample.latest_pose_received_wall_us)});
        headers.push_back({"X-Latest-Pose-Pushed-Wall-US", std::to_string(sample.latest_pose_pushed_wall_us)});
        headers.push_back({"X-Latest-Pose-Received-Minus-UTC-US", std::to_string(signed_delta_us(sample.latest_pose_received_wall_us, sample.latest_pose_us))});
        headers.push_back({"X-Latest-Pose-Pushed-Minus-UTC-US", std::to_string(signed_delta_us(sample.latest_pose_pushed_wall_us, sample.latest_pose_us))});
        headers.push_back({"X-Latest-Orientation-US", std::to_string(sample.latest_orientation_us)});
        headers.push_back({"X-Latest-Orientation-Received-Wall-US", std::to_string(sample.latest_orientation_received_wall_us)});
        headers.push_back({"X-Latest-Orientation-Pushed-Wall-US", std::to_string(sample.latest_orientation_pushed_wall_us)});
        headers.push_back({"X-Latest-Orientation-Received-Minus-UTC-US", std::to_string(signed_delta_us(sample.latest_orientation_received_wall_us, sample.latest_orientation_us))});
        headers.push_back({"X-Latest-Orientation-Pushed-Minus-UTC-US", std::to_string(signed_delta_us(sample.latest_orientation_pushed_wall_us, sample.latest_orientation_us))});
        headers.push_back({"X-Newest-Possible-US", std::to_string(sample.newest_possible_us)});
        headers.push_back({"X-Newest-Possible-Minus-T0-US", std::to_string(signed_delta_us(sample.newest_possible_us, sample.t0_us))});
        headers.push_back({"X-Now-Minus-T0-US", std::to_string(signed_delta_us(now_us, sample.t0_us))});
        headers.push_back({"X-Now-Minus-Latest-Camera-UTC-US", std::to_string(signed_delta_us(now_us, sample.latest_camera_utc_us))});
        headers.push_back({"X-Now-Minus-Sample-Built-US", std::to_string(signed_delta_us(now_us, sample.sample_built_at_us))});
        headers.push_back({"X-Now-Minus-Sample-Published-US", std::to_string(signed_delta_us(now_us, sample.sample_published_at_us))});
        headers.push_back({"X-Sample-Build-Started-At-US", std::to_string(sample.sample_build_started_at_us)});
        headers.push_back({"X-Sample-Build-Started-Minus-T0-US", std::to_string(signed_delta_us(sample.sample_build_started_at_us, sample.t0_us))});
        headers.push_back({"X-Sample-Build-Started-Minus-Newest-Possible-US", std::to_string(signed_delta_us(sample.sample_build_started_at_us, sample.newest_possible_us))});
        headers.push_back({"X-Sample-Built-At-US", std::to_string(sample.sample_built_at_us)});
        headers.push_back({"X-Sample-Published-At-US", std::to_string(sample.sample_published_at_us)});
        headers.push_back({"X-Sample-Built-Minus-T0-US", std::to_string(signed_delta_us(sample.sample_built_at_us, sample.t0_us))});
        headers.push_back({"X-Sample-Published-Minus-T0-US", std::to_string(signed_delta_us(sample.sample_published_at_us, sample.t0_us))});
        headers.push_back({"X-Pose-T0-Err-US", std::to_string(sample.pose_t0_err_us)});
        headers.push_back({"X-Orientation-T0-Err-US", std::to_string(sample.orientation_t0_err_us)});
        headers.push_back({"X-Camera-State-Skew-US", std::to_string(sample.camera_state_skew_us)});
      }
      send_all(client_fd, make_response("200 OK", "application/octet-stream", sample.npz_bytes, headers));
      return;
    }
    std::string body = "not found\n";
    send_all(client_fd, make_response("404 Not Found", "text/plain", std::vector<uint8_t>(body.begin(), body.end())));
  }

  void thread_main() {
    while (running_.load() && !g_stop.load()) {
      sockaddr_in client_addr{};
      socklen_t client_len = sizeof(client_addr);
      int client_fd = ::accept(server_fd_, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
      if (client_fd < 0) {
        if (!running_.load()) break;
        if (errno == EINTR) continue;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      handle_client(client_fd);
      ::shutdown(client_fd, SHUT_RDWR);
      ::close(client_fd);
    }
  }

  SensorContainerConfig cfg_;
  const std::vector<std::shared_ptr<CameraSource>>* cameras_;
  const StateTracker* state_;
  const LatestSampleStore* latest_store_;
  int server_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread thread_;
};
