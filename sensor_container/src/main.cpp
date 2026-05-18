#include "sensor_container_common.hpp"
#include "state_tracker.inc"
#include "camera_source.inc"
#include "sample_builder.inc"
#include "http_server.inc"

static void print_usage() {
  std::cout
      << "Usage: /workspace/alpamayo_sensor_container [--config PATH] [--port N]\n"
      << "  /latest  : latest valid planner NPZ sample or 503\n"
      << "  /healthz : minimal JSON health summary\n";
}

int main(int argc, char** argv) {
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  gst_init(&argc, &argv);

  SensorContainerConfig cfg = default_config();
  fs::path config_path = "/workspace/sensor_container/config/sensor_container.example.yaml";
  bool use_config = true;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--help") {
      print_usage();
      return 0;
    } else if (a == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (a == "--no-config") {
      use_config = false;
    } else if (a == "--port" && i + 1 < argc) {
      cfg.bind_port = std::atoi(argv[++i]);
    } else if (a == "--clip-id" && i + 1 < argc) {
      cfg.clip_id = argv[++i];
    } else {
      std::cerr << "Unknown arg: " << a << "\n";
      print_usage();
      return 1;
    }
  }

  if (use_config && fs::exists(config_path)) {
    std::string error;
    if (!load_yaml_config(config_path, cfg, error)) {
      log_error("failed to load config " + config_path.string() + ": " + error);
      return 1;
    }
    log_info("loaded config: " + config_path.string());
  }

  TimeAnchors anchors;
  UtcMapper utc_mapper(anchors);
  StateTracker state(
      cfg.imu_dev, cfg.imu_baud, &utc_mapper,
      {cfg.gnss_antenna_to_ego_x_m, cfg.gnss_antenna_to_ego_y_m, cfg.gnss_antenna_to_ego_z_m});
  if (!state.start()) {
    log_error("failed to start state tracker");
    return 1;
  }

  std::vector<std::shared_ptr<CameraSource>> cameras;
  cameras.reserve(cfg.cameras.size());
  for (const auto& cam_cfg : cfg.cameras) {
    auto cam = std::make_shared<CameraSource>(cam_cfg, &utc_mapper);
    if (!cam->start()) {
      log_error("failed to start camera_" + cam_cfg.name);
      return 1;
    }
    cameras.push_back(std::move(cam));
  }

  LatestSampleStore latest_store;
  SampleBuilder builder(cfg, cameras, &state, &latest_store);
  builder.start();

  HttpServer server(cfg, &cameras, &state, &latest_store);
  if (!server.start()) return 1;

  std::ostringstream camera_order_log;
  camera_order_log << "sensor container bring-up running. active cameras:";
  for (const auto& cam_cfg : cfg.cameras) {
    camera_order_log << " " << cam_cfg.name << "(" << cam_cfg.planner_camera_id << ")";
  }
  log_info(camera_order_log.str());
  log_info("waiting for live valid sample generation");

  uint64_t last_stats_ns = 0;
  while (!g_stop.load()) {
    bool any_error = state.has_error();
    for (auto& cam : cameras) {
      cam->poll_bus();
      any_error = any_error || cam->has_error();
    }
    uint64_t now_ns = now_master_ns();
    if (last_stats_ns == 0 || now_ns - last_stats_ns >= 10000000000ULL) {
      last_stats_ns = now_ns;
      std::ostringstream os;
      os << "camera_stats";
      for (const auto& cam : cameras) os << " " << cam->status_string();
      log_info(os.str());
      log_info(state.imu_status_string());
      log_info(state.gnss_status_string());
    }
    if (any_error) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  g_stop.store(true);
  server.stop();
  builder.stop();
  for (auto& cam : cameras) cam->stop();
  state.stop();
  return 0;
}
