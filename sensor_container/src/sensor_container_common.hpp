#pragma once

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <nvbufsurface.h>
extern "C" {
#include <libswscale/swscale.h>
#include <libavutil/pixfmt.h>
}

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <linux/videodev2.h>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <optional>
#include <poll.h>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <utility>
#include <vector>

#include <zlib.h>

namespace fs = std::filesystem;

static std::atomic<bool> g_stop{false};
static void on_signal(int) { g_stop.store(true); }

static int xioctl(int fd, unsigned long req, void* arg) {
  int r;
  do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
  return r;
}

static inline uint64_t now_master_ns() {
  timespec ts{};
#ifdef CLOCK_MONOTONIC_RAW
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
#else
  clock_gettime(CLOCK_MONOTONIC, &ts);
#endif
  return uint64_t(ts.tv_sec) * 1000000000ULL + uint64_t(ts.tv_nsec);
}

static inline uint64_t now_realtime_ns() {
  timespec ts{};
  clock_gettime(CLOCK_REALTIME, &ts);
  return uint64_t(ts.tv_sec) * 1000000000ULL + uint64_t(ts.tv_nsec);
}

static inline bool plausible_utc_ns(uint64_t utc_ns) {
  constexpr uint64_t kOneYearNs = 365ULL * 24ULL * 3600ULL * 1000000000ULL;
  constexpr uint64_t kMinUnixNs = 1451606400ULL * 1000000000ULL;  // 2016-01-01
  constexpr uint64_t kMaxUnixNs = 2208988800ULL * 1000000000ULL;  // 2040-01-01
  if (utc_ns < kMinUnixNs || utc_ns > kMaxUnixNs) return false;
  const uint64_t now_ns = now_realtime_ns();
  const uint64_t delta = (utc_ns > now_ns) ? (utc_ns - now_ns) : (now_ns - utc_ns);
  return delta <= kOneYearNs;
}

static inline bool plausible_gps_week(uint16_t gps_week) {
  return gps_week >= 1900 && gps_week <= 3200;
}

static void log_line(const char* level, const std::string& message) {
  std::cout << "[" << level << "] " << message << "\n";
}

static void log_info(const std::string& message) { log_line("info", message); }
static void log_warn(const std::string& message) { log_line("warn", message); }
static void log_error(const std::string& message) { log_line("error", message); }
static void log_ready(const std::string& message) { log_line("ready", message); }

static std::string trim_copy(const std::string& value) {
  const size_t first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const size_t last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

static std::string to_lower_copy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

static std::string strip_yaml_comment(const std::string& line) {
  bool in_single = false;
  bool in_double = false;
  for (size_t i = 0; i < line.size(); ++i) {
    char ch = line[i];
    if (ch == '\'' && !in_double) {
      in_single = !in_single;
    } else if (ch == '"' && !in_single) {
      in_double = !in_double;
    } else if (ch == '#' && !in_single && !in_double) {
      return line.substr(0, i);
    }
  }
  return line;
}

static std::string yaml_unquote(std::string value) {
  value = trim_copy(value);
  if (value.size() >= 2) {
    if ((value.front() == '"' && value.back() == '"') ||
        (value.front() == '\'' && value.back() == '\'')) {
      value = value.substr(1, value.size() - 2);
    }
  }
  return value;
}

static bool parse_yaml_int(const std::string& text, int& out) {
  const std::string value = trim_copy(text);
  if (value.empty()) return false;
  char* end = nullptr;
  long parsed = std::strtol(value.c_str(), &end, 10);
  if (!end || *end != '\0') return false;
  out = static_cast<int>(parsed);
  return true;
}

static bool parse_yaml_u64(const std::string& text, uint64_t& out) {
  const std::string value = trim_copy(text);
  if (value.empty()) return false;
  char* end = nullptr;
  unsigned long long parsed = std::strtoull(value.c_str(), &end, 10);
  if (!end || *end != '\0') return false;
  out = static_cast<uint64_t>(parsed);
  return true;
}

static bool parse_yaml_double(const std::string& text, double& out) {
  const std::string value = trim_copy(text);
  if (value.empty()) return false;
  char* end = nullptr;
  out = std::strtod(value.c_str(), &end);
  return end && *end == '\0';
}

static bool parse_yaml_bool(const std::string& text, bool& out) {
  const std::string value = to_lower_copy(trim_copy(text));
  if (value == "true" || value == "yes" || value == "on" || value == "1") {
    out = true;
    return true;
  }
  if (value == "false" || value == "no" || value == "off" || value == "0") {
    out = false;
    return true;
  }
  return false;
}

static std::vector<std::string> split_csv_items(const std::string& value) {
  std::vector<std::string> items;
  size_t start = 0;
  while (start <= value.size()) {
    size_t comma = value.find(',', start);
    std::string token = yaml_unquote(value.substr(start, comma == std::string::npos ? std::string::npos : comma - start));
    token = trim_copy(token);
    if (!token.empty()) items.push_back(token);
    if (comma == std::string::npos) break;
    start = comma + 1;
  }
  return items;
}

static std::string normalize_camera_device(std::string device) {
  device = trim_copy(device);
  if (device.rfind("dev/video", 0) == 0) device.insert(device.begin(), '/');
  return device;
}

static std::string normalize_camera_name(std::string name) {
  name = trim_copy(name);
  if (name == "front_wide") return "front";
  return name;
}

struct SimpleYamlConfig {
  std::unordered_map<std::string, std::string> scalars;
  std::unordered_map<std::string, std::vector<std::string>> lists;
};

static bool parse_simple_yaml_file(const fs::path& path, SimpleYamlConfig& out, std::string& error) {
  std::ifstream ifs(path);
  if (!ifs) {
    error = "failed to open config file";
    return false;
  }

  out = SimpleYamlConfig{};
  std::string current_list_key;
  std::string line;
  int line_no = 0;
  while (std::getline(ifs, line)) {
    ++line_no;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    const std::string uncommented = strip_yaml_comment(line);
    const std::string trimmed = trim_copy(uncommented);
    if (trimmed.empty()) continue;

    const size_t indent = uncommented.find_first_not_of(" ");
    const size_t leading_spaces = (indent == std::string::npos) ? uncommented.size() : indent;
    if (leading_spaces == 0) {
      current_list_key.clear();
      const size_t colon = trimmed.find(':');
      if (colon == std::string::npos) {
        error = "invalid config line " + std::to_string(line_no) + ": missing ':'";
        return false;
      }
      const std::string key = trim_copy(trimmed.substr(0, colon));
      const std::string value = trim_copy(trimmed.substr(colon + 1));
      if (key.empty()) {
        error = "invalid config line " + std::to_string(line_no) + ": empty key";
        return false;
      }
      if (value.empty()) {
        current_list_key = key;
        out.lists[key];
      } else {
        out.scalars[key] = yaml_unquote(value);
      }
      continue;
    }

    if (current_list_key.empty()) {
      error = "invalid config line " + std::to_string(line_no) + ": unexpected indentation";
      return false;
    }
    if (trimmed.rfind("- ", 0) != 0) {
      error = "invalid config line " + std::to_string(line_no) + ": expected list item";
      return false;
    }
    const std::string item = yaml_unquote(trimmed.substr(2));
    if (item.empty()) {
      error = "invalid config line " + std::to_string(line_no) + ": empty list item";
      return false;
    }
    out.lists[current_list_key].push_back(item);
  }
  return true;
}

struct TimeAnchors {
  uint64_t master_anchor_ns = now_master_ns();
  uint64_t realtime_anchor_ns = now_realtime_ns();
};

struct LinFitWindow {
  struct XY { double x; double y; };
  std::deque<XY> q;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_xx = 0.0;
  double sum_xy = 0.0;
  double window_sec = 10.0;

  void push(double x, double y) {
    q.push_back({x, y});
    sum_x += x;
    sum_y += y;
    sum_xx += x * x;
    sum_xy += x * y;
    while (!q.empty() && (x - q.front().x) > window_sec) {
      XY old = q.front();
      q.pop_front();
      sum_x -= old.x;
      sum_y -= old.y;
      sum_xx -= old.x * old.x;
      sum_xy -= old.x * old.y;
    }
  }

  bool estimate(double& a, double& b) const {
    if (q.size() < 30) return false;
    double n = static_cast<double>(q.size());
    double denom = n * sum_xx - sum_x * sum_x;
    if (std::abs(denom) < 1e-12) return false;
    a = (n * sum_xy - sum_x * sum_y) / denom;
    b = (sum_y - a * sum_x) / n;
    return true;
  }
};

class UtcMapper {
 public:
  explicit UtcMapper(TimeAnchors anchors) : anchors_(anchors) {}

  void update(uint64_t master_ns, uint64_t utc_ns) {
    if (!plausible_utc_ns(utc_ns)) {
      static std::atomic<uint64_t> rejected_count{0};
      const uint64_t count = rejected_count.fetch_add(1);
      if (count < 5 || (count % 1000) == 0) {
        log_warn("rejected implausible UTC mapper update utc_ns=" + std::to_string(utc_ns));
      }
      return;
    }
    std::lock_guard<std::mutex> lk(m_);
    double x = static_cast<double>(master_ns) * 1e-9;
    double y = static_cast<double>(utc_ns) * 1e-9;
    if (!have_origin_) {
      x0_ = x;
      y0_ = y;
      have_origin_ = true;
    }
    fit_.push(x - x0_, y - y0_);
    last_master_ns_ = master_ns;
    last_utc_ns_ = utc_ns;
    double aa = 0.0, bb = 0.0;
    if (fit_.estimate(aa, bb)) {
      a_ = aa;
      b_ = bb;
      have_fit_ = true;
    }
  }

  bool map(uint64_t master_ns, uint64_t& utc_ns) const {
    std::lock_guard<std::mutex> lk(m_);
    if (!have_origin_) return false;
    if (have_fit_) {
      double x_rel = static_cast<double>(master_ns) * 1e-9 - x0_;
      double y_rel = a_ * x_rel + b_;
      const uint64_t candidate = static_cast<uint64_t>(llround((y0_ + y_rel) * 1e9));
      if (plausible_utc_ns(candidate)) {
        utc_ns = candidate;
        return true;
      }
      return false;
    }
    if (last_master_ns_ != 0 && last_utc_ns_ != 0) {
      int64_t offset = static_cast<int64_t>(last_utc_ns_) - static_cast<int64_t>(last_master_ns_);
      const uint64_t candidate = static_cast<uint64_t>(static_cast<int64_t>(master_ns) + offset);
      if (plausible_utc_ns(candidate)) {
        utc_ns = candidate;
        return true;
      }
      return false;
    }
    return false;
  }

  uint64_t fallback_from_anchor(uint64_t master_ns) const {
    if (master_ns >= anchors_.master_anchor_ns) {
      return anchors_.realtime_anchor_ns + (master_ns - anchors_.master_anchor_ns);
    }
    return anchors_.realtime_anchor_ns - (anchors_.master_anchor_ns - master_ns);
  }

 private:
  TimeAnchors anchors_;
  mutable std::mutex m_;
  LinFitWindow fit_;
  bool have_origin_{false};
  bool have_fit_{false};
  double x0_{0.0};
  double y0_{0.0};
  double a_{1.0};
  double b_{0.0};
  uint64_t last_master_ns_{0};
  uint64_t last_utc_ns_{0};
};

static bool set_serial(int fd, int baud) {
  termios tty{};
  if (tcgetattr(fd, &tty) != 0) return false;
  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~HUPCL;

  speed_t sp = B115200;
  switch (baud) {
    case 9600: sp = B9600; break;
    case 19200: sp = B19200; break;
    case 38400: sp = B38400; break;
    case 57600: sp = B57600; break;
    case 115200: sp = B115200; break;
    case 230400: sp = B230400; break;
    default: sp = B115200; break;
  }
  cfsetispeed(&tty, sp);
  cfsetospeed(&tty, sp);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;
  return tcsetattr(fd, TCSANOW, &tty) == 0;
}

static inline uint16_t be_u16(const uint8_t* p) { return (uint16_t(p[0]) << 8) | uint16_t(p[1]); }
static inline uint32_t be_u32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}
static inline uint64_t be_u64(const uint8_t* p) {
  return (uint64_t(p[0]) << 56) | (uint64_t(p[1]) << 48) | (uint64_t(p[2]) << 40) | (uint64_t(p[3]) << 32) |
         (uint64_t(p[4]) << 24) | (uint64_t(p[5]) << 16) | (uint64_t(p[6]) << 8) | uint64_t(p[7]);
}
static inline int32_t be_i32(const uint8_t* p) { return static_cast<int32_t>(be_u32(p)); }
static inline int64_t xsens_fp1632_i64(const uint8_t* p) {
  uint64_t u = (uint64_t(p[4]) << 40) | (uint64_t(p[5]) << 32) |
               (uint64_t(p[0]) << 24) | (uint64_t(p[1]) << 16) |
               (uint64_t(p[2]) << 8) | uint64_t(p[3]);
  if (u & (1ULL << 47)) u |= 0xFFFF000000000000ULL;
  return static_cast<int64_t>(u);
}
static inline float be_f32(const uint8_t* p) {
  uint32_t u = be_u32(p);
  float f;
  std::memcpy(&f, &u, sizeof(float));
  return f;
}
static inline double be_f64(const uint8_t* p) {
  uint64_t u = be_u64(p);
  double d;
  std::memcpy(&d, &u, sizeof(double));
  return d;
}

static bool checksum_ok_deque(const std::deque<uint8_t>& buf, size_t frame_len) {
  uint32_t sum = 0;
  for (size_t i = 1; i < frame_len; ++i) sum += buf[i];
  return (sum & 0xFF) == 0;
}

static bool xsens_utc_to_ns(uint16_t year, uint8_t mon, uint8_t day,
                            uint8_t hour, uint8_t min, uint8_t sec,
                            uint32_t nanos, uint64_t& out_ns) {
  if (year < 1970 || mon < 1 || mon > 12 || day < 1 || day > 31 ||
      hour > 23 || min > 59 || sec > 60 || nanos >= 1000000000U) {
    return false;
  }
  struct tm tmv{};
  tmv.tm_year = static_cast<int>(year) - 1900;
  tmv.tm_mon = static_cast<int>(mon) - 1;
  tmv.tm_mday = static_cast<int>(day);
  tmv.tm_hour = static_cast<int>(hour);
  tmv.tm_min = static_cast<int>(min);
  tmv.tm_sec = (sec == 60) ? 59 : static_cast<int>(sec);
  time_t sec_epoch = timegm(&tmv);
  if (sec_epoch < 0) return false;
  out_ns = uint64_t(sec_epoch) * 1000000000ULL + uint64_t(nanos);
  if (sec == 60) out_ns += 1000000000ULL;
  return true;
}

static uint32_t novatel_crc32_value(uint32_t value) {
  constexpr uint32_t kPolynomial = 0xEDB88320U;
  uint32_t crc = value;
  for (int i = 0; i < 8; ++i) {
    crc = (crc & 1U) ? ((crc >> 1) ^ kPolynomial) : (crc >> 1);
  }
  return crc;
}

static uint32_t novatel_crc32_block(const uint8_t* data, size_t size) {
  uint32_t crc = 0;
  for (size_t i = 0; i < size; ++i) {
    uint32_t temp1 = (crc >> 8) & 0x00FFFFFFU;
    uint32_t temp2 = novatel_crc32_value((crc ^ data[i]) & 0xFFU);
    crc = temp1 ^ temp2;
  }
  return crc;
}

static bool ascii_xor_checksum_ok(const std::string& line) {
  if (line.size() < 4) return false;
  if (line[0] != '$' && line[0] != '#' && line[0] != '%') return false;
  size_t star = line.find('*');
  if (star == std::string::npos || star + 2 >= line.size()) return false;

  char* end = nullptr;
  if (line[0] == '$') {
    if (star + 2 >= line.size()) return false;
    unsigned chk = 0;
    for (size_t i = 1; i < star; ++i) chk ^= static_cast<unsigned char>(line[i]);
    unsigned want = std::strtoul(line.c_str() + star + 1, &end, 16);
    return end == line.c_str() + star + 3 && chk == want;
  }

  if (star + 8 >= line.size()) return false;
  uint32_t want = std::strtoul(line.c_str() + star + 1, &end, 16);
  if (end != line.c_str() + star + 9) return false;
  uint32_t got = novatel_crc32_block(reinterpret_cast<const uint8_t*>(line.data() + 1), star - 1);
  return got == want;
}

static std::vector<std::string> split_csv(const std::string& s) {
  std::vector<std::string> out;
  size_t start = 0;
  while (start <= s.size()) {
    size_t pos = s.find(',', start);
    if (pos == std::string::npos) {
      out.push_back(s.substr(start));
      break;
    }
    out.push_back(s.substr(start, pos - start));
    start = pos + 1;
  }
  return out;
}

static bool parse_nmea_hms(const std::string& s, uint8_t& hour, uint8_t& min, uint8_t& sec, uint32_t& nanos) {
  if (s.size() < 6) return false;
  hour = static_cast<uint8_t>(std::atoi(s.substr(0, 2).c_str()));
  min = static_cast<uint8_t>(std::atoi(s.substr(2, 2).c_str()));
  double sec_f = std::strtod(s.c_str() + 4, nullptr);
  if (hour > 23 || min > 59 || sec_f < 0.0 || sec_f >= 61.0) return false;
  sec = static_cast<uint8_t>(std::floor(sec_f));
  nanos = static_cast<uint32_t>(llround((sec_f - static_cast<double>(sec)) * 1e9));
  if (nanos >= 1000000000U) {
    nanos -= 1000000000U;
    sec = static_cast<uint8_t>(sec + 1);
  }
  return true;
}

static bool parse_nmea_date_ddmmyy(const std::string& s, uint16_t& year, uint8_t& mon, uint8_t& day) {
  if (s.size() != 6) return false;
  day = static_cast<uint8_t>(std::atoi(s.substr(0, 2).c_str()));
  mon = static_cast<uint8_t>(std::atoi(s.substr(2, 2).c_str()));
  int yy = std::atoi(s.substr(4, 2).c_str());
  year = static_cast<uint16_t>((yy >= 80) ? (1900 + yy) : (2000 + yy));
  return true;
}

static bool parse_nmea_latlon(const std::string& value, const std::string& hemi, bool is_lat, double& deg_out) {
  int deg_digits = is_lat ? 2 : 3;
  if (value.size() < static_cast<size_t>(deg_digits + 3) || hemi.empty()) return false;
  double raw = std::strtod(value.c_str(), nullptr);
  double scale = 100.0;
  int deg = static_cast<int>(raw / scale);
  double minutes = raw - static_cast<double>(deg) * scale;
  deg_out = static_cast<double>(deg) + minutes / 60.0;
  char h = hemi[0];
  if (h == 'S' || h == 'W') deg_out = -deg_out;
  return h == 'N' || h == 'S' || h == 'E' || h == 'W';
}

struct CameraConfig {
  std::string name;
  std::string device;
  int planner_camera_id = -1;
  std::string backend = "v4l2";
  int source_width = 3840;
  int source_height = 2160;
  int source_fps = 30;
  int planner_ingest_fps = 15;
  int planner_width = 576;
  int planner_height = 320;
  std::string pixel_format = "UYVY";
};

struct SensorContainerConfig {
  fs::path config_path;
  std::string bind_host = "0.0.0.0";
  int bind_port = 18080;
  std::string clip_id = "bringup_run";
  std::string vehicle_id = "test_car_01";
  std::string imu_dev = "/dev/ttyUSB0";
  int imu_baud = 115200;
  double gnss_antenna_to_ego_x_m = 0.0;
  double gnss_antenna_to_ego_y_m = 0.0;
  double gnss_antenna_to_ego_z_m = 0.0;
  bool allow_poseless_test_mode = false;
  uint64_t camera_target_mismatch_us = 50000;
  uint64_t localization_stale_us = 100000;
  uint64_t camera_state_skew_us = 50000;
  double fixed_delta_seconds = 0.1;
  std::vector<CameraConfig> cameras;
};

static SensorContainerConfig default_config() {
  SensorContainerConfig cfg;
  cfg.cameras = {
      {"left", "/dev/video2", 0, "v4l2", 1920, 1080, 30, 15, 576, 320, "UYVY"},
      {"front", "/dev/video5", 1, "v4l2", 1920, 1080, 30, 15, 576, 320, "UYVY"},
      {"right", "/dev/video0", 2, "v4l2", 1920, 1080, 30, 15, 576, 320, "UYVY"},
      {"front_tele", "/dev/video4", 6, "v4l2", 1920, 1080, 30, 15, 576, 320, "UYVY"},
  };
  return cfg;
}

static bool load_yaml_config(const fs::path& path, SensorContainerConfig& cfg, std::string& error) {
  SimpleYamlConfig yaml;
  if (!parse_simple_yaml_file(path, yaml, error)) return false;
  auto get_scalar = [&](const std::string& key) -> std::optional<std::string> {
    auto it = yaml.scalars.find(key);
    if (it == yaml.scalars.end()) return std::nullopt;
    return it->second;
  };
  auto assign_int = [&](const std::string& key, int& target, int min_value) -> bool {
    auto value = get_scalar(key);
    if (!value) return true;
    int parsed = 0;
    if (!parse_yaml_int(*value, parsed) || parsed < min_value) {
      error = "invalid integer for '" + key + "'";
      return false;
    }
    target = parsed;
    return true;
  };
  auto assign_u64 = [&](const std::string& key, uint64_t& target, uint64_t min_value) -> bool {
    auto value = get_scalar(key);
    if (!value) return true;
    uint64_t parsed = 0;
    if (!parse_yaml_u64(*value, parsed) || parsed < min_value) {
      error = "invalid integer for '" + key + "'";
      return false;
    }
    target = parsed;
    return true;
  };
  auto assign_bool = [&](const std::string& key, bool& target) -> bool {
    auto value = get_scalar(key);
    if (!value) return true;
    bool parsed = false;
    if (!parse_yaml_bool(*value, parsed)) {
      error = "invalid boolean for '" + key + "'";
      return false;
    }
    target = parsed;
    return true;
  };
  auto assign_double = [&](const std::string& key, double& target, double min_value) -> bool {
    auto value = get_scalar(key);
    if (!value) return true;
    double parsed = 0.0;
    if (!parse_yaml_double(*value, parsed) || parsed < min_value) {
      error = "invalid double for '" + key + "'";
      return false;
    }
    target = parsed;
    return true;
  };

  if (auto v = get_scalar("bind_host")) cfg.bind_host = *v;
  if (auto v = get_scalar("clip_id")) cfg.clip_id = *v;
  if (auto v = get_scalar("vehicle_id")) cfg.vehicle_id = *v;
  if (auto v = get_scalar("imu_dev")) cfg.imu_dev = *v;
  if (auto v = get_scalar("camera_backend")) {
    for (auto& cam : cfg.cameras) cam.backend = to_lower_copy(*v);
  }
  if (auto v = get_scalar("pixel_format")) {
    for (auto& cam : cfg.cameras) cam.pixel_format = *v;
  }

  if (!assign_int("bind_port", cfg.bind_port, 1)) return false;
  if (!assign_int("imu_baud", cfg.imu_baud, 1)) return false;
  if (!assign_double("gnss_antenna_to_ego_x_m", cfg.gnss_antenna_to_ego_x_m, -1000.0)) return false;
  if (!assign_double("gnss_antenna_to_ego_y_m", cfg.gnss_antenna_to_ego_y_m, -1000.0)) return false;
  if (!assign_double("gnss_antenna_to_ego_z_m", cfg.gnss_antenna_to_ego_z_m, -1000.0)) return false;
  if (!assign_u64("camera_target_mismatch_us", cfg.camera_target_mismatch_us, 1)) return false;
  if (!assign_u64("localization_stale_us", cfg.localization_stale_us, 1)) return false;
  if (!assign_u64("camera_state_skew_us", cfg.camera_state_skew_us, 1)) return false;
  if (!assign_double("fixed_delta_seconds", cfg.fixed_delta_seconds, 0.001)) return false;
  if (!assign_bool("allow_poseless_test_mode", cfg.allow_poseless_test_mode)) return false;

  int planner_width = cfg.cameras.empty() ? 576 : cfg.cameras.front().planner_width;
  int planner_height = cfg.cameras.empty() ? 320 : cfg.cameras.front().planner_height;
  int source_width = cfg.cameras.empty() ? 3840 : cfg.cameras.front().source_width;
  int source_height = cfg.cameras.empty() ? 2160 : cfg.cameras.front().source_height;
  int source_fps = cfg.cameras.empty() ? 30 : cfg.cameras.front().source_fps;
  int planner_ingest_fps = cfg.cameras.empty() ? 15 : cfg.cameras.front().planner_ingest_fps;
  if (!assign_int("planner_width", planner_width, 1)) return false;
  if (!assign_int("planner_height", planner_height, 1)) return false;
  if (!assign_int("camera_width", source_width, 1)) return false;
  if (!assign_int("camera_height", source_height, 1)) return false;
  if (!assign_int("camera_fps", source_fps, 1)) return false;
  if (!assign_int("planner_ingest_fps", planner_ingest_fps, 1)) return false;
  for (auto& cam : cfg.cameras) {
    cam.planner_width = planner_width;
    cam.planner_height = planner_height;
    cam.source_width = source_width;
    cam.source_height = source_height;
    cam.source_fps = source_fps;
    cam.planner_ingest_fps = planner_ingest_fps;
  }

  std::vector<std::string> camera_devices;
  auto camera_devices_list = yaml.lists.find("camera_devices");
  if (camera_devices_list != yaml.lists.end()) {
    camera_devices = camera_devices_list->second;
  } else if (auto value = get_scalar("camera_devices")) {
    camera_devices = split_csv_items(*value);
  }

  std::vector<std::string> camera_names;
  auto camera_names_list = yaml.lists.find("camera_names");
  if (camera_names_list != yaml.lists.end()) {
    camera_names = camera_names_list->second;
  } else if (auto value = get_scalar("camera_names")) {
    camera_names = split_csv_items(*value);
  }

  if (!camera_devices.empty()) {
    const std::vector<CameraConfig> default_cameras = cfg.cameras;
    if (camera_devices.size() > default_cameras.size()) {
      error = "camera_devices must have at most " + std::to_string(default_cameras.size()) + " entries";
      return false;
    }
    if (!camera_names.empty() && camera_names.size() != camera_devices.size()) {
      error = "camera_names must have the same number of entries as camera_devices";
      return false;
    }

    std::vector<CameraConfig> selected_cameras;
    selected_cameras.reserve(camera_devices.size());
    for (size_t i = 0; i < camera_devices.size(); ++i) {
      CameraConfig cam;
      if (camera_names.empty()) {
        cam = default_cameras[i];
      } else {
        const std::string name = normalize_camera_name(camera_names[i]);
        auto it = std::find_if(default_cameras.begin(), default_cameras.end(),
                               [&](const CameraConfig& candidate) { return candidate.name == name; });
        if (it == default_cameras.end()) {
          error = "unknown camera name '" + name + "' in camera_names";
          return false;
        }
        cam = *it;
      }
      cam.device = normalize_camera_device(camera_devices[i]);
      selected_cameras.push_back(std::move(cam));
    }
    cfg.cameras = std::move(selected_cameras);
  } else if (!camera_names.empty()) {
    const std::vector<CameraConfig> default_cameras = cfg.cameras;
    std::vector<CameraConfig> selected_cameras;
    selected_cameras.reserve(camera_names.size());
    for (const auto& raw_name : camera_names) {
      const std::string name = normalize_camera_name(raw_name);
      auto it = std::find_if(default_cameras.begin(), default_cameras.end(),
                             [&](const CameraConfig& candidate) { return candidate.name == name; });
      if (it == default_cameras.end()) {
        error = "unknown camera name '" + name + "' in camera_names";
        return false;
      }
      selected_cameras.push_back(*it);
    }
    cfg.cameras = std::move(selected_cameras);
  }

  if (cfg.cameras.empty()) {
    error = "at least one camera must be configured";
    return false;
  }
  return true;
}

enum class ImagePixelFormat {
  kUnknown = 0,
  kRgb24,
  kRgba32,
  kUyvy422,
};

struct ImageFrame {
  uint64_t seq = 0;
  uint64_t master_ns = 0;
  uint64_t utc_us = 0;
  int width = 0;
  int height = 0;
  int stride = 0;
  ImagePixelFormat format = ImagePixelFormat::kUnknown;
  std::shared_ptr<std::vector<uint8_t>> bytes;
  std::shared_ptr<void> hold_ref;
  const uint8_t* data = nullptr;
  size_t data_size = 0;
};

class FrameRing {
 public:
  explicit FrameRing(size_t max_frames = 180) : max_frames_(max_frames) {}

  void push(ImageFrame frame) {
    std::lock_guard<std::mutex> lk(m_);
    frames_.push_back(std::move(frame));
    while (frames_.size() > max_frames_) frames_.pop_front();
  }

  bool nearest(uint64_t target_utc_us, ImageFrame& out, uint64_t& abs_err_us) const {
    std::lock_guard<std::mutex> lk(m_);
    if (frames_.empty()) return false;
    size_t best_idx = 0;
    uint64_t best_err = std::numeric_limits<uint64_t>::max();
    for (size_t i = 0; i < frames_.size(); ++i) {
      uint64_t err = (frames_[i].utc_us > target_utc_us) ? (frames_[i].utc_us - target_utc_us) : (target_utc_us - frames_[i].utc_us);
      if (err < best_err) {
        best_err = err;
        best_idx = i;
      }
    }
    out = frames_[best_idx];
    abs_err_us = best_err;
    return true;
  }

  uint64_t latest_utc_us() const {
    std::lock_guard<std::mutex> lk(m_);
    return frames_.empty() ? 0 : frames_.back().utc_us;
  }

  uint64_t latest_seq() const {
    std::lock_guard<std::mutex> lk(m_);
    return frames_.empty() ? 0 : frames_.back().seq;
  }

  size_t size() const {
    std::lock_guard<std::mutex> lk(m_);
    return frames_.size();
  }

  void clear() {
    std::lock_guard<std::mutex> lk(m_);
    frames_.clear();
  }

 private:
  size_t max_frames_;
  mutable std::mutex m_;
  std::deque<ImageFrame> frames_;
};

struct OrientationSample {
  uint64_t utc_us = 0;
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double yaw_deg = 0.0;
  uint16_t pc = 0;
};

struct PoseSample {
  uint64_t utc_us = 0;
  std::array<double, 3> enu_xyz{0.0, 0.0, 0.0};
  std::array<double, 9> world_from_ego{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0,
  };
  std::array<double, 3> lla{0.0, 0.0, 0.0};
  uint16_t pc = 0;
};

static double wrap_angle_deg(double angle_deg) {
  double out = std::fmod(angle_deg + 180.0, 360.0);
  if (out < 0.0) out += 360.0;
  return out - 180.0;
}

class OrientationRing {
 public:
  explicit OrientationRing(size_t max_samples = 4096) : max_samples_(max_samples) {}

  void push(const OrientationSample& sample) {
    std::lock_guard<std::mutex> lk(m_);
    samples_.push_back(sample);
    while (samples_.size() > max_samples_) samples_.pop_front();
  }

  bool nearest(uint64_t target_utc_us, OrientationSample& out, uint64_t& abs_err_us) const {
    std::lock_guard<std::mutex> lk(m_);
    if (samples_.empty()) return false;
    size_t best_idx = 0;
    uint64_t best_err = std::numeric_limits<uint64_t>::max();
    for (size_t i = 0; i < samples_.size(); ++i) {
      uint64_t err = (samples_[i].utc_us > target_utc_us) ? (samples_[i].utc_us - target_utc_us) : (target_utc_us - samples_[i].utc_us);
      if (err < best_err) {
        best_err = err;
        best_idx = i;
      }
    }
    out = samples_[best_idx];
    abs_err_us = best_err;
    return true;
  }

  bool interpolate(uint64_t target_utc_us, OrientationSample& out, uint64_t& support_err_us) const {
    std::lock_guard<std::mutex> lk(m_);
    if (samples_.empty()) return false;
    if (samples_.size() == 1) {
      out = samples_.front();
      support_err_us = (out.utc_us > target_utc_us) ? (out.utc_us - target_utc_us) : (target_utc_us - out.utc_us);
      return true;
    }
    if (target_utc_us <= samples_.front().utc_us) {
      out = samples_.front();
      support_err_us = samples_.front().utc_us - target_utc_us;
      return true;
    }
    if (target_utc_us >= samples_.back().utc_us) {
      out = samples_.back();
      support_err_us = target_utc_us - samples_.back().utc_us;
      return true;
    }

    size_t upper_idx = 1;
    while (upper_idx < samples_.size() && samples_[upper_idx].utc_us < target_utc_us) ++upper_idx;
    if (upper_idx >= samples_.size()) {
      out = samples_.back();
      support_err_us = target_utc_us - samples_.back().utc_us;
      return true;
    }

    const auto& a = samples_[upper_idx - 1];
    const auto& b = samples_[upper_idx];
    const uint64_t span_us = std::max<uint64_t>(1, b.utc_us - a.utc_us);
    const double alpha = static_cast<double>(target_utc_us - a.utc_us) / static_cast<double>(span_us);

    out.utc_us = target_utc_us;
    out.pc = (alpha < 0.5) ? a.pc : b.pc;
    out.roll_deg = a.roll_deg + (b.roll_deg - a.roll_deg) * alpha;
    out.pitch_deg = a.pitch_deg + (b.pitch_deg - a.pitch_deg) * alpha;
    const double yaw_delta_deg = wrap_angle_deg(b.yaw_deg - a.yaw_deg);
    out.yaw_deg = wrap_angle_deg(a.yaw_deg + yaw_delta_deg * alpha);
    support_err_us = std::max<uint64_t>(target_utc_us - a.utc_us, b.utc_us - target_utc_us);
    return true;
  }

  uint64_t latest_utc_us() const {
    std::lock_guard<std::mutex> lk(m_);
    return samples_.empty() ? 0 : samples_.back().utc_us;
  }

  size_t size() const {
    std::lock_guard<std::mutex> lk(m_);
    return samples_.size();
  }

 private:
  size_t max_samples_;
  mutable std::mutex m_;
  std::deque<OrientationSample> samples_;
};

class PoseRing {
 public:
  explicit PoseRing(size_t max_samples = 512) : max_samples_(max_samples) {}

  void push(const PoseSample& sample) {
    std::lock_guard<std::mutex> lk(m_);
    samples_.push_back(sample);
    while (samples_.size() > max_samples_) samples_.pop_front();
  }

  bool nearest(uint64_t target_utc_us, PoseSample& out, uint64_t& abs_err_us) const {
    std::lock_guard<std::mutex> lk(m_);
    if (samples_.empty()) return false;
    size_t best_idx = 0;
    uint64_t best_err = std::numeric_limits<uint64_t>::max();
    for (size_t i = 0; i < samples_.size(); ++i) {
      uint64_t err = (samples_[i].utc_us > target_utc_us) ? (samples_[i].utc_us - target_utc_us) : (target_utc_us - samples_[i].utc_us);
      if (err < best_err) {
        best_err = err;
        best_idx = i;
      }
    }
    out = samples_[best_idx];
    abs_err_us = best_err;
    return true;
  }

  bool interpolate_xyz(uint64_t target_utc_us, PoseSample& out, uint64_t& support_err_us) const {
    std::lock_guard<std::mutex> lk(m_);
    if (samples_.empty()) return false;
    if (samples_.size() == 1) {
      out = samples_.front();
      support_err_us = (out.utc_us > target_utc_us) ? (out.utc_us - target_utc_us) : (target_utc_us - out.utc_us);
      return true;
    }
    if (target_utc_us <= samples_.front().utc_us) {
      out = samples_.front();
      support_err_us = samples_.front().utc_us - target_utc_us;
      return true;
    }
    if (target_utc_us >= samples_.back().utc_us) {
      out = samples_.back();
      support_err_us = target_utc_us - samples_.back().utc_us;
      return true;
    }

    size_t upper_idx = 1;
    while (upper_idx < samples_.size() && samples_[upper_idx].utc_us < target_utc_us) ++upper_idx;
    if (upper_idx >= samples_.size()) {
      out = samples_.back();
      support_err_us = target_utc_us - samples_.back().utc_us;
      return true;
    }

    const auto& a = samples_[upper_idx - 1];
    const auto& b = samples_[upper_idx];
    const uint64_t span_us = std::max<uint64_t>(1, b.utc_us - a.utc_us);
    const double alpha = static_cast<double>(target_utc_us - a.utc_us) / static_cast<double>(span_us);

    out = a;
    out.utc_us = target_utc_us;
    out.pc = (alpha < 0.5) ? a.pc : b.pc;
    for (int i = 0; i < 3; ++i) {
      out.enu_xyz[i] = a.enu_xyz[i] + (b.enu_xyz[i] - a.enu_xyz[i]) * alpha;
      out.lla[i] = a.lla[i] + (b.lla[i] - a.lla[i]) * alpha;
    }
    support_err_us = std::max<uint64_t>(target_utc_us - a.utc_us, b.utc_us - target_utc_us);
    return true;
  }

  uint64_t latest_utc_us() const {
    std::lock_guard<std::mutex> lk(m_);
    return samples_.empty() ? 0 : samples_.back().utc_us;
  }

  size_t size() const {
    std::lock_guard<std::mutex> lk(m_);
    return samples_.size();
  }

  std::optional<PoseSample> latest() const {
    std::lock_guard<std::mutex> lk(m_);
    if (samples_.empty()) return std::nullopt;
    return samples_.back();
  }

 private:
  size_t max_samples_;
  mutable std::mutex m_;
  std::deque<PoseSample> samples_;
};

static std::array<double, 3> lla_to_ecef(double lat_deg, double lon_deg, double alt_m) {
  constexpr double a = 6378137.0;
  constexpr double e2 = 6.69437999014e-3;
  double lat = lat_deg * M_PI / 180.0;
  double lon = lon_deg * M_PI / 180.0;
  double sin_lat = std::sin(lat);
  double cos_lat = std::cos(lat);
  double sin_lon = std::sin(lon);
  double cos_lon = std::cos(lon);
  double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
  return {
      (N + alt_m) * cos_lat * cos_lon,
      (N + alt_m) * cos_lat * sin_lon,
      (N * (1.0 - e2) + alt_m) * sin_lat,
  };
}

static std::array<double, 3> ecef_to_enu(const std::array<double, 3>& ecef_xyz,
                                         const std::array<double, 3>& ref_lla,
                                         const std::array<double, 3>& ref_ecef) {
  double ref_lat = ref_lla[0] * M_PI / 180.0;
  double ref_lon = ref_lla[1] * M_PI / 180.0;
  double sin_lat = std::sin(ref_lat);
  double cos_lat = std::cos(ref_lat);
  double sin_lon = std::sin(ref_lon);
  double cos_lon = std::cos(ref_lon);
  double dx = ecef_xyz[0] - ref_ecef[0];
  double dy = ecef_xyz[1] - ref_ecef[1];
  double dz = ecef_xyz[2] - ref_ecef[2];
  return {
      -sin_lon * dx + cos_lon * dy,
      -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz,
      cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz,
  };
}

static std::array<double, 9> matmul33(const std::array<double, 9>& A, const std::array<double, 9>& B) {
  std::array<double, 9> out{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      double v = 0.0;
      for (int k = 0; k < 3; ++k) v += A[r * 3 + k] * B[k * 3 + c];
      out[r * 3 + c] = v;
    }
  }
  return out;
}

static std::array<double, 9> transpose33(const std::array<double, 9>& M) {
  return {
      M[0], M[3], M[6],
      M[1], M[4], M[7],
      M[2], M[5], M[8],
  };
}

static std::array<double, 3> matvec33(const std::array<double, 9>& M, const std::array<double, 3>& v) {
  return {
      M[0] * v[0] + M[1] * v[1] + M[2] * v[2],
      M[3] * v[0] + M[4] * v[1] + M[5] * v[2],
      M[6] * v[0] + M[7] * v[1] + M[8] * v[2],
  };
}

static std::array<double, 9> rotation_matrix_from_ypr_rad(double yaw, double pitch, double roll) {
  double cy = std::cos(yaw), sy = std::sin(yaw);
  double cp = std::cos(pitch), sp = std::sin(pitch);
  double cr = std::cos(roll), sr = std::sin(roll);
  std::array<double, 9> rz{cy, -sy, 0.0, sy, cy, 0.0, 0.0, 0.0, 1.0};
  std::array<double, 9> ry{cp, 0.0, sp, 0.0, 1.0, 0.0, -sp, 0.0, cp};
  std::array<double, 9> rx{1.0, 0.0, 0.0, 0.0, cr, -sr, 0.0, sr, cr};
  return matmul33(matmul33(rz, ry), rx);
}

static bool gps_week_tow_to_unix_ns(uint16_t gps_week, double tow_sec, uint64_t& out_ns) {
  if (!plausible_gps_week(gps_week) || tow_sec < 0.0 || !std::isfinite(tow_sec) ||
      tow_sec > 7.0 * 86400.0 + 1.0) {
    return false;
  }
  constexpr uint64_t gps_epoch_unix_ns = 315964800ULL * 1000000000ULL;  // 1980-01-06
  constexpr int64_t gps_minus_utc_sec = 18;  // valid for current deployed systems
  int64_t whole_sec = static_cast<int64_t>(std::floor(tow_sec));
  double frac_sec = tow_sec - static_cast<double>(whole_sec);
  if (frac_sec < 0.0) return false;
  uint64_t frac_ns = static_cast<uint64_t>(llround(frac_sec * 1e9));
  if (frac_ns >= 1000000000ULL) {
    frac_ns -= 1000000000ULL;
    ++whole_sec;
  }
  uint64_t gps_ns = gps_epoch_unix_ns +
                    static_cast<uint64_t>(gps_week) * 7ULL * 24ULL * 3600ULL * 1000000000ULL +
                    static_cast<uint64_t>(whole_sec) * 1000000000ULL + frac_ns;
  if (gps_ns < static_cast<uint64_t>(gps_minus_utc_sec) * 1000000000ULL) return false;
  out_ns = gps_ns - static_cast<uint64_t>(gps_minus_utc_sec) * 1000000000ULL;
  return true;
}
