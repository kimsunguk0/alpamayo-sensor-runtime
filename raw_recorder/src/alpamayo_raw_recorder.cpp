#include <gst/gst.h>
#include <gst/video/video.h>

#include <linux/videodev2.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <limits>
#include <optional>
#include <poll.h>
#include <set>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <vector>

namespace fs = std::filesystem;

static std::atomic<bool> g_stop{false};
static void on_signal(int) { g_stop.store(true); }

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

static std::string ns_to_utc_string(uint64_t rt_ns) {
  time_t sec = static_cast<time_t>(rt_ns / 1000000000ULL);
  struct tm tmv{};
  gmtime_r(&sec, &tmv);
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d",
                tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
  return std::string(buf);
}

static std::string json_escape(const std::string& s) {
  std::ostringstream os;
  for (char ch : s) {
    switch (ch) {
      case '\\': os << "\\\\"; break;
      case '"': os << "\\\""; break;
      case '\n': os << "\\n"; break;
      case '\r': os << "\\r"; break;
      case '\t': os << "\\t"; break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20) {
          os << "\\u"
             << std::hex << std::setw(4) << std::setfill('0')
             << static_cast<int>(static_cast<unsigned char>(ch))
             << std::dec;
        } else {
          os << ch;
        }
    }
  }
  return os.str();
}

static std::string shell_quote(const std::string& s) {
  std::string out = "'";
  for (char ch : s) {
    if (ch == '\'') {
      out += "'\\''";
    } else {
      out.push_back(ch);
    }
  }
  out.push_back('\'');
  return out;
}

static void log_line(const char* level, const std::string& message) {
  std::cout << "[" << level << "] " << message << "\n";
}

static void log_info(const std::string& message) { log_line("info", message); }
static void log_warn(const std::string& message) { log_line("warn", message); }
static void log_ready(const std::string& message) { log_line("ready", message); }
static void log_error(const std::string& message) { log_line("error", message); }

class CsvSpoolWriter {
 public:
  CsvSpoolWriter() = default;
  ~CsvSpoolWriter() { close(); }

  bool open(const fs::path& path, const std::string& header) {
    close();
    path_ = path;
    fs::create_directories(path.parent_path());
    ofs_.open(path, std::ios::out | std::ios::trunc);
    if (!ofs_) return false;
    ofs_ << header << '\n';
    ofs_.flush();
    stop_ = false;
    worker_ = std::thread([this]() { worker_main(); });
    return true;
  }

  void push(std::string line) {
    std::lock_guard<std::mutex> lk(m_);
    queue_.push_back(std::move(line));
    cv_.notify_one();
  }

  void close() {
    {
      std::lock_guard<std::mutex> lk(m_);
      if (!ofs_.is_open() && !worker_.joinable()) return;
      stop_ = true;
      cv_.notify_all();
    }
    if (worker_.joinable()) worker_.join();
    if (ofs_.is_open()) {
      ofs_.flush();
      ofs_.close();
    }
    queue_.clear();
  }

  const fs::path& path() const { return path_; }

 private:
  void worker_main() {
    std::deque<std::string> local;
    while (true) {
      {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [&]() { return stop_ || !queue_.empty(); });
        if (queue_.empty() && stop_) break;
        local.swap(queue_);
      }
      while (!local.empty()) {
        ofs_ << local.front() << '\n';
        local.pop_front();
      }
      ofs_.flush();
    }
  }

  fs::path path_;
  std::ofstream ofs_;
  std::mutex m_;
  std::condition_variable cv_;
  std::deque<std::string> queue_;
  bool stop_{false};
  std::thread worker_;
};

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

  void clear() {
    q.clear();
    sum_x = sum_y = sum_xx = sum_xy = 0.0;
  }

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

  size_t size() const { return q.size(); }
};

class UtcMapper {
 public:
  explicit UtcMapper(TimeAnchors anchors) : anchors_(anchors) {}

  void update(uint64_t master_ns, uint64_t utc_ns) {
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
    double aa = 0.0;
    double bb = 0.0;
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
      utc_ns = static_cast<uint64_t>(llround((y0_ + y_rel) * 1e9));
      return true;
    }
    if (last_master_ns_ != 0 && last_utc_ns_ != 0) {
      int64_t offset = static_cast<int64_t>(last_utc_ns_) - static_cast<int64_t>(last_master_ns_);
      utc_ns = static_cast<uint64_t>(static_cast<int64_t>(master_ns) + offset);
      return true;
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

static int xioctl(int fd, unsigned long req, void* arg) {
  int r;
  do {
    r = ioctl(fd, req, arg);
  } while (r == -1 && errno == EINTR);
  return r;
}

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

static bool nmea_checksum_ok(const std::string& line) {
  if (line.size() < 4 || line[0] != '$') return false;
  size_t star = line.find('*');
  if (star == std::string::npos || star + 2 >= line.size()) return false;
  unsigned chk = 0;
  for (size_t i = 1; i < star; ++i) chk ^= static_cast<unsigned char>(line[i]);
  char* end = nullptr;
  unsigned want = std::strtoul(line.c_str() + star + 1, &end, 16);
  return end == line.c_str() + star + 3 && chk == want;
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

static bool ascii_checksum_ok(const std::string& line) {
  if (line.size() < 4) return false;
  if (line[0] != '$' && line[0] != '#' && line[0] != '%') return false;
  size_t star = line.find('*');
  if (star == std::string::npos) return false;
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

static std::optional<uint64_t> parse_optional_u64(const std::string& s) {
  if (s.empty()) return std::nullopt;
  try {
    return static_cast<uint64_t>(std::stoull(s));
  } catch (...) {
    return std::nullopt;
  }
}

static std::optional<int> parse_optional_int(const std::string& s) {
  if (s.empty()) return std::nullopt;
  try {
    return std::stoi(s);
  } catch (...) {
    return std::nullopt;
  }
}

static std::optional<double> parse_optional_double(const std::string& s) {
  if (s.empty()) return std::nullopt;
  char* end = nullptr;
  const double value = std::strtod(s.c_str(), &end);
  if (!end || *end != '\0') return std::nullopt;
  return value;
}

static bool parse_nmea_hms(const std::string& s, uint8_t& hour, uint8_t& min,
                           uint8_t& sec, uint32_t& nanos) {
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

static bool parse_nmea_date_ddmmyy(const std::string& s, uint16_t& year,
                                   uint8_t& mon, uint8_t& day) {
  if (s.size() != 6) return false;
  day = static_cast<uint8_t>(std::atoi(s.substr(0, 2).c_str()));
  mon = static_cast<uint8_t>(std::atoi(s.substr(2, 2).c_str()));
  int yy = std::atoi(s.substr(4, 2).c_str());
  year = static_cast<uint16_t>((yy >= 80) ? (1900 + yy) : (2000 + yy));
  return true;
}

static bool parse_nmea_latlon(const std::string& value, const std::string& hemi,
                              bool is_lat, double& deg_out) {
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

static double wrap_angle_deg(double angle_deg) {
  double out = std::fmod(angle_deg + 180.0, 360.0);
  if (out < 0.0) out += 360.0;
  return out - 180.0;
}

static bool gps_week_tow_to_unix_ns(uint16_t gps_week, double tow_sec, uint64_t& out_ns) {
  if (gps_week == 0 || tow_sec < 0.0 || !std::isfinite(tow_sec)) return false;
  constexpr uint64_t gps_epoch_unix_ns = 315964800ULL * 1000000000ULL;
  constexpr int64_t gps_minus_utc_sec = 18;
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

struct CameraConfig {
  std::string name;
  std::string device;
  int width = 3840;
  int height = 2160;
  int fps = 30;
  int bitrate = 80000000;
  int chunk_sec = 60;
  std::string container = "mkv";
  std::string codec = "h265";
};

struct RecorderConfig {
  fs::path config_path;
  bool config_loaded = false;
  fs::path output_root = "/workspace/datasample";
  std::string session_id;
  std::string vehicle_id = "test_car_01";
  std::string timezone = "UTC";
  std::vector<CameraConfig> cameras;
  std::string imu_dev;
  fs::path sensor_dump;
  int imu_baud = 115200;
  int camera_fps = 30;
  int camera_width = 3840;
  int camera_height = 2160;
  int camera_bitrate = 80000000;
  std::string camera_container = "mkv";
  std::string camera_codec = "h265";
  int imu_hz = 100;
  int gnss_hz = 10;
  int duration_sec = 0;
  int chunk_sec = 60;
  bool finalize_spool = true;
  bool cleanup_spool = false;
  bool validate_session = false;
};

static void sync_camera_common_settings(RecorderConfig& cfg);

static std::string camera_frames_header() {
  return "frame_id,chunk_id,frame_index_in_chunk,timestamp_utc_ns,timestamp_monotonic_ns,width,height,exposure_time_us,gain,dropped_frame,trigger_id,seq,flags,file_size_bytes,gst_pts_ns";
}

static std::string imu_header() {
  return "row_id,timestamp_utc_ns,timestamp_monotonic_ns,ax,ay,az,gx,gy,gz,roll_deg,pitch_deg,yaw_deg,temperature,seq";
}

static std::string gnss_header() {
  return "row_id,timestamp_utc_ns,timestamp_monotonic_ns,lat,lon,alt,fix_type,num_sats,hdop,vdop,x_local,y_local,z_local,qw,qx,qy,qz,vx,vy,vz,pc,utc_valid";
}

static void write_text_file(const fs::path& path, const std::string& text) {
  fs::create_directories(path.parent_path());
  std::ofstream ofs(path);
  ofs << text;
}

static std::string make_session_meta_json(const RecorderConfig& cfg) {
  std::ostringstream os;
  os << "{\n";
  if (cfg.config_loaded) {
    os << "  \"config_path\": \"" << json_escape(cfg.config_path.string()) << "\",\n";
  }
  os << "  \"session_id\": \"" << json_escape(cfg.session_id) << "\",\n";
  os << "  \"vehicle_id\": \"" << json_escape(cfg.vehicle_id) << "\",\n";
  os << "  \"timezone\": \"" << json_escape(cfg.timezone) << "\",\n";
  os << "  \"time_sync\": {\n";
  os << "    \"master_clock\": \"utc\",\n";
  os << "    \"utc_source\": \"gnss_or_system\",\n";
  os << "    \"store_monotonic\": true\n";
  os << "  },\n";
  os << "  \"camera_names\": [";
  for (size_t i = 0; i < cfg.cameras.size(); ++i) {
    if (i) os << ", ";
    os << "\"" << json_escape(cfg.cameras[i].name) << "\"";
  }
  os << "],\n";
  os << "  \"camera_devices\": {\n";
  for (size_t i = 0; i < cfg.cameras.size(); ++i) {
    os << "    \"" << json_escape(cfg.cameras[i].name) << "\": \"" << json_escape(cfg.cameras[i].device) << "\"";
    os << (i + 1 == cfg.cameras.size() ? "\n" : ",\n");
  }
  os << "  },\n";
  os << "  \"camera_fps\": " << cfg.camera_fps << ",\n";
  os << "  \"camera_chunk_backend\": \"gstreamer\",\n";
  os << "  \"camera_chunk_codec\": \"nvv4l2" << json_escape(cfg.camera_codec) << "\",\n";
  os << "  \"camera_container\": \"" << json_escape(cfg.camera_container) << "\",\n";
  os << "  \"camera_chunk_seconds\": " << cfg.chunk_sec << ",\n";
  os << "  \"imu_hz\": " << cfg.imu_hz << ",\n";
  os << "  \"gnss_ins_hz\": " << cfg.gnss_hz << "\n";
  os << "}\n";
  return os.str();
}

static std::string make_intrinsics_json(const RecorderConfig& cfg) {
  std::ostringstream os;
  os << "{\n";
  for (size_t i = 0; i < cfg.cameras.size(); ++i) {
    const auto& cam = cfg.cameras[i];
    os << "  \"camera_" << json_escape(cam.name) << "\": {\n";
    os << "    \"image_size\": [" << cam.width << ", " << cam.height << "],\n";
    os << "    \"focal_length_px\": null,\n";
    os << "    \"principal_point_px\": null,\n";
    os << "    \"distortion_model\": null\n";
    os << "  }" << (i + 1 == cfg.cameras.size() ? "\n" : ",\n");
  }
  os << "}\n";
  return os.str();
}

static std::string make_mounts_json(const RecorderConfig& cfg) {
  std::ostringstream os;
  os << "{\n";
  for (size_t i = 0; i < cfg.cameras.size(); ++i) {
    const auto& cam = cfg.cameras[i];
    os << "  \"camera_" << json_escape(cam.name) << "\": {\n";
    os << "    \"description\": \"" << json_escape(cam.name) << "\",\n";
    os << "    \"nominal_fov_deg\": null\n";
    os << "  },\n";
  }
  os << "  \"imu\": {\n";
  os << "    \"description\": \"vehicle body mounted\"\n";
  os << "  },\n";
  os << "  \"gnss_ins\": {\n";
  os << "    \"description\": \"roof antenna or fused navigation output\"\n";
  os << "  }\n";
  os << "}\n";
  return os.str();
}

static std::string csv_field_i64(std::optional<int64_t> v) {
  return v ? std::to_string(*v) : std::string();
}

static std::string csv_field_u64(std::optional<uint64_t> v) {
  return v ? std::to_string(*v) : std::string();
}

static std::string csv_field_double(std::optional<double> v) {
  if (!v) return std::string();
  std::ostringstream os;
  os << std::setprecision(12) << *v;
  return os.str();
}

class CameraRecorder {
 public:
  CameraRecorder(CameraConfig cfg,
                 fs::path session_dir,
                 CsvSpoolWriter* writer,
                 UtcMapper* utc_mapper,
                 TimeAnchors anchors)
      : cfg_(std::move(cfg)),
        session_dir_(std::move(session_dir)),
        writer_(writer),
        utc_mapper_(utc_mapper),
        anchors_(anchors) {}

  ~CameraRecorder() { destroy(); }

  bool start() {
    std::string ext = (cfg_.container == "mkv") ? "mkv" : "mp4";
    std::string muxer = (cfg_.container == "mkv") ? "matroskamux" : "qtmux";
    std::string parser = (cfg_.codec == "h264") ? "h264parse" : "h265parse";
    std::string encoder = (cfg_.codec == "h264") ? "nvv4l2h264enc" : "nvv4l2h265enc";

    chunk_ext_ = ext;
    chunks_dir_ = session_dir_ / "sensors" / ("camera_" + cfg_.name) / "chunks";
    warmup_dir_ = session_dir_ / ".warmup" / ("camera_" + cfg_.name);
    fs::create_directories(chunks_dir_);
    fs::create_directories(warmup_dir_);

    std::ostringstream pipeline;
    pipeline
        << "v4l2src name=src device=" << cfg_.device << " io-mode=2 do-timestamp=false ! "
        << "video/x-raw,format=UYVY,width=" << cfg_.width << ",height=" << cfg_.height
        << ",framerate=" << cfg_.fps << "/1 ! "
        << "queue max-size-buffers=4 max-size-bytes=0 max-size-time=0 ! "
        << "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
        << encoder << " bitrate=" << cfg_.bitrate
        << " iframeinterval=" << cfg_.fps
        << " control-rate=1 ! "
        << parser << " ! "
        << "identity name=idxprobe signal-handoffs=true silent=true ! "
        << "splitmuxsink name=chunksink async-finalize=true muxer-factory=" << muxer
        << " max-size-time=" << (static_cast<uint64_t>(cfg_.chunk_sec) * 1000000000ULL)
        << " send-keyframe-requests=true";

    GError* error = nullptr;
    pipeline_ = gst_parse_launch(pipeline.str().c_str(), &error);
    if (!pipeline_) {
      std::cerr << "[CAM " << cfg_.name << "] failed to build pipeline: "
                << (error ? error->message : "unknown") << "\n";
      if (error) g_error_free(error);
      return false;
    }
    if (error) {
      std::cerr << "[CAM " << cfg_.name << "] pipeline warning: " << error->message << "\n";
      g_error_free(error);
    }

    meta_sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "idxprobe");
    split_sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "chunksink");
    if (!meta_sink_ || !split_sink_) {
      std::cerr << "[CAM " << cfg_.name << "] failed to get named elements\n";
      return false;
    }

    g_signal_connect(meta_sink_, "handoff", G_CALLBACK(&CameraRecorder::on_identity_handoff), this);
    g_signal_connect(split_sink_, "format-location-full", G_CALLBACK(&CameraRecorder::on_format_location_full), this);

    bus_ = gst_element_get_bus(pipeline_);
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      std::cerr << "[CAM " << cfg_.name << "] failed to go to PLAYING\n";
      return false;
    }
    warmup_start_master_ns_ = now_master_ns();
    started_ = true;
    return true;
  }

  void request_stop() {
    if (!pipeline_ || stop_requested_) return;
    stop_requested_ = true;
    gst_element_send_event(pipeline_, gst_event_new_eos());
  }

  void force_stop() {
    if (!pipeline_) return;
    gst_element_set_state(pipeline_, GST_STATE_NULL);
  }

  void poll_bus() {
    if (!bus_) return;
    while (true) {
      GstMessage* msg = gst_bus_pop(bus_);
      if (!msg) break;
      handle_bus_message(msg);
      gst_message_unref(msg);
    }
  }

  bool eos_received() const { return eos_received_; }
  bool has_error() const { return has_error_; }
  uint64_t frame_count() const { return frame_count_.load(); }
  uint64_t dropped_gaps() const { return dropped_gap_count_.load(); }
  bool ready() const { return ready_.load(); }
  bool capture_started() const { return capture_active_.load(); }
  std::optional<uint64_t> first_master_ns() const { return first_master_ns_; }
  std::optional<uint64_t> last_master_ns() const { return last_master_ns_; }
  const CameraConfig& config() const { return cfg_; }
  std::string chunk_extension() const { return chunk_ext_; }

  double recent_fps() const {
    std::lock_guard<std::mutex> lk(warmup_mu_);
    return recent_fps_locked();
  }

  std::string status_string() const {
    std::ostringstream os;
    os << cfg_.name << "=" << warmup_frame_count_.load() << "frames";
    os << "(" << std::fixed << std::setprecision(2) << recent_fps() << "fps";
    os << ",gaps=" << recent_gap_count() << ")";
    os << (ready() ? "[ready]" : "[warming]");
    return os.str();
  }

  void arm_capture() {
    if (capture_requested_.exchange(true)) return;
    {
      std::lock_guard<std::mutex> lk(chunk_mu_);
      chunk_boundaries_.clear();
      next_frame_index_in_chunk_.clear();
      csv_counts_by_chunk_.clear();
      record_fragment_base_.reset();
    }
    capture_active_.store(false);
    frame_count_.store(0);
    dropped_gap_count_.store(0);
    first_master_ns_.reset();
    last_master_ns_.reset();
    last_pts_ns_.reset();
    if (split_sink_) {
      g_signal_emit_by_name(split_sink_, "split-now");
    }
  }

  std::map<uint32_t, uint64_t> csv_counts_by_chunk() const {
    std::lock_guard<std::mutex> lk(chunk_mu_);
    return csv_counts_by_chunk_;
  }

 private:
  struct ChunkBoundary {
    uint32_t chunk_id;
    uint64_t start_pts_ns;
  };

  static gchar* on_format_location_full(GstElement*, guint fragment_id, GstSample* sample, gpointer user_data) {
    auto* self = static_cast<CameraRecorder*>(user_data);
    uint64_t start_pts_ns = 0;
    if (sample) {
      GstBuffer* buffer = gst_sample_get_buffer(sample);
      if (buffer && GST_BUFFER_PTS_IS_VALID(buffer)) {
        start_pts_ns = GST_BUFFER_PTS(buffer);
      }
    }
    fs::path path = self->resolve_chunk_path(static_cast<uint32_t>(fragment_id), start_pts_ns);
    return g_strdup(path.string().c_str());
  }

  static void on_identity_handoff(GstElement*, GstBuffer* buffer, gpointer user_data) {
    auto* self = static_cast<CameraRecorder*>(user_data);
    self->handle_buffer(buffer);
  }

  void register_chunk_boundary(uint32_t chunk_id, uint64_t start_pts_ns) {
    std::lock_guard<std::mutex> lk(chunk_mu_);
    chunk_boundaries_.push_back({chunk_id, start_pts_ns});
    std::sort(chunk_boundaries_.begin(), chunk_boundaries_.end(), [](const ChunkBoundary& a, const ChunkBoundary& b) {
      if (a.start_pts_ns == b.start_pts_ns) return a.chunk_id < b.chunk_id;
      return a.start_pts_ns < b.start_pts_ns;
    });
    next_frame_index_in_chunk_[chunk_id] = 0;
  }

  std::pair<uint32_t, uint64_t> assign_chunk_for_pts(uint64_t pts_ns) {
    std::lock_guard<std::mutex> lk(chunk_mu_);
    uint32_t chunk_id = 0;
    if (!chunk_boundaries_.empty()) {
      for (const auto& boundary : chunk_boundaries_) {
        if (boundary.start_pts_ns <= pts_ns) {
          chunk_id = boundary.chunk_id;
        } else {
          break;
        }
      }
    }
    uint64_t frame_index = next_frame_index_in_chunk_[chunk_id]++;
    csv_counts_by_chunk_[chunk_id]++;
    return {chunk_id, frame_index};
  }

  double recent_fps_locked() const {
    if (warmup_recent_pts_.size() < 2) return 0.0;
    const uint64_t span = warmup_recent_pts_.back() - warmup_recent_pts_.front();
    if (span == 0) return 0.0;
    return static_cast<double>(warmup_recent_pts_.size() - 1) / (static_cast<double>(span) * 1e-9);
  }

  uint64_t recent_gap_count() const {
    std::lock_guard<std::mutex> lk(warmup_mu_);
    return recent_gap_count_locked();
  }

  uint64_t recent_gap_count_locked() const {
    if (warmup_recent_pts_.size() < 2) return 0;
    const uint64_t expected = static_cast<uint64_t>(1000000000ULL / std::max(1, cfg_.fps));
    uint64_t gaps = 0;
    for (size_t i = 1; i < warmup_recent_pts_.size(); ++i) {
      const uint64_t delta = warmup_recent_pts_[i] - warmup_recent_pts_[i - 1];
      if (delta > expected + expected / 2) {
        gaps += std::max<uint64_t>(1, delta / expected - 1);
      }
    }
    return gaps;
  }

  void update_warmup_state(uint64_t pts_ns) {
    bool became_ready = false;
    {
      std::lock_guard<std::mutex> lk(warmup_mu_);
      warmup_recent_pts_.push_back(pts_ns);
      while (warmup_recent_pts_.size() > 90) warmup_recent_pts_.pop_front();
      const uint64_t warmup_frames = warmup_frame_count_.fetch_add(1) + 1;
      if (!ready_.load() && warmup_frames >= 90 && warmup_recent_pts_.size() >= 60) {
        const double fps = recent_fps_locked();
        const uint64_t gaps = recent_gap_count_locked();
        if (fps >= static_cast<double>(cfg_.fps) * 0.97 &&
            fps <= static_cast<double>(cfg_.fps) * 1.03 &&
            gaps == 0 &&
            (now_master_ns() - warmup_start_master_ns_) >= 2000000000ULL) {
          ready_.store(true);
          became_ready = true;
        }
      }
    }
    if (became_ready) {
      log_ready("camera_" + cfg_.name + " warmup stable at " + status_string());
    }
  }

  fs::path resolve_chunk_path(uint32_t fragment_id, uint64_t start_pts_ns) {
    if (!capture_requested_.load()) {
      return warmup_dir_ / warmup_chunk_filename(fragment_id);
    }

    std::lock_guard<std::mutex> lk(chunk_mu_);
    if (!record_fragment_base_) {
      record_fragment_base_ = fragment_id;
      chunk_boundaries_.clear();
      next_frame_index_in_chunk_.clear();
      csv_counts_by_chunk_.clear();
      capture_active_.store(true);
      log_ready("camera_" + cfg_.name + " switched to clean capture chunk");
    }
    const uint32_t recorded_chunk_id = fragment_id - *record_fragment_base_;
    chunk_boundaries_.push_back({recorded_chunk_id, start_pts_ns});
    std::sort(chunk_boundaries_.begin(), chunk_boundaries_.end(), [](const ChunkBoundary& a, const ChunkBoundary& b) {
      if (a.start_pts_ns == b.start_pts_ns) return a.chunk_id < b.chunk_id;
      return a.start_pts_ns < b.start_pts_ns;
    });
    next_frame_index_in_chunk_[recorded_chunk_id] = 0;
    return chunks_dir_ / chunk_filename(recorded_chunk_id);
  }

  uint64_t normalize_pts_to_master(uint64_t pts_ns) const {
    if (pts_ns == GST_CLOCK_TIME_NONE) return now_master_ns();
    if (pts_ns < 1000000000000ULL) {
      return anchors_.master_anchor_ns + pts_ns;
    }
    return pts_ns;
  }

  void handle_buffer(GstBuffer* buffer) {
    uint64_t pts_ns = GST_BUFFER_PTS_IS_VALID(buffer) ? GST_BUFFER_PTS(buffer) : GST_CLOCK_TIME_NONE;
    uint64_t master_ns = normalize_pts_to_master(pts_ns);
    uint64_t utc_ns = 0;
    if (!utc_mapper_->map(master_ns, utc_ns)) {
      utc_ns = utc_mapper_->fallback_from_anchor(master_ns);
    }

    update_warmup_state(pts_ns == GST_CLOCK_TIME_NONE ? master_ns : pts_ns);
    if (!capture_active_.load()) return;

    if (!first_master_ns_) first_master_ns_ = master_ns;
    last_master_ns_ = master_ns;

    auto [chunk_id, frame_index] = assign_chunk_for_pts(pts_ns == GST_CLOCK_TIME_NONE ? 0 : pts_ns);
    uint64_t frame_id = frame_count_.fetch_add(1);
    bool dropped_frame = false;
    if (last_pts_ns_ && pts_ns != GST_CLOCK_TIME_NONE) {
      uint64_t expected = static_cast<uint64_t>(1000000000ULL / std::max(1, cfg_.fps));
      uint64_t delta = pts_ns - *last_pts_ns_;
      if (delta > expected + expected / 2) {
        dropped_frame = true;
        if (expected > 0) {
          dropped_gap_count_.fetch_add(std::max<uint64_t>(1, delta / expected - 1));
        } else {
          dropped_gap_count_.fetch_add(1);
        }
      }
    }
    if (pts_ns != GST_CLOCK_TIME_NONE) last_pts_ns_ = pts_ns;

    uint64_t seq = (GST_BUFFER_OFFSET(buffer) != GST_BUFFER_OFFSET_NONE) ? GST_BUFFER_OFFSET(buffer) : frame_id;
    uint32_t flags = static_cast<uint32_t>(GST_BUFFER_FLAGS(buffer));

    std::ostringstream row;
    row << frame_id << ','
        << chunk_id << ','
        << frame_index << ','
        << utc_ns << ','
        << master_ns << ','
        << cfg_.width << ','
        << cfg_.height << ','
        << ','
        << ','
        << (dropped_frame ? "true" : "false") << ','
        << ','
        << seq << ','
        << flags << ','
        << ','
        << (pts_ns == GST_CLOCK_TIME_NONE ? 0 : pts_ns);
    writer_->push(row.str());
  }

  std::string chunk_filename(uint32_t chunk_id) const {
    std::ostringstream os;
    os << "chunk_" << std::setw(4) << std::setfill('0') << chunk_id << '.' << chunk_ext_;
    return os.str();
  }

  std::string warmup_chunk_filename(uint32_t chunk_id) const {
    std::ostringstream os;
    os << "warmup_" << std::setw(4) << std::setfill('0') << chunk_id << '.' << chunk_ext_;
    return os.str();
  }

  void handle_bus_message(GstMessage* msg) {
    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_ERROR: {
        GError* err = nullptr;
        gchar* dbg = nullptr;
        gst_message_parse_error(msg, &err, &dbg);
        std::cerr << "[CAM " << cfg_.name << "] ERROR: "
                  << (err ? err->message : "unknown")
                  << (dbg ? std::string(" | ") + dbg : std::string()) << "\n";
        if (err) g_error_free(err);
        if (dbg) g_free(dbg);
        has_error_ = true;
        break;
      }
      case GST_MESSAGE_EOS:
        eos_received_ = true;
        break;
      case GST_MESSAGE_WARNING: {
        GError* err = nullptr;
        gchar* dbg = nullptr;
        gst_message_parse_warning(msg, &err, &dbg);
        std::cerr << "[CAM " << cfg_.name << "] WARNING: "
                  << (err ? err->message : "unknown")
                  << (dbg ? std::string(" | ") + dbg : std::string()) << "\n";
        if (err) g_error_free(err);
        if (dbg) g_free(dbg);
        break;
      }
      default:
        break;
    }
  }

  void destroy() {
    if (pipeline_) gst_element_set_state(pipeline_, GST_STATE_NULL);
    if (bus_) gst_object_unref(bus_);
    bus_ = nullptr;
    if (meta_sink_) gst_object_unref(meta_sink_);
    meta_sink_ = nullptr;
    if (split_sink_) gst_object_unref(split_sink_);
    split_sink_ = nullptr;
    if (pipeline_) gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    std::error_code ec;
    if (!warmup_dir_.empty()) fs::remove_all(warmup_dir_, ec);
  }

  CameraConfig cfg_;
  fs::path session_dir_;
  fs::path chunks_dir_;
  fs::path warmup_dir_;
  CsvSpoolWriter* writer_;
  UtcMapper* utc_mapper_;
  TimeAnchors anchors_;
  GstElement* pipeline_{nullptr};
  GstElement* meta_sink_{nullptr};
  GstElement* split_sink_{nullptr};
  GstBus* bus_{nullptr};
  std::string chunk_ext_;
  bool started_{false};
  bool stop_requested_{false};
  std::atomic<bool> ready_{false};
  std::atomic<bool> capture_requested_{false};
  std::atomic<bool> capture_active_{false};
  std::atomic<bool> eos_received_{false};
  std::atomic<bool> has_error_{false};
  std::atomic<uint64_t> frame_count_{0};
  std::atomic<uint64_t> dropped_gap_count_{0};
  std::atomic<uint64_t> warmup_frame_count_{0};
  std::optional<uint64_t> first_master_ns_;
  std::optional<uint64_t> last_master_ns_;
  std::optional<uint64_t> last_pts_ns_;
  mutable std::mutex chunk_mu_;
  std::vector<ChunkBoundary> chunk_boundaries_;
  std::unordered_map<uint32_t, uint64_t> next_frame_index_in_chunk_;
  std::map<uint32_t, uint64_t> csv_counts_by_chunk_;
  std::optional<uint32_t> record_fragment_base_;
  mutable std::mutex warmup_mu_;
  std::deque<uint64_t> warmup_recent_pts_;
  uint64_t warmup_start_master_ns_{0};
};

class SerialSensorRecorder {
 public:
  struct ImuSnapshot {
    bool valid = false;
    uint64_t master_ns = 0;
    uint64_t utc_ns = 0;
    uint16_t pc = 0;
    bool have_acc = false;
    double acc[3] = {0.0, 0.0, 0.0};
    bool have_gyro = false;
    double gyro[3] = {0.0, 0.0, 0.0};
    bool have_euler = false;
    double euler_deg[3] = {0.0, 0.0, 0.0};
  };

  struct GnssSnapshot {
    bool valid = false;
    uint64_t master_ns = 0;
    uint64_t utc_ns = 0;
    std::optional<double> lat;
    std::optional<double> lon;
    std::optional<double> alt;
    std::optional<double> vx;
    std::optional<double> vy;
    std::optional<double> vz;
    uint16_t pc = 0;
    uint8_t utc_valid = 0;
  };

  SerialSensorRecorder(std::string dev,
                       fs::path dump_path,
                       int baud,
                       CsvSpoolWriter* imu_writer,
                       CsvSpoolWriter* gnss_writer,
                       UtcMapper* utc_mapper)
      : dev_(std::move(dev)),
        dump_path_(std::move(dump_path)),
        baud_(baud),
        imu_writer_(imu_writer),
        gnss_writer_(gnss_writer),
        utc_mapper_(utc_mapper) {}

  ~SerialSensorRecorder() { stop(); }

  bool start() {
    if (!dump_path_.empty()) {
      if (!fs::exists(dump_path_)) {
        log_error("sensor dump not found: " + dump_path_.string());
        return false;
      }
      if (!load_dump_events(dump_events_)) return false;
      imu_seen_.store(false);
      gnss_seen_.store(false);
      observed_imu_count_.store(0);
      observed_gnss_count_.store(0);
      for (const auto& ev : dump_events_) {
        if (ev.type == "imu") {
          imu_seen_.store(true);
          observed_imu_count_.fetch_add(1);
        } else if (ev.type == "gnss") {
          gnss_seen_.store(true);
          observed_gnss_count_.fetch_add(1);
        }
      }
      if (!ready_for_capture()) {
        log_error("sensor dump missing required imu/gnss samples");
        return false;
      }
      log_info("sensor dump preflight ready: imu=" + std::to_string(observed_imu_count_.load()) +
               " gnss=" + std::to_string(observed_gnss_count_.load()));
      return true;
    }
    if (dev_.empty()) {
      log_error("no IMU/GNSS source configured. Set --imu-dev or --sensor-dump");
      return false;
    }
    if (!fs::exists(dev_)) {
      log_error("IMU device not found: " + dev_);
      return false;
    }
    running_ = true;
    thread_ = std::thread([this]() { thread_main_serial(); });
    return true;
  }

  void stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
  }

  bool active() const { return running_; }
  bool ready_for_capture() const { return imu_seen_.load() && gnss_seen_.load(); }
  bool imu_ready() const { return imu_seen_.load(); }
  bool gnss_ready() const { return gnss_seen_.load(); }
  bool has_error() const { return has_error_.load(); }
  std::optional<ImuSnapshot> latest_imu_snapshot() const {
    std::lock_guard<std::mutex> lk(snapshot_mu_);
    if (!last_imu_snapshot_.valid) return std::nullopt;
    return last_imu_snapshot_;
  }
  std::optional<GnssSnapshot> latest_gnss_snapshot() const {
    std::lock_guard<std::mutex> lk(snapshot_mu_);
    if (last_gnss_fix_snapshot_.valid) return last_gnss_fix_snapshot_;
    if (last_gnss_snapshot_.valid) return last_gnss_snapshot_;
    return std::nullopt;
  }
  std::string latest_imu_log_string() const { return format_imu_snapshot(latest_imu_snapshot()); }
  std::string latest_gnss_log_string() const { return format_gnss_snapshot(latest_gnss_snapshot()); }

  void arm_capture() {
    capture_enabled_.store(true);
    if (!dump_path_.empty() && !running_) {
      running_ = true;
      thread_ = std::thread([this]() { thread_main_dump(); });
    }
  }

  std::string status_string() const {
    std::ostringstream os;
    os << "imu=" << (imu_seen_.load() ? "ready" : "wait")
       << "(" << observed_imu_count_.load() << ")"
       << " gnss=" << (gnss_seen_.load() ? "ready" : "wait")
       << "(" << observed_gnss_count_.load() << ")";
    if (capture_enabled_.load()) os << " capture=armed";
    return os.str();
  }

 private:
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
    auto parsed = parse_optional_double(text);
    if (!parsed || !std::isfinite(*parsed)) return false;
    out = *parsed;
    return true;
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

  static std::string format_optional_double(const std::optional<double>& value, int precision = 6) {
    if (!value) return "null";
    std::ostringstream os;
    os << std::fixed << std::setprecision(precision) << *value;
    return os.str();
  }

  static std::string format_imu_snapshot(const std::optional<ImuSnapshot>& snapshot) {
    if (!snapshot) return "IMU no-sample";
    std::ostringstream os;
    os << std::fixed << std::setprecision(6)
       << "IMU ts=" << snapshot->master_ns
       << " utc_ns=" << snapshot->utc_ns
       << " pc=" << snapshot->pc;
    if (snapshot->have_acc) {
      os << " acc=[" << snapshot->acc[0] << "," << snapshot->acc[1] << "," << snapshot->acc[2] << "]";
    } else {
      os << " acc=[null]";
    }
    if (snapshot->have_gyro) {
      os << " gyro=[" << snapshot->gyro[0] << "," << snapshot->gyro[1] << "," << snapshot->gyro[2] << "]";
    } else {
      os << " gyro=[null]";
    }
    if (snapshot->have_euler) {
      os << " euler_deg=[" << snapshot->euler_deg[0] << "," << snapshot->euler_deg[1] << "," << snapshot->euler_deg[2] << "]";
    } else {
      os << " euler_deg=[null]";
    }
    return os.str();
  }

  static std::string format_gnss_snapshot(const std::optional<GnssSnapshot>& snapshot) {
    if (!snapshot) return "GNSS no-sample";
    std::ostringstream os;
    os << "GNSS ts=" << snapshot->master_ns
       << " utc_ns=" << snapshot->utc_ns
       << " lat=" << format_optional_double(snapshot->lat, 7)
       << " lon=" << format_optional_double(snapshot->lon, 7)
       << " alt_m=" << format_optional_double(snapshot->alt, 3)
       << " vel=[" << format_optional_double(snapshot->vx, 3)
       << "," << format_optional_double(snapshot->vy, 3)
       << "," << format_optional_double(snapshot->vz, 3) << "]"
       << " pc=" << snapshot->pc
       << " utc_valid=0x" << std::hex << std::nouppercase
       << static_cast<unsigned>(snapshot->utc_valid) << std::dec;
    return os.str();
  }

  void mark_error(const std::string& message) {
    if (!has_error_.exchange(true)) {
      log_error(message);
    }
  }

  bool parse_ascii_log(const std::string& line, ParsedAsciiLog& out) const {
    if (!ascii_checksum_ok(line)) return false;
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
      utc_valid = 0x03;
      utc_mapper_->update(t_rx_ns, utc_ns);
      return true;
    }
    if (utc_mapper_->map(t_rx_ns, utc_ns)) return true;
    utc_ns = utc_mapper_->fallback_from_anchor(t_rx_ns);
    return true;
  }

  bool parse_proprietary_header_week_tow(const std::vector<std::string>& header_fields,
                                         uint16_t& gps_week, double& tow_sec) const {
    for (size_t i = 0; i + 1 < header_fields.size(); ++i) {
      uint16_t maybe_week = 0;
      double maybe_tow = 0.0;
      if (!try_parse_uint16(header_fields[i], maybe_week) || maybe_week < 1000 || maybe_week > 5000) continue;
      if (!try_parse_double(header_fields[i + 1], maybe_tow) ||
          maybe_tow < 0.0 || maybe_tow > 7.0 * 86400.0 + 1.0) {
        continue;
      }
      gps_week = maybe_week;
      tow_sec = maybe_tow;
      return true;
    }
    return false;
  }

  void emit_imu_row(uint64_t master_ns, uint64_t utc_ns, uint16_t pc,
                    bool have_acc, const float acc[3],
                    bool have_gyro, const float gyro[3],
                    bool have_euler, const double euler_deg[3]) {
    imu_seen_.store(true);
    observed_imu_count_.fetch_add(1);
    last_imu_master_ns_.store(master_ns);
    {
      std::lock_guard<std::mutex> lk(snapshot_mu_);
      last_imu_snapshot_.valid = true;
      last_imu_snapshot_.master_ns = master_ns;
      last_imu_snapshot_.utc_ns = utc_ns;
      last_imu_snapshot_.pc = pc;
      last_imu_snapshot_.have_acc = have_acc;
      if (have_acc) {
        last_imu_snapshot_.acc[0] = acc[0];
        last_imu_snapshot_.acc[1] = acc[1];
        last_imu_snapshot_.acc[2] = acc[2];
      }
      last_imu_snapshot_.have_gyro = have_gyro;
      if (have_gyro) {
        last_imu_snapshot_.gyro[0] = gyro[0];
        last_imu_snapshot_.gyro[1] = gyro[1];
        last_imu_snapshot_.gyro[2] = gyro[2];
      }
      last_imu_snapshot_.have_euler = have_euler;
      if (have_euler) {
        last_imu_snapshot_.euler_deg[0] = euler_deg[0];
        last_imu_snapshot_.euler_deg[1] = euler_deg[1];
        last_imu_snapshot_.euler_deg[2] = euler_deg[2];
      }
    }
    if (!capture_enabled_.load()) return;
    std::ostringstream row;
    row << imu_row_id_++ << ','
        << utc_ns << ','
        << master_ns << ','
        << (have_acc ? csv_field_double(acc[0]) : "") << ','
        << (have_acc ? csv_field_double(acc[1]) : "") << ','
        << (have_acc ? csv_field_double(acc[2]) : "") << ','
        << (have_gyro ? csv_field_double(gyro[0]) : "") << ','
        << (have_gyro ? csv_field_double(gyro[1]) : "") << ','
        << (have_gyro ? csv_field_double(gyro[2]) : "") << ','
        << (have_euler ? csv_field_double(euler_deg[0]) : "") << ','
        << (have_euler ? csv_field_double(euler_deg[1]) : "") << ','
        << (have_euler ? csv_field_double(euler_deg[2]) : "") << ','
        << ','
        << pc;
    imu_writer_->push(row.str());
  }

  void emit_gnss_row(uint64_t master_ns, uint64_t utc_ns,
                     std::optional<double> lat, std::optional<double> lon,
                     std::optional<double> alt,
                     std::optional<int> fix_type,
                     std::optional<int> num_sats,
                     std::optional<double> hdop,
                     std::optional<double> vdop,
                     std::optional<double> vx,
                     std::optional<double> vy,
                     std::optional<double> vz,
                     uint16_t pc,
                     uint8_t utc_valid) {
    gnss_seen_.store(true);
    observed_gnss_count_.fetch_add(1);
    last_gnss_master_ns_.store(master_ns);
    {
      std::lock_guard<std::mutex> lk(snapshot_mu_);
      last_gnss_snapshot_.valid = true;
      last_gnss_snapshot_.master_ns = master_ns;
      last_gnss_snapshot_.utc_ns = utc_ns;
      last_gnss_snapshot_.lat = lat;
      last_gnss_snapshot_.lon = lon;
      last_gnss_snapshot_.alt = alt;
      last_gnss_snapshot_.vx = vx;
      last_gnss_snapshot_.vy = vy;
      last_gnss_snapshot_.vz = vz;
      last_gnss_snapshot_.pc = pc;
      last_gnss_snapshot_.utc_valid = utc_valid;
      if (lat || lon || alt || vx || vy || vz) {
        last_gnss_fix_snapshot_ = last_gnss_snapshot_;
      }
    }
    if (!capture_enabled_.load()) return;
    std::ostringstream row;
    row << gnss_row_id_++ << ','
        << utc_ns << ','
        << master_ns << ','
        << csv_field_double(lat) << ','
        << csv_field_double(lon) << ','
        << csv_field_double(alt) << ','
        << csv_field_i64(fix_type ? std::optional<int64_t>(*fix_type) : std::nullopt) << ','
        << csv_field_i64(num_sats ? std::optional<int64_t>(*num_sats) : std::nullopt) << ','
        << csv_field_double(hdop) << ','
        << csv_field_double(vdop) << ','
        << ',' << ',' << ',' << ',' << ',' << ',' << ','
        << csv_field_double(vx) << ','
        << csv_field_double(vy) << ','
        << csv_field_double(vz) << ','
        << pc << ','
        << static_cast<unsigned>(utc_valid);
    gnss_writer_->push(row.str());
  }

  struct DumpEvent {
    uint64_t offset_ns = 0;
    std::string type;
    std::optional<uint64_t> utc_ns;
    std::optional<double> ax;
    std::optional<double> ay;
    std::optional<double> az;
    std::optional<double> gx;
    std::optional<double> gy;
    std::optional<double> gz;
    std::optional<double> roll_deg;
    std::optional<double> pitch_deg;
    std::optional<double> yaw_deg;
    std::optional<double> lat;
    std::optional<double> lon;
    std::optional<double> alt;
    std::optional<int> fix_type;
    std::optional<int> num_sats;
    std::optional<double> hdop;
    std::optional<double> vdop;
    std::optional<double> vx;
    std::optional<double> vy;
    std::optional<double> vz;
    std::optional<int> seq;
    std::optional<int> pc;
    std::optional<int> utc_valid;
  };

  bool load_dump_events(std::vector<DumpEvent>& events) {
    events.clear();
    std::ifstream ifs(dump_path_);
    if (!ifs) {
      mark_error("failed to open sensor dump: " + dump_path_.string());
      return false;
    }

    std::string line;
    bool header_seen = false;
    while (std::getline(ifs, line)) {
      if (line.empty() || line[0] == '#') continue;
      auto fields = split_csv(line);
      if (fields.empty()) continue;
      if (!header_seen && fields[0] == "offset_ns") {
        header_seen = true;
        continue;
      }
      header_seen = true;
      if (fields.size() < 22) {
        mark_error("malformed sensor dump row");
        return false;
      }

      DumpEvent ev;
      auto offset = parse_optional_u64(fields[0]);
      if (!offset) {
        mark_error("invalid sensor dump offset");
        return false;
      }
      ev.offset_ns = *offset;
      ev.type = fields[1];
      ev.utc_ns = parse_optional_u64(fields[2]);
      ev.ax = parse_optional_double(fields[3]);
      ev.ay = parse_optional_double(fields[4]);
      ev.az = parse_optional_double(fields[5]);
      ev.gx = parse_optional_double(fields[6]);
      ev.gy = parse_optional_double(fields[7]);
      ev.gz = parse_optional_double(fields[8]);
      size_t next_index = 9;
      if (fields.size() >= 25) {
        ev.roll_deg = parse_optional_double(fields[next_index++]);
        ev.pitch_deg = parse_optional_double(fields[next_index++]);
        ev.yaw_deg = parse_optional_double(fields[next_index++]);
      }
      ev.lat = parse_optional_double(fields[next_index++]);
      ev.lon = parse_optional_double(fields[next_index++]);
      ev.alt = parse_optional_double(fields[next_index++]);
      ev.fix_type = parse_optional_int(fields[next_index++]);
      ev.num_sats = parse_optional_int(fields[next_index++]);
      ev.hdop = parse_optional_double(fields[next_index++]);
      ev.vdop = parse_optional_double(fields[next_index++]);
      ev.vx = parse_optional_double(fields[next_index++]);
      ev.vy = parse_optional_double(fields[next_index++]);
      ev.vz = parse_optional_double(fields[next_index++]);
      ev.seq = parse_optional_int(fields[next_index++]);
      ev.pc = parse_optional_int(fields[next_index++]);
      ev.utc_valid = parse_optional_int(fields[next_index++]);

      if (ev.type != "imu" && ev.type != "gnss") {
        mark_error("unsupported sensor dump row type: " + ev.type);
        return false;
      }
      events.push_back(std::move(ev));
    }

    if (events.empty()) {
      mark_error("sensor dump is empty: " + dump_path_.string());
      return false;
    }
    return true;
  }

  void sleep_until_ns(uint64_t target_ns) const {
    while (running_ && !g_stop.load()) {
      uint64_t now_ns = now_master_ns();
      if (now_ns >= target_ns) break;
      uint64_t remain_ns = target_ns - now_ns;
      if (remain_ns > 2000000ULL) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      } else if (remain_ns > 100000ULL) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      } else {
        std::this_thread::yield();
      }
    }
  }

  void thread_main_dump() {
    const uint64_t start_master_ns = now_master_ns();
    for (const auto& ev : dump_events_) {
      if (!running_ || g_stop.load()) break;
      const uint64_t master_ns = start_master_ns + ev.offset_ns;
      sleep_until_ns(master_ns);

      uint64_t utc_ns = 0;
      if (ev.utc_ns) {
        utc_ns = *ev.utc_ns;
      } else if (!utc_mapper_->map(master_ns, utc_ns)) {
        utc_ns = utc_mapper_->fallback_from_anchor(master_ns);
      }

      if (ev.type == "gnss") {
        const uint8_t utc_valid = static_cast<uint8_t>(ev.utc_valid.value_or(ev.utc_ns ? 0x03 : 0x00));
        if (ev.utc_ns && (utc_valid & 0x03) == 0x03) {
          utc_mapper_->update(master_ns, *ev.utc_ns);
        }
        emit_gnss_row(master_ns, utc_ns, ev.lat, ev.lon, ev.alt,
                      ev.fix_type, ev.num_sats, ev.hdop, ev.vdop,
                      ev.vx, ev.vy, ev.vz,
                      static_cast<uint16_t>(ev.pc.value_or(0)), utc_valid);
      } else {
        float acc[3]{
            static_cast<float>(ev.ax.value_or(0.0)),
            static_cast<float>(ev.ay.value_or(0.0)),
            static_cast<float>(ev.az.value_or(0.0)),
        };
        float gyro[3]{
            static_cast<float>(ev.gx.value_or(0.0)),
            static_cast<float>(ev.gy.value_or(0.0)),
            static_cast<float>(ev.gz.value_or(0.0)),
        };
        double euler_deg[3]{
            ev.roll_deg.value_or(0.0),
            ev.pitch_deg.value_or(0.0),
            ev.yaw_deg.value_or(0.0),
        };
        const uint16_t seq = static_cast<uint16_t>(ev.seq.value_or(ev.pc.value_or(0)));
        emit_imu_row(master_ns, utc_ns, seq,
                     ev.ax.has_value() || ev.ay.has_value() || ev.az.has_value(), acc,
                     ev.gx.has_value() || ev.gy.has_value() || ev.gz.has_value(), gyro,
                     ev.roll_deg.has_value() || ev.pitch_deg.has_value() || ev.yaw_deg.has_value(), euler_deg);
      }
    }
    running_ = false;
  }

  void process_nmea_sentence(const std::string& line, uint64_t t_rx_ns, uint16_t pc_last) {
    if (!nmea_checksum_ok(line)) return;
    size_t star = line.find('*');
    std::string body = line.substr(1, star - 1);
    auto fields = split_csv(body);
    if (fields.empty()) return;

    if (fields[0].size() >= 5 && fields[0].substr(fields[0].size() - 3) == "RMC") {
      if (fields.size() < 10 || fields[2] != "A") return;
      double lat = 0.0;
      double lon = 0.0;
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
      emit_gnss_row(t_rx_ns, utc_ns, lat, lon, std::nullopt,
                    std::optional<int>(1), std::nullopt, std::nullopt, std::nullopt,
                    fields.size() > 7 && !fields[7].empty()
                        ? std::optional<double>(std::strtod(fields[7].c_str(), nullptr) * 0.514444)
                        : std::nullopt,
                    std::nullopt, std::nullopt, pc_last, 0x03);
      nmea_have_date_ = true;
      nmea_year_ = year;
      nmea_mon_ = mon;
      nmea_day_ = day;
      return;
    }

    if (fields[0].size() >= 5 && fields[0].substr(fields[0].size() - 3) == "GGA") {
      if (fields.size() < 10) return;
      int fix_quality = fields[6].empty() ? 0 : std::atoi(fields[6].c_str());
      if (fix_quality <= 0) return;
      double lat = 0.0;
      double lon = 0.0;
      if (!parse_nmea_latlon(fields[2], fields[3], true, lat)) return;
      if (!parse_nmea_latlon(fields[4], fields[5], false, lon)) return;
      std::optional<double> alt;
      if (!fields[9].empty()) alt = std::strtod(fields[9].c_str(), nullptr);
      std::optional<double> hdop;
      if (fields.size() > 8 && !fields[8].empty()) hdop = std::strtod(fields[8].c_str(), nullptr);
      std::optional<int> num_sats;
      if (fields.size() > 7 && !fields[7].empty()) num_sats = std::atoi(fields[7].c_str());

      uint64_t utc_ns = 0;
      uint8_t utc_valid = 0;
      if (nmea_have_date_) {
        uint8_t hour = 0, min = 0, sec = 0;
        uint32_t nanos = 0;
        if (parse_nmea_hms(fields[1], hour, min, sec, nanos) &&
            xsens_utc_to_ns(nmea_year_, nmea_mon_, nmea_day_, hour, min, sec, nanos, utc_ns)) {
          utc_valid = 0x03;
          utc_mapper_->update(t_rx_ns, utc_ns);
        }
      }
      emit_gnss_row(t_rx_ns, utc_ns, lat, lon, alt, fix_quality, num_sats, hdop, std::nullopt,
                    std::nullopt, std::nullopt, std::nullopt, pc_last, utc_valid);
    }
  }

  void process_pppnava_sentence(const ParsedAsciiLog& log, uint64_t t_rx_ns, uint16_t pc_last) {
    if (log.body_fields.size() < 6) return;
    if (log.body_fields[0] != "SOL_COMPUTED") return;

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
    double tow_ms = 0.0;
    uint64_t utc_ns = 0;
    uint8_t utc_valid = 0;
    if (log.header_fields.size() > 5 &&
        try_parse_uint16(log.header_fields[4], gps_week) &&
        gps_week >= 1000 &&
        try_parse_double(log.header_fields[5], tow_ms) &&
        tow_ms >= 0.0 && tow_ms <= 7.0 * 86400.0 * 1000.0 &&
        parse_proprietary_time_from_week_tow(gps_week, tow_ms * 1e-3, t_rx_ns, utc_ns, utc_valid)) {
      emit_gnss_row(t_rx_ns, utc_ns, lat, lon, alt_m, 4, std::nullopt, std::nullopt, std::nullopt,
                    std::nullopt, std::nullopt, std::nullopt, pc_last, utc_valid);
      return;
    }

    parse_proprietary_time_from_week_tow(0, -1.0, t_rx_ns, utc_ns, utc_valid);
    emit_gnss_row(t_rx_ns, utc_ns, lat, lon, alt_m, 4, std::nullopt, std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt, std::nullopt, pc_last, utc_valid);
  }

  void process_inspva_sentence(const ParsedAsciiLog& log, uint64_t t_rx_ns, uint16_t pc_last) {
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
      double euler_deg[3]{roll_deg, pitch_deg, yaw_deg};
      emit_imu_row(t_rx_ns, utc_ns, pc_last, false, nullptr, false, nullptr, true, euler_deg);
      emit_gnss_row(t_rx_ns, utc_ns, lat, lon, alt_m, 4, std::nullopt, std::nullopt, std::nullopt,
                    ve, vn, vu, pc_last, utc_valid);
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
      double euler_deg[3]{roll_deg, pitch_deg, yaw_deg};
      emit_imu_row(t_rx_ns, utc_ns, pc_last, false, nullptr, false, nullptr, true, euler_deg);
      emit_gnss_row(t_rx_ns, utc_ns, lat, lon, hgt + undulation, 4, std::nullopt, std::nullopt, std::nullopt,
                    ve, vn, vu, pc_last, utc_valid);
    }
  }

  void process_drpvaa_sentence(const ParsedAsciiLog& log, uint64_t t_rx_ns, uint16_t pc_last) {
    if (log.body_fields.size() < 28) return;

    double lat = 0.0, lon = 0.0, hgt = 0.0, undulation = 0.0;
    double ve = 0.0, vn = 0.0, vu = 0.0;
    double heading_deg = 0.0, pitch_up_deg = 0.0, roll_deg = 0.0;
    if (!try_parse_double(log.body_fields[9], lat) ||
        !try_parse_double(log.body_fields[10], lon) ||
        !try_parse_double(log.body_fields[11], hgt) ||
        !try_parse_double(log.body_fields[12], undulation) ||
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
    double euler_deg[3]{roll_deg, pitch_deg, yaw_deg};
    emit_imu_row(t_rx_ns, utc_ns, pc_last, false, nullptr, false, nullptr, true, euler_deg);

    if (log.body_fields[0] != "INSUFFICIENT_OBS" && std::fabs(lat) > 1e-9 && std::fabs(lon) > 1e-9) {
      emit_gnss_row(t_rx_ns, utc_ns, lat, lon, hgt + undulation, 4, std::nullopt, std::nullopt, std::nullopt,
                    ve, vn, vu, pc_last, utc_valid);
    }
  }

  void process_rawimuxa_sentence(const ParsedAsciiLog& log, uint64_t t_rx_ns, uint16_t pc_last) {
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

    float acc[3]{
        static_cast<float>(static_cast<double>(x_acc_lsb) * scale.accel_delta_v_per_lsb * scale.rate_hz),
        static_cast<float>(-static_cast<double>(neg_y_acc_lsb) * scale.accel_delta_v_per_lsb * scale.rate_hz),
        static_cast<float>(static_cast<double>(z_acc_lsb) * scale.accel_delta_v_per_lsb * scale.rate_hz),
    };
    float gyro[3]{
        static_cast<float>(static_cast<double>(x_gyro_lsb) * scale.gyro_delta_deg_per_lsb_xy * scale.rate_hz),
        static_cast<float>(-static_cast<double>(neg_y_gyro_lsb) * scale.gyro_delta_deg_per_lsb_xy * scale.rate_hz),
        static_cast<float>(static_cast<double>(z_gyro_lsb) * scale.gyro_delta_deg_per_lsb_z * scale.rate_hz),
    };

    uint64_t utc_ns = 0;
    uint8_t utc_valid = 0;
    parse_proprietary_time_from_week_tow(gps_week, tow_sec, t_rx_ns, utc_ns, utc_valid);
    emit_imu_row(t_rx_ns, utc_ns, pc_last, true, acc, true, gyro, false, nullptr);
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
      process_rawimuxa_sentence(log, t_rx_ns, pc_last);
    }
  }

  void thread_main_serial() {
    const double bits_per_byte = 10.0;
    const uint64_t byte_time_ns = static_cast<uint64_t>(llround(1e9 * bits_per_byte / static_cast<double>(baud_)));

    int fd = ::open(dev_.c_str(), O_RDONLY | O_NOCTTY);
    if (fd < 0) {
      mark_error("IMU open failed: " + dev_ + " errno=" + std::to_string(errno));
      running_ = false;
      return;
    }
    if (!set_serial(fd, baud_)) {
      mark_error("failed to configure serial: " + dev_);
      ::close(fd);
      running_ = false;
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
    double a = 1.0;
    double b = 0.0;
    bool have_ab = false;
    uint16_t pc_last = 0;
    bool ascii_collect = false;
    std::string ascii_line;

    while (running_ && !g_stop.load()) {
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
          buf.push_back(tmp[i]);
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
        float acc[3]{0.0f, 0.0f, 0.0f};
        float gyro[3]{0.0f, 0.0f, 0.0f};
        double euler_deg[3]{0.0, 0.0, 0.0};
        double lat = 0.0, lon = 0.0, alt = 0.0;
        double vel[3]{0.0, 0.0, 0.0};
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
          } else if (data_id == 0x5022 && sz == 6) {
            alt = static_cast<double>(xsens_fp1632_i64(payload + ppos)) / 4294967296.0;
            got_alt = true;
          } else if (data_id == 0xD010 && sz == 12) {
            for (int i = 0; i < 3; ++i) vel[i] = static_cast<double>(be_f32(payload + ppos + 4 * i));
            got_vel = true;
          } else if (data_id == 0xD011 && sz == 24) {
            for (int i = 0; i < 3; ++i) vel[i] = be_f64(payload + ppos + 8 * i);
            got_vel = true;
          } else if (data_id == 0xD012 && sz == 18) {
            for (int i = 0; i < 3; ++i) vel[i] = static_cast<double>(xsens_fp1632_i64(payload + ppos + 6 * i)) / 4294967296.0;
            got_vel = true;
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
          }
        } else if (!utc_mapper_->map(t_master_ns, t_utc_ns)) {
          t_utc_ns = utc_mapper_->fallback_from_anchor(t_master_ns);
        }

        emit_imu_row(t_master_ns, t_utc_ns, pc_last, got_acc, acc, got_gyro, gyro, got_euler, euler_deg);
        if (got_latlon || got_alt || got_vel || got_utc) {
          emit_gnss_row(
              t_master_ns,
              t_utc_ns,
              got_latlon ? std::optional<double>(lat) : std::nullopt,
              got_latlon ? std::optional<double>(lon) : std::nullopt,
              got_alt ? std::optional<double>(alt) : std::nullopt,
              std::nullopt,
              std::nullopt,
              std::nullopt,
              std::nullopt,
              got_vel ? std::optional<double>(vel[0]) : std::nullopt,
              got_vel ? std::optional<double>(vel[1]) : std::nullopt,
              got_vel ? std::optional<double>(vel[2]) : std::nullopt,
              pc_last,
              utc_valid);
        }
      }
    }

    ::close(fd);
    running_ = false;
  }

  std::string dev_;
  fs::path dump_path_;
  int baud_;
  CsvSpoolWriter* imu_writer_;
  CsvSpoolWriter* gnss_writer_;
  UtcMapper* utc_mapper_;
  std::atomic<bool> running_{false};
  std::atomic<bool> capture_enabled_{false};
  std::atomic<bool> has_error_{false};
  std::atomic<bool> imu_seen_{false};
  std::atomic<bool> gnss_seen_{false};
  std::atomic<uint64_t> observed_imu_count_{0};
  std::atomic<uint64_t> observed_gnss_count_{0};
  std::atomic<uint64_t> last_imu_master_ns_{0};
  std::atomic<uint64_t> last_gnss_master_ns_{0};
  mutable std::mutex snapshot_mu_;
  ImuSnapshot last_imu_snapshot_;
  GnssSnapshot last_gnss_snapshot_;
  GnssSnapshot last_gnss_fix_snapshot_;
  std::thread thread_;
  uint64_t imu_row_id_{0};
  uint64_t gnss_row_id_{0};
  bool nmea_have_date_{false};
  uint16_t nmea_year_{0};
  uint8_t nmea_mon_{0};
  uint8_t nmea_day_{0};
  std::vector<DumpEvent> dump_events_;
};

struct ValidationResult {
  bool ok = true;
  std::string message;
};

static bool parse_camera_csv_counts(const fs::path& csv_path, std::map<uint32_t, uint64_t>& out_counts) {
  std::ifstream ifs(csv_path);
  if (!ifs) return false;
  std::string line;
  if (!std::getline(ifs, line)) return false;
  while (std::getline(ifs, line)) {
    if (line.empty()) continue;
    std::istringstream iss(line);
    std::string field;
    int idx = 0;
    uint32_t chunk_id = 0;
    while (std::getline(iss, field, ',')) {
      if (idx == 1) {
        if (field.empty()) return false;
        chunk_id = static_cast<uint32_t>(std::stoul(field));
        break;
      }
      ++idx;
    }
    out_counts[chunk_id]++;
  }
  return true;
}

static bool read_csv_rows(const fs::path& csv_path, std::string& header, std::vector<std::vector<std::string>>& rows) {
  std::ifstream ifs(csv_path);
  if (!ifs) return false;
  if (!std::getline(ifs, header)) return false;
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) continue;
    std::vector<std::string> fields;
    std::string field;
    std::istringstream iss(line);
    while (std::getline(iss, field, ',')) fields.push_back(field);
    rows.push_back(std::move(fields));
  }
  return true;
}

static bool write_csv_rows(const fs::path& csv_path, const std::string& header, const std::vector<std::vector<std::string>>& rows) {
  std::ofstream ofs(csv_path, std::ios::trunc);
  if (!ofs) return false;
  ofs << header << '\n';
  for (const auto& row : rows) {
    for (size_t i = 0; i < row.size(); ++i) {
      if (i) ofs << ',';
      ofs << row[i];
    }
    ofs << '\n';
  }
  return ofs.good();
}

struct CountFramesCtx {
  uint64_t count = 0;
};

static void validation_handoff(GstElement*, GstBuffer*, GstPad*, gpointer user_data) {
  auto* ctx = static_cast<CountFramesCtx*>(user_data);
  ctx->count++;
}

static bool count_frames_in_video(const fs::path& path,
                                  const std::string& container,
                                  const std::string& codec,
                                  uint64_t& frame_count,
                                  std::string& error_out) {
  std::string demuxer = (container == "mkv") ? "matroskademux" : "qtdemux";
  std::string parser = (codec == "h264") ? "h264parse" : "h265parse";
  std::ostringstream desc;
  desc << "filesrc location=\"" << path.string() << "\" ! " << demuxer << " ! " << parser << " ! nvv4l2decoder ! "
       << "fakesink name=vsink sync=false async=false signal-handoffs=true";
  GError* error = nullptr;
  GstElement* pipeline = gst_parse_launch(desc.str().c_str(), &error);
  if (!pipeline) {
    error_out = error ? error->message : "failed to parse validation pipeline";
    if (error) g_error_free(error);
    return false;
  }
  if (error) g_error_free(error);

  GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "vsink");
  CountFramesCtx ctx;
  g_signal_connect(sink, "handoff", G_CALLBACK(validation_handoff), &ctx);

  GstBus* bus = gst_element_get_bus(pipeline);
  gst_element_set_state(pipeline, GST_STATE_PLAYING);

  bool done = false;
  bool ok = true;
  while (!done) {
    GstMessage* msg = gst_bus_timed_pop_filtered(
        bus, GST_SECOND,
        static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_WARNING));
    if (!msg) continue;
    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_ERROR: {
        GError* err = nullptr;
        gchar* dbg = nullptr;
        gst_message_parse_error(msg, &err, &dbg);
        error_out = err ? err->message : "decode error";
        if (dbg) error_out += std::string(" | ") + dbg;
        if (err) g_error_free(err);
        if (dbg) g_free(dbg);
        ok = false;
        done = true;
        break;
      }
      case GST_MESSAGE_EOS:
        done = true;
        break;
      case GST_MESSAGE_WARNING:
        break;
      default:
        break;
    }
    gst_message_unref(msg);
  }

  gst_element_set_state(pipeline, GST_STATE_NULL);
  if (sink) gst_object_unref(sink);
  if (bus) gst_object_unref(bus);
  gst_object_unref(pipeline);
  frame_count = ctx.count;
  return ok;
}

static bool collect_decoded_chunk_counts(const CameraConfig& cam, const fs::path& session_dir,
                                         std::vector<uint64_t>& decoded_counts,
                                         std::string& report) {
  fs::path chunks_dir = session_dir / "sensors" / ("camera_" + cam.name) / "chunks";
  decoded_counts.clear();
  for (uint32_t chunk_id = 0;; ++chunk_id) {
    std::ostringstream filename;
    filename << "chunk_" << std::setw(4) << std::setfill('0') << chunk_id << "."
             << ((cam.container == "mkv") ? "mkv" : "mp4");
    fs::path chunk_path = chunks_dir / filename.str();
    if (!fs::exists(chunk_path)) break;
    uint64_t decoded = 0;
    std::string decode_error;
    if (!count_frames_in_video(chunk_path, cam.container, cam.codec, decoded, decode_error)) {
      report += "  " + chunk_path.filename().string() + " decode failed: " + decode_error + "\n";
      return false;
    }
    decoded_counts.push_back(decoded);
  }
  return !decoded_counts.empty();
}

static bool reconcile_camera_csv_chunking(const CameraConfig& cam, const fs::path& session_dir, std::string& report) {
  fs::path csv_path = session_dir / "sensors" / ("camera_" + cam.name) / "frames.csv";
  std::string header;
  std::vector<std::vector<std::string>> rows;
  if (!read_csv_rows(csv_path, header, rows)) {
    report += "camera_" + cam.name + ": failed to read frames.csv for reconcile\n";
    return false;
  }

  std::vector<uint64_t> decoded_counts;
  if (!collect_decoded_chunk_counts(cam, session_dir, decoded_counts, report)) {
    report += "camera_" + cam.name + ": failed to collect decoded chunk counts\n";
    return false;
  }

  uint64_t decoded_total = 0;
  for (uint64_t count : decoded_counts) decoded_total += count;
  if (decoded_total != rows.size()) {
    std::ostringstream os;
    os << "camera_" << cam.name << ": decoded_total=" << decoded_total
       << " rows=" << rows.size() << " (skip reconcile)\n";
    report += os.str();
    return false;
  }

  size_t row_index = 0;
  for (size_t chunk_id = 0; chunk_id < decoded_counts.size(); ++chunk_id) {
    for (uint64_t frame_index = 0; frame_index < decoded_counts[chunk_id]; ++frame_index) {
      if (row_index >= rows.size()) break;
      if (rows[row_index].size() < 3) rows[row_index].resize(3);
      rows[row_index][1] = std::to_string(chunk_id);
      rows[row_index][2] = std::to_string(frame_index);
      row_index++;
    }
  }

  if (!write_csv_rows(csv_path, header, rows)) {
    report += "camera_" + cam.name + ": failed to rewrite frames.csv\n";
    return false;
  }

  std::ostringstream os;
  os << "camera_" << cam.name << ": reconciled chunk metadata using decoded chunk sizes";
  for (size_t i = 0; i < decoded_counts.size(); ++i) {
    os << (i == 0 ? " [" : ", ") << decoded_counts[i];
  }
  os << "]\n";
  report += os.str();
  return true;
}

static void reconcile_session_chunking(const RecorderConfig& cfg, const fs::path& session_dir) {
  std::string report;
  for (const auto& cam : cfg.cameras) {
    reconcile_camera_csv_chunking(cam, session_dir, report);
  }
  if (!report.empty()) {
    std::cout << report;
  }
}

static ValidationResult validate_camera_session(const RecorderConfig& cfg, const fs::path& session_dir) {
  ValidationResult result;
  std::ostringstream report;
  bool overall_ok = true;
  for (const auto& cam : cfg.cameras) {
    fs::path cam_dir = session_dir / "sensors" / ("camera_" + cam.name);
    fs::path csv_path = cam_dir / "frames.csv";
    std::map<uint32_t, uint64_t> csv_counts;
    if (!parse_camera_csv_counts(csv_path, csv_counts)) {
      overall_ok = false;
      report << "camera_" << cam.name << ": failed to read frames.csv\n";
      continue;
    }

    report << "camera_" << cam.name << ":\n";
    std::set<uint32_t> seen_chunk_ids;
    for (const auto& [chunk_id, csv_count] : csv_counts) {
      std::ostringstream filename;
      filename << "chunk_" << std::setw(4) << std::setfill('0') << chunk_id << "."
               << ((cam.container == "mkv") ? "mkv" : "mp4");
      fs::path chunk_path = cam_dir / "chunks" / filename.str();
      uint64_t decoded = 0;
      std::string decode_error;
      bool ok = count_frames_in_video(chunk_path, cam.container, cam.codec, decoded, decode_error);
      bool counts_match = ok && decoded >= csv_count;
      overall_ok = overall_ok && counts_match;
      seen_chunk_ids.insert(chunk_id);
      report << "  " << chunk_path.filename().string()
             << " csv=" << csv_count
             << " decoded=" << decoded
             << (counts_match
                     ? (decoded == csv_count ? " OK" : " OK(overlap=" + std::to_string(decoded - csv_count) + ")")
                     : " MISMATCH");
      if (!decode_error.empty()) report << " (" << decode_error << ")";
      report << '\n';
    }

    for (uint32_t chunk_id = 0;; ++chunk_id) {
      if (seen_chunk_ids.count(chunk_id)) continue;
      std::ostringstream filename;
      filename << "chunk_" << std::setw(4) << std::setfill('0') << chunk_id << "."
               << ((cam.container == "mkv") ? "mkv" : "mp4");
      fs::path chunk_path = cam_dir / "chunks" / filename.str();
      if (!fs::exists(chunk_path)) {
        if (chunk_id > csv_counts.size()) break;
        continue;
      }
      uint64_t decoded = 0;
      std::string decode_error;
      bool ok = count_frames_in_video(chunk_path, cam.container, cam.codec, decoded, decode_error);
      if (!ok) {
        overall_ok = false;
        report << "  " << chunk_path.filename().string()
               << " decode failed";
        if (!decode_error.empty()) report << " (" << decode_error << ")";
        report << '\n';
      } else {
        report << "  " << chunk_path.filename().string()
               << " decoded=" << decoded
               << " ignored(no csv rows)\n";
      }
    }
  }
  result.ok = overall_ok;
  result.message = report.str();
  return result;
}

static RecorderConfig default_config() {
  RecorderConfig cfg;
  cfg.cameras = {
      {"front", "/dev/video2", 3840, 2160, 30, 80000000, 60, "mkv", "h265"},
      {"front_tele", "/dev/video7", 3840, 2160, 30, 80000000, 60, "mkv", "h265"},
      {"left", "/dev/video1", 3840, 2160, 30, 80000000, 60, "mkv", "h265"},
      {"right", "/dev/video6", 3840, 2160, 30, 80000000, 60, "mkv", "h265"},
  };
  sync_camera_common_settings(cfg);
  return cfg;
}

static void usage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0 << " [options]\n"
	      << "\n"
	      << "Config:\n"
	      << "  Auto-loads ./config.yaml, ./alpamayo_recorder.yaml, or /workspace/raw_recorder/config/raw_recorder.yaml\n"
      << "  --config PATH             load a specific config YAML before CLI overrides\n"
      << "  --no-config               ignore any auto-detected config file\n"
      << "\n"
      << "Recording options:\n"
      << "  --output-root PATH        default /workspace/datasample\n"
      << "  --session-id ID           explicit session id (used as session folder name)\n"
      << "  --duration-sec N          default 0 (run until Ctrl+C)\n"
      << "  --chunk-sec N             default 60\n"
      << "  --container TYPE          default mkv (mkv|mp4)\n"
      << "  --bitrate N               default 80000000\n"
      << "  --imu-dev PATH            default disabled\n"
      << "  --sensor-dump PATH        replay IMU/GNSS events from CSV dump\n"
      << "  --imu-baud N              default 115200\n"
      << "  --finalize                run alpamayo_spool_finalize.py after recording (default on)\n"
      << "  --no-finalize             skip parquet/sample-index conversion\n"
      << "  --cleanup-spool           remove CSV spool after finalize\n"
      << "  --validate                decode each chunk and compare against frames.csv\n"
      << "  --camera DEV1,DEV2,...    camera devices in order: front,front_tele,left,right\n"
      << "  --camera NAME=/dev/videoX override/add a named camera mapping (legacy form)\n"
      << "\n"
      << "Validation-only mode:\n"
      << "  --validate-session DIR    validate an existing session directory\n";
}

static std::string trim_copy(const std::string& value) {
  const size_t first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const size_t last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

static std::string normalize_camera_device(std::string device) {
  device = trim_copy(device);
  if (device.rfind("dev/video", 0) == 0) {
    device.insert(device.begin(), '/');
  }
  return device;
}

static std::vector<std::string> default_camera_names() {
  return {"front", "front_tele", "left", "right"};
}

static void sync_camera_common_settings(RecorderConfig& cfg) {
  for (auto& cam : cfg.cameras) {
    cam.width = cfg.camera_width;
    cam.height = cfg.camera_height;
    cam.fps = cfg.camera_fps;
    cam.bitrate = cfg.camera_bitrate;
    cam.chunk_sec = cfg.chunk_sec;
    cam.container = cfg.camera_container;
    cam.codec = cfg.camera_codec;
  }
}

static bool parse_camera_arg(const std::string& arg, CameraConfig& out) {
  size_t eq = arg.find('=');
  if (eq == std::string::npos || eq == 0 || eq + 1 >= arg.size()) return false;
  out = CameraConfig{};
  out.name = trim_copy(arg.substr(0, eq));
  out.device = normalize_camera_device(arg.substr(eq + 1));
  out.width = 3840;
  out.height = 2160;
  out.fps = 30;
  out.bitrate = 80000000;
  out.chunk_sec = 60;
  out.container = "mkv";
  out.codec = "h265";
  return !out.name.empty() && !out.device.empty();
}

static bool parse_camera_device_list_arg(const std::string& arg, std::vector<CameraConfig>& out) {
  const std::vector<std::string> names = default_camera_names();
  std::vector<std::string> devices;
  size_t start = 0;
  while (start <= arg.size()) {
    const size_t comma = arg.find(',', start);
    std::string token = normalize_camera_device(
        arg.substr(start, comma == std::string::npos ? std::string::npos : comma - start));
    if (token.empty()) return false;
    devices.push_back(token);
    if (comma == std::string::npos) break;
    start = comma + 1;
  }
  if (devices.empty() || devices.size() > names.size()) return false;
  out.clear();
  out.reserve(devices.size());
  for (size_t i = 0; i < devices.size(); ++i) {
    CameraConfig cam;
    cam.name = names[i];
    cam.device = devices[i];
    out.push_back(cam);
  }
  return true;
}

struct SimpleYamlConfig {
  std::unordered_map<std::string, std::string> scalars;
  std::unordered_map<std::string, std::vector<std::string>> lists;
};

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

static bool parse_yaml_int(const std::string& text, int& out) {
  const std::string value = trim_copy(text);
  if (value.empty()) return false;
  char* end = nullptr;
  long parsed = std::strtol(value.c_str(), &end, 10);
  if (!end || *end != '\0') return false;
  out = static_cast<int>(parsed);
  return true;
}

static std::vector<std::string> split_csv_items(const std::string& value) {
  std::vector<std::string> items;
  size_t start = 0;
  while (start <= value.size()) {
    const size_t comma = value.find(',', start);
    std::string token = yaml_unquote(
        value.substr(start, comma == std::string::npos ? std::string::npos : comma - start));
    token = trim_copy(token);
    if (!token.empty()) items.push_back(token);
    if (comma == std::string::npos) break;
    start = comma + 1;
  }
  return items;
}

static bool build_camera_configs_from_devices(const std::vector<std::string>& devices,
                                              const std::vector<std::string>& names,
                                              std::vector<CameraConfig>& out) {
  const std::vector<std::string> resolved_names = names.empty() ? default_camera_names() : names;
  if (devices.empty() || devices.size() > resolved_names.size()) return false;
  out.clear();
  out.reserve(devices.size());
  for (size_t i = 0; i < devices.size(); ++i) {
    CameraConfig cam;
    cam.name = trim_copy(resolved_names[i]);
    cam.device = normalize_camera_device(devices[i]);
    if (cam.name.empty() || cam.device.empty()) return false;
    out.push_back(cam);
  }
  return true;
}

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

static bool load_yaml_config(const fs::path& path, RecorderConfig& cfg, std::string& error) {
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

  if (auto value = get_scalar("output_root")) cfg.output_root = *value;
  if (auto value = get_scalar("session_id")) cfg.session_id = *value;
  if (auto value = get_scalar("vehicle_id")) cfg.vehicle_id = *value;
  if (auto value = get_scalar("timezone")) cfg.timezone = *value;
  if (auto value = get_scalar("imu_dev")) cfg.imu_dev = *value;
  if (auto value = get_scalar("sensor_dump")) cfg.sensor_dump = *value;
  if (auto value = get_scalar("container")) cfg.camera_container = to_lower_copy(*value);
  if (auto value = get_scalar("camera_container")) cfg.camera_container = to_lower_copy(*value);
  if (auto value = get_scalar("codec")) cfg.camera_codec = to_lower_copy(*value);
  if (auto value = get_scalar("camera_codec")) cfg.camera_codec = to_lower_copy(*value);

  if (cfg.camera_container != "mkv" && cfg.camera_container != "mp4") {
    error = "camera container must be mkv or mp4";
    return false;
  }
  if (cfg.camera_codec != "h264" && cfg.camera_codec != "h265") {
    error = "camera codec must be h264 or h265";
    return false;
  }

  if (!assign_int("imu_baud", cfg.imu_baud, 1)) return false;
  if (!assign_int("camera_fps", cfg.camera_fps, 1)) return false;
  if (!assign_int("camera_width", cfg.camera_width, 1)) return false;
  if (!assign_int("camera_height", cfg.camera_height, 1)) return false;
  if (!assign_int("bitrate", cfg.camera_bitrate, 1)) return false;
  if (!assign_int("camera_bitrate", cfg.camera_bitrate, 1)) return false;
  if (!assign_int("imu_hz", cfg.imu_hz, 1)) return false;
  if (!assign_int("gnss_hz", cfg.gnss_hz, 1)) return false;
  if (!assign_int("duration_sec", cfg.duration_sec, 0)) return false;
  if (!assign_int("chunk_sec", cfg.chunk_sec, 1)) return false;

  if (!assign_bool("finalize", cfg.finalize_spool)) return false;
  if (!assign_bool("finalize_spool", cfg.finalize_spool)) return false;
  if (!assign_bool("cleanup_spool", cfg.cleanup_spool)) return false;
  if (!assign_bool("validate", cfg.validate_session)) return false;

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

  if (!camera_names.empty() && camera_devices.size() != camera_names.size()) {
    error = "camera_names and camera_devices must have the same length";
    return false;
  }

  if (!camera_devices.empty()) {
    std::vector<CameraConfig> cameras;
    if (!build_camera_configs_from_devices(camera_devices, camera_names, cameras)) {
      error = "invalid camera_devices list";
      return false;
    }
    cfg.cameras = std::move(cameras);
  }

  sync_camera_common_settings(cfg);
  cfg.config_path = path;
  cfg.config_loaded = true;
  return true;
}

static std::vector<fs::path> default_config_search_paths() {
  std::vector<fs::path> paths;
  paths.push_back(fs::current_path() / "config.yaml");
  paths.push_back(fs::current_path() / "alpamayo_recorder.yaml");
  paths.push_back("/workspace/raw_recorder/config/raw_recorder.yaml");
  paths.push_back("/workspace/config.yaml");
  paths.push_back("/workspace/alpamayo_recorder.yaml");
  return paths;
}

int main(int argc, char** argv) {
  gst_init(&argc, &argv);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  RecorderConfig cfg = default_config();
  std::optional<fs::path> validate_only_dir;
  bool custom_cameras = false;
  std::optional<fs::path> requested_config_path;
  bool disable_config = false;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--help" || a == "-h") {
      usage(argv[0]);
      return 0;
    } else if (a == "--config" && i + 1 < argc) {
      requested_config_path = fs::path(argv[++i]);
    } else if (a == "--no-config") {
      disable_config = true;
    }
  }

  if (!disable_config) {
    fs::path config_path;
    bool have_config = false;
    if (requested_config_path) {
      config_path = *requested_config_path;
      have_config = true;
    } else {
      for (const auto& candidate : default_config_search_paths()) {
        if (fs::exists(candidate)) {
          config_path = candidate;
          have_config = true;
          break;
        }
      }
    }
    if (have_config) {
      std::string config_error;
      if (!load_yaml_config(config_path, cfg, config_error)) {
        log_error("failed to load config '" + config_path.string() + "': " + config_error);
        return 2;
      }
      log_info("loaded config: " + config_path.string());
    }
  }

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--config" && i + 1 < argc) {
      ++i;
      continue;
    } else if (a == "--no-config") {
      continue;
    } else if (a == "--output-root" && i + 1 < argc) {
      cfg.output_root = argv[++i];
    } else if (a == "--session-id" && i + 1 < argc) {
      cfg.session_id = argv[++i];
    } else if (a == "--duration-sec" && i + 1 < argc) {
      cfg.duration_sec = std::max(0, std::atoi(argv[++i]));
    } else if (a == "--chunk-sec" && i + 1 < argc) {
      cfg.chunk_sec = std::max(1, std::atoi(argv[++i]));
    } else if (a == "--container" && i + 1 < argc) {
      std::string container = argv[++i];
      if (container != "mkv" && container != "mp4") {
        std::cerr << "Invalid --container value: " << container << "\n";
        return 2;
      }
      cfg.camera_container = container;
    } else if (a == "--bitrate" && i + 1 < argc) {
      cfg.camera_bitrate = std::atoi(argv[++i]);
    } else if (a == "--imu-dev" && i + 1 < argc) {
      cfg.imu_dev = argv[++i];
    } else if (a == "--sensor-dump" && i + 1 < argc) {
      cfg.sensor_dump = argv[++i];
    } else if (a == "--imu-baud" && i + 1 < argc) {
      cfg.imu_baud = std::atoi(argv[++i]);
    } else if (a == "--finalize") {
      cfg.finalize_spool = true;
    } else if (a == "--no-finalize") {
      cfg.finalize_spool = false;
    } else if (a == "--cleanup-spool") {
      cfg.cleanup_spool = true;
    } else if (a == "--validate") {
      cfg.validate_session = true;
    } else if (a == "--validate-session" && i + 1 < argc) {
      validate_only_dir = fs::path(argv[++i]);
    } else if (a == "--camera" && i + 1 < argc) {
      std::string camera_arg = argv[++i];
      if (camera_arg.find('=') == std::string::npos) {
        while (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
          camera_arg += " ";
          camera_arg += argv[++i];
        }
      }
      if (!custom_cameras) {
        cfg.cameras.clear();
        custom_cameras = true;
      }
      if (camera_arg.find('=') != std::string::npos) {
        CameraConfig cam;
        if (!parse_camera_arg(camera_arg, cam)) {
          std::cerr << "Invalid --camera value\n";
          return 2;
        }
        cfg.cameras.push_back(cam);
      } else {
        std::vector<CameraConfig> parsed_cameras;
        if (!parse_camera_device_list_arg(camera_arg, parsed_cameras)) {
          std::cerr << "Invalid --camera value\n";
          return 2;
        }
        for (auto& cam : parsed_cameras) {
          cfg.cameras.push_back(cam);
        }
      }
    } else {
      std::cerr << "Unknown arg: " << a << "\n";
      usage(argv[0]);
      return 2;
    }
  }

  if (validate_only_dir) {
    ValidationResult vr = validate_camera_session(cfg, *validate_only_dir);
    std::cout << vr.message;
    return vr.ok ? 0 : 1;
  }

  if (cfg.cameras.empty()) {
    std::cerr << "No cameras configured\n";
    return 2;
  }

  if (cfg.session_id.empty()) {
    cfg.session_id = "session_" + ns_to_utc_string(now_realtime_ns()) + "_run_0001";
  }

  sync_camera_common_settings(cfg);

  fs::path session_dir = cfg.output_root / cfg.session_id;
  fs::create_directories(session_dir / "sensors");
  write_text_file(session_dir / "session_meta.json", make_session_meta_json(cfg));
  write_text_file(session_dir / "calib" / "camera_intrinsics.json", make_intrinsics_json(cfg));
  write_text_file(session_dir / "calib" / "sensor_mounts_placeholder.json", make_mounts_json(cfg));

  std::vector<std::unique_ptr<CsvSpoolWriter>> camera_writers;
  camera_writers.reserve(cfg.cameras.size());
  for (const auto& cam : cfg.cameras) {
    auto writer = std::make_unique<CsvSpoolWriter>();
    fs::path csv_path = session_dir / "sensors" / ("camera_" + cam.name) / "frames.csv";
    if (!writer->open(csv_path, camera_frames_header())) {
      log_error("failed to open " + csv_path.string());
      return 1;
    }
    camera_writers.push_back(std::move(writer));
  }

  TimeAnchors anchors;
  UtcMapper utc_mapper(anchors);
  CsvSpoolWriter imu_writer;
  CsvSpoolWriter gnss_writer;
  if (!imu_writer.open(session_dir / "sensors" / "imu" / "imu.csv", imu_header())) {
    log_error("failed to open IMU spool");
    return 1;
  }
  if (!gnss_writer.open(session_dir / "sensors" / "gnss_ins" / "gnss_ins.csv", gnss_header())) {
    log_error("failed to open GNSS spool");
    return 1;
  }
  SerialSensorRecorder serial_recorder(cfg.imu_dev, cfg.sensor_dump, cfg.imu_baud,
                                       &imu_writer, &gnss_writer, &utc_mapper);
  if (!serial_recorder.start()) {
    imu_writer.close();
    gnss_writer.close();
    for (auto& writer : camera_writers) writer->close();
    return 1;
  }

  std::vector<std::unique_ptr<CameraRecorder>> recorders;
  recorders.reserve(cfg.cameras.size());
  for (size_t i = 0; i < cfg.cameras.size(); ++i) {
    recorders.push_back(std::make_unique<CameraRecorder>(
        cfg.cameras[i], session_dir, camera_writers[i].get(), &utc_mapper, anchors));
    if (!recorders.back()->start()) {
      std::cerr << "Failed to start camera " << cfg.cameras[i].name << "\n";
      g_stop.store(true);
      break;
    }
  }
  if (g_stop.load()) {
    for (auto& recorder : recorders) recorder->force_stop();
    serial_recorder.stop();
    imu_writer.close();
    gnss_writer.close();
    for (auto& writer : camera_writers) writer->close();
    return 1;
  }

  log_info("waiting for camera warmup stabilization and IMU/GNSS readiness before recording");
  uint64_t wait_start_ns = now_master_ns();
  uint64_t last_wait_log_ns = 0;
  while (!g_stop.load()) {
    bool any_error = serial_recorder.has_error();
    bool cameras_ready = true;
    std::ostringstream cam_status;
    for (size_t i = 0; i < recorders.size(); ++i) {
      auto& recorder = recorders[i];
      recorder->poll_bus();
      any_error = any_error || recorder->has_error();
      cameras_ready = cameras_ready && recorder->ready();
      if (i) cam_status << " ";
      cam_status << recorder->status_string();
    }

    if (any_error) {
      g_stop.store(true);
      break;
    }

    const bool sensors_ready = serial_recorder.ready_for_capture();
    if (cameras_ready && sensors_ready) break;

    const uint64_t now_ns = now_master_ns();
    if (last_wait_log_ns == 0 || now_ns - last_wait_log_ns >= 2000000000ULL) {
      last_wait_log_ns = now_ns;
      if (!cameras_ready) log_warn("camera warmup in progress: " + cam_status.str());
      if (!sensors_ready) log_warn("waiting for IMU/GNSS values: " + serial_recorder.status_string());
      const double waited = static_cast<double>(now_ns - wait_start_ns) * 1e-9;
      log_info("capture gate still closed after " + std::to_string(static_cast<int>(std::floor(waited))) + "s");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (g_stop.load()) {
    for (auto& recorder : recorders) recorder->force_stop();
    serial_recorder.stop();
    imu_writer.close();
    gnss_writer.close();
    for (auto& writer : camera_writers) writer->close();
    return 1;
  }

  log_ready("all cameras stable and IMU/GNSS ready. ready to capture");
  serial_recorder.arm_capture();
  for (auto& recorder : recorders) recorder->arm_capture();
  log_info("capture boundary requested; waiting for first clean recorded chunk");

  uint64_t last_capture_wait_log_ns = 0;
  while (!g_stop.load()) {
    bool any_error = serial_recorder.has_error();
    bool capture_started = true;
    std::ostringstream cam_status;
    for (size_t i = 0; i < recorders.size(); ++i) {
      auto& recorder = recorders[i];
      recorder->poll_bus();
      any_error = any_error || recorder->has_error();
      capture_started = capture_started && recorder->capture_started();
      if (i) cam_status << " ";
      cam_status << recorder->config().name << "="
                 << (recorder->capture_started() ? "capturing" : "waiting-split");
    }
    if (any_error) {
      g_stop.store(true);
      break;
    }
    if (capture_started) break;

    const uint64_t now_ns = now_master_ns();
    if (last_capture_wait_log_ns == 0 || now_ns - last_capture_wait_log_ns >= 2000000000ULL) {
      last_capture_wait_log_ns = now_ns;
      log_warn("waiting for clean capture boundary: " + cam_status.str());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (g_stop.load()) {
    for (auto& recorder : recorders) recorder->force_stop();
    serial_recorder.stop();
    imu_writer.close();
    gnss_writer.close();
    for (auto& writer : camera_writers) writer->close();
    return 1;
  }

  log_ready("capture gate opened. recording started");

  std::cout << "Recording session: " << session_dir << "\n";
  for (const auto& cam : cfg.cameras) {
    std::cout << "  camera_" << cam.name << " -> " << cam.device
              << " " << cam.width << "x" << cam.height << "@" << cam.fps
              << " bitrate=" << cam.bitrate << " chunk=" << cam.chunk_sec << "s\n";
  }
  if (!cfg.sensor_dump.empty()) {
    std::cout << "  IMU/GNSS dump: " << cfg.sensor_dump << "\n";
  } else if (cfg.imu_dev.empty()) {
    std::cout << "  IMU/GNSS: disabled (no --imu-dev provided)\n";
  } else {
    std::cout << "  IMU/GNSS: " << cfg.imu_dev << " @ " << cfg.imu_baud << "\n";
  }

  const uint64_t start_ns = now_master_ns();
  uint64_t last_report_ns = start_ns;
  while (!g_stop.load()) {
    uint64_t now_ns = now_master_ns();
    if (cfg.duration_sec > 0 &&
        now_ns - start_ns >= static_cast<uint64_t>(cfg.duration_sec) * 1000000000ULL) {
      break;
    }
    for (auto& recorder : recorders) recorder->poll_bus();
    bool any_error = false;
    for (auto& recorder : recorders) any_error = any_error || recorder->has_error();
    if (any_error) {
      g_stop.store(true);
      break;
    }
    if (now_ns - last_report_ns >= 10000000000ULL) {
      last_report_ns = now_ns;
      double elapsed = static_cast<double>(now_ns - start_ns) * 1e-9;
      std::cout << std::fixed << std::setprecision(2)
                << "[STATS] elapsed=" << elapsed << "s";
      for (const auto& recorder : recorders) {
        double fps = recorder->frame_count() / std::max(0.001, elapsed);
        std::cout << " " << recorder->config().name << "=" << recorder->frame_count()
                  << "frames(" << fps << "fps,gaps=" << recorder->dropped_gaps() << ")";
      }
      std::cout << std::defaultfloat << "\n"
                << "  " << serial_recorder.latest_imu_log_string() << "\n"
                << "  " << serial_recorder.latest_gnss_log_string() << "\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  for (auto& recorder : recorders) recorder->request_stop();
  uint64_t eos_deadline = now_master_ns() + 15000000000ULL;
  while (now_master_ns() < eos_deadline) {
    bool all_done = true;
    for (auto& recorder : recorders) {
      recorder->poll_bus();
      all_done = all_done && (recorder->eos_received() || recorder->has_error());
    }
    if (all_done) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  for (auto& recorder : recorders) recorder->force_stop();

  serial_recorder.stop();
  imu_writer.close();
  gnss_writer.close();
  for (auto& writer : camera_writers) writer->close();

  std::cout << "Session complete: " << session_dir << "\n";
  for (const auto& recorder : recorders) {
    double fps = 0.0;
    if (recorder->first_master_ns() && recorder->last_master_ns() &&
        *recorder->last_master_ns() > *recorder->first_master_ns()) {
      double span = static_cast<double>(*recorder->last_master_ns() - *recorder->first_master_ns()) * 1e-9;
      fps = recorder->frame_count() / span;
    }
    std::cout << "  camera_" << recorder->config().name
              << ": frames=" << recorder->frame_count()
              << " fps~=" << std::fixed << std::setprecision(2) << fps
              << " gaps=" << recorder->dropped_gaps() << std::defaultfloat << "\n";
  }

  reconcile_session_chunking(cfg, session_dir);

  if (cfg.validate_session) {
    ValidationResult vr = validate_camera_session(cfg, session_dir);
    std::cout << vr.message;
    if (!vr.ok) {
      std::cerr << "Validation failed\n";
      return 1;
    }
  }

	  if (cfg.finalize_spool) {
	    fs::path finalize_python = "/workspace/.venv-alpamayo/bin/python";
	    if (!fs::exists(finalize_python)) {
	      finalize_python = "python3";
	    }
	    fs::path finalize_script = "/workspace/raw_recorder/tools/alpamayo_spool_finalize.py";
	    if (!fs::exists(finalize_script)) {
	      finalize_script = "/workspace/alpamayo_spool_finalize.py";
	    }
	    std::ostringstream cmd;
	    cmd << shell_quote(finalize_python.string())
	        << " " << shell_quote(finalize_script.string())
	        << " --session-dir " << shell_quote(session_dir.string());
	    if (cfg.cleanup_spool) cmd << " --cleanup-spool";
    int rc = std::system(cmd.str().c_str());
    if (rc != 0) {
      std::cerr << "Finalize failed with rc=" << rc << "\n";
      return 1;
    }
  }

  return 0;
}
