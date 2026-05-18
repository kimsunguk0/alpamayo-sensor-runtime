// sync4cam_xsens.cpp
// Build:
//   g++ -O2 -std=c++17 -pthread sync4cam_xsens.cpp -o sync4cam_xsens
//
// Run (prod):
//   sudo ./sync4cam_xsens --log prod /dev/video0 /dev/video3 /dev/video5 /dev/video6
//
// Run (debug, all debug toggles ON by default if none specified):
//   sudo ./sync4cam_xsens --log debug /dev/video0 /dev/video3 /dev/video5 /dev/video6
//
// Debug toggles (optional):
//   --dbg-drift --dbg-v4l2 --dbg-queue --dbg-imu-fit
//
// NOTE (per your request):
// - PROD output does NOT include MASTER_SINCE_START_S or WALL~=
// - PROD output includes (1) both master_ns[4] and aligned_ns[4] + seq[4]
// - PROD output prints IMU0/IMU1/IMU2 (3 samples), like your original style

#include <linux/videodev2.h>

#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <poll.h>
#include <signal.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

static std::atomic<bool> g_stop{false};
static void on_sigint(int) { g_stop.store(true); }
static std::atomic<bool> g_dbg_mtdata{false};

static int xioctl(int fd, unsigned long req, void* arg) {
  int r;
  do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
  return r;
}

// ---------------- Host time (MASTER CLOCK) ----------------
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

static std::string ns_to_wall_string(uint64_t rt_ns) {
  time_t sec = (time_t)(rt_ns / 1000000000ULL);
  struct tm tmv{};
  localtime_r(&sec, &tmv);
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
                tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
  return std::string(buf);
}

// ---------------- Serial helpers ----------------
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

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;  // 0.1s
  return (tcsetattr(fd, TCSANOW, &tty) == 0);
}

// ---------------- Xsens helpers ----------------
static inline uint16_t be_u16(const uint8_t* p) { return (uint16_t(p[0]) << 8) | uint16_t(p[1]); }
static inline uint32_t be_u32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}
static inline uint64_t be_u64(const uint8_t* p) {
  return (uint64_t(p[0]) << 56) | (uint64_t(p[1]) << 48) | (uint64_t(p[2]) << 40) | (uint64_t(p[3]) << 32) |
         (uint64_t(p[4]) << 24) | (uint64_t(p[5]) << 16) | (uint64_t(p[6]) << 8) | uint64_t(p[7]);
}
static inline int32_t be_i32(const uint8_t* p) { return (int32_t)be_u32(p); }
static inline int64_t be_i64(const uint8_t* p) { return (int64_t)be_u64(p); }
static inline int64_t be_i48(const uint8_t* p) {
  uint64_t u = (uint64_t(p[0]) << 40) | (uint64_t(p[1]) << 32) |
               (uint64_t(p[2]) << 24) | (uint64_t(p[3]) << 16) |
               (uint64_t(p[4]) << 8)  |  uint64_t(p[5]);
  if (u & (1ULL << 47)) u |= 0xFFFF000000000000ULL;
  return (int64_t)u;
}
static inline int64_t xsens_fp1632_i64(const uint8_t* p) {
  // Xsens 16.32 fixed point transmits only 6 bytes as:
  // [frac31:24, frac23:16, frac15:8, frac7:0, int15:8, int7:0]
  uint64_t u = (uint64_t(p[4]) << 40) | (uint64_t(p[5]) << 32) |
               (uint64_t(p[0]) << 24) | (uint64_t(p[1]) << 16) |
               (uint64_t(p[2]) << 8)  |  uint64_t(p[3]);
  if (u & (1ULL << 47)) u |= 0xFFFF000000000000ULL;
  return (int64_t)u;
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
  // Xsens: sum(all bytes excluding preamble) & 0xFF == 0 (checksum included)
  uint32_t sum = 0;
  for (size_t i = 1; i < frame_len; i++) sum += buf[i];
  return (sum & 0xFF) == 0;
}

// Sliding window linear regression: y = a*x + b
struct LinFitWindow {
  struct XY { double x; double y; };
  std::deque<XY> q;
  double sum_x=0, sum_y=0, sum_xx=0, sum_xy=0;
  double window_sec = 10.0;

  void clear() { q.clear(); sum_x=sum_y=sum_xx=sum_xy=0; }

  void push(double x, double y) {
    q.push_back({x, y});
    sum_x += x; sum_y += y; sum_xx += x*x; sum_xy += x*y;

    while (!q.empty() && (x - q.front().x) > window_sec) {
      auto o = q.front(); q.pop_front();
      sum_x -= o.x; sum_y -= o.y; sum_xx -= o.x*o.x; sum_xy -= o.x*o.y;
    }
  }

  bool estimate(double& a, double& b) const {
    size_t n = q.size();
    if (n < 30) return false;
    double denom = (double)n * sum_xx - sum_x * sum_x;
    if (std::abs(denom) < 1e-12) return false;
    a = ((double)n * sum_xy - sum_x * sum_y) / denom;
    b = (sum_y - a * sum_x) / (double)n;
    return true;
  }

  size_t size() const { return q.size(); }
};

// ---------------- IMU stats for debug ----------------
struct ImuSyncStats {
  std::mutex m;
  bool have_ab = false;
  double a = 1.0;
  double b = 0.0;
  size_t win_n = 0;
  uint64_t last_rx_ns = 0;
};
static ImuSyncStats g_imu_stats;

struct UtcSyncStats {
  std::mutex m;
  bool have_origin = false;
  bool have_map = false;
  double x0 = 0.0;
  double y0 = 0.0;
  double a = 1.0;
  double b = 0.0;
  size_t win_n = 0;
  uint64_t last_master_ns = 0;
  uint64_t last_utc_ns = 0;
};
static UtcSyncStats g_utc_stats;

static bool map_master_to_utc_from_stats(uint64_t master_ns, uint64_t& utc_ns) {
  std::lock_guard<std::mutex> lk(g_utc_stats.m);
  if (!g_utc_stats.have_origin) return false;

  if (!g_utc_stats.have_map) {
    if (g_utc_stats.last_master_ns == 0 || g_utc_stats.last_utc_ns == 0) return false;
    int64_t off_ns = (int64_t)g_utc_stats.last_utc_ns - (int64_t)g_utc_stats.last_master_ns;
    utc_ns = (uint64_t)((int64_t)master_ns + off_ns);
    return true;
  }

  double x_rel = (double)master_ns * 1e-9 - g_utc_stats.x0;
  double y_rel = g_utc_stats.a * x_rel + g_utc_stats.b;
  utc_ns = (uint64_t) llround((g_utc_stats.y0 + y_rel) * 1e9);
  return true;
}

// ---------------- IMU sample + ring buffer ----------------
struct ImuSample {
  uint64_t t_master_ns = 0;   // mapped to MASTER CLOCK domain
  uint64_t t_utc_ns = 0;      // mapped to GNSS UTC epoch when available
  uint16_t pc = 0;
  float acc[3]{0,0,0};
  float gyro[3]{0,0,0};
  bool have_acc=false;
  bool have_gyro=false;
};

struct GnssSample {
  uint64_t t_master_ns = 0;
  uint64_t t_utc_ns = 0;
  uint16_t pc = 0;
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double alt_m = 0.0;
  double vel_mps[3]{0,0,0};
  uint8_t utc_valid = 0;
  bool have_pos = false;
  bool have_alt = false;
  bool have_vel = false;
  bool have_utc = false;
};

class ImuRing {
public:
  explicit ImuRing(size_t cap) : cap_(cap), buf_(cap) {}

  void push(const ImuSample& s) {
    std::lock_guard<std::mutex> lk(m_);
    buf_[head_] = s;
    head_ = (head_ + 1) % cap_;
    if (count_ < cap_) count_++;
  }

  bool get_last_n_before(uint64_t t_ref, int n, std::vector<ImuSample>& out) {
    out.clear();
    out.reserve(n);

    std::lock_guard<std::mutex> lk(m_);
    if (count_ == 0) return false;

    size_t idx = (head_ + cap_ - 1) % cap_;
    for (size_t k = 0; k < count_; ++k) {
      const ImuSample& s = buf_[idx];
      if (s.t_master_ns != 0 && s.t_master_ns <= t_ref) {
        out.push_back(s); // newest->older
        if ((int)out.size() == n) break;
      }
      idx = (idx + cap_ - 1) % cap_;
    }

    if ((int)out.size() < n) return false;
    std::reverse(out.begin(), out.end()); // oldest->newest
    return true;
  }

private:
  size_t cap_;
  std::vector<ImuSample> buf_;
  size_t head_{0};
  size_t count_{0};
  std::mutex m_;
};

class GnssRing {
public:
  explicit GnssRing(size_t cap) : cap_(cap), buf_(cap) {}

  void push(const GnssSample& s) {
    std::lock_guard<std::mutex> lk(m_);
    buf_[head_] = s;
    head_ = (head_ + 1) % cap_;
    if (count_ < cap_) count_++;
  }

  bool get_last_before(uint64_t t_ref, GnssSample& out) {
    std::lock_guard<std::mutex> lk(m_);
    if (count_ == 0) return false;

    out = GnssSample{};
    size_t idx = (head_ + cap_ - 1) % cap_;
    bool have_any = false;
    for (size_t k = 0; k < count_; ++k) {
      const GnssSample& s = buf_[idx];
      if (s.t_master_ns != 0 && s.t_master_ns <= t_ref &&
          (s.have_pos || s.have_alt || s.have_vel || s.have_utc)) {
        if (!have_any) {
          out = s;
          have_any = true;
        } else {
          if (!out.have_pos && s.have_pos) {
            out.lat_deg = s.lat_deg;
            out.lon_deg = s.lon_deg;
            out.have_pos = true;
          }
          if (!out.have_alt && s.have_alt) {
            out.alt_m = s.alt_m;
            out.have_alt = true;
          }
          if (!out.have_vel && s.have_vel) {
            for (int i = 0; i < 3; ++i) out.vel_mps[i] = s.vel_mps[i];
            out.have_vel = true;
          }
          if (!out.have_utc && s.have_utc) {
            out.t_utc_ns = s.t_utc_ns;
            out.utc_valid = s.utc_valid;
            out.have_utc = true;
          }
        }
        if (out.have_pos && out.have_alt && out.have_vel && out.have_utc) return true;
      }
      idx = (idx + cap_ - 1) % cap_;
    }
    return have_any;
  }

private:
  size_t cap_;
  std::vector<GnssSample> buf_;
  size_t head_{0};
  size_t count_{0};
  std::mutex m_;
};

struct TimeMapWindow {
  LinFitWindow fit;
  double x0 = 0.0;
  double y0 = 0.0;
  bool have_origin = false;
  bool have_fit = false;
  double a = 1.0;
  double b = 0.0;
  uint64_t last_master_ns = 0;
  uint64_t last_utc_ns = 0;

  void reset(double window_sec) {
    fit.window_sec = window_sec;
    fit.clear();
    x0 = 0.0;
    y0 = 0.0;
    have_origin = false;
    have_fit = false;
    a = 1.0;
    b = 0.0;
    last_master_ns = 0;
    last_utc_ns = 0;
  }

  void push(uint64_t master_ns, uint64_t utc_ns) {
    double x = (double)master_ns * 1e-9;
    double y = (double)utc_ns * 1e-9;
    if (!have_origin) {
      x0 = x;
      y0 = y;
      have_origin = true;
    }

    fit.push(x - x0, y - y0);
    last_master_ns = master_ns;
    last_utc_ns = utc_ns;

    double aa = 0.0, bb = 0.0;
    if (fit.estimate(aa, bb)) {
      a = aa;
      b = bb;
      have_fit = true;
    }
  }

  bool map(uint64_t master_ns, uint64_t& utc_ns) const {
    if (!have_origin) return false;

    if (!have_fit) {
      int64_t off_ns = (int64_t)last_utc_ns - (int64_t)last_master_ns;
      utc_ns = (uint64_t)((int64_t)master_ns + off_ns);
      return true;
    }

    double x_rel = (double)master_ns * 1e-9 - x0;
    double y_rel = a * x_rel + b;
    utc_ns = (uint64_t) llround((y0 + y_rel) * 1e9);
    return true;
  }
};

static bool xsens_utc_to_ns(uint16_t year, uint8_t mon, uint8_t day,
                            uint8_t hour, uint8_t min, uint8_t sec,
                            uint32_t nanos, uint64_t& out_ns) {
  if (year < 1970 || mon < 1 || mon > 12 || day < 1 || day > 31 ||
      hour > 23 || min > 59 || sec > 60 || nanos >= 1000000000U) {
    return false;
  }

  struct tm tmv{};
  tmv.tm_year = (int)year - 1900;
  tmv.tm_mon = (int)mon - 1;
  tmv.tm_mday = (int)day;
  tmv.tm_hour = (int)hour;
  tmv.tm_min = (int)min;
  tmv.tm_sec = (sec == 60) ? 59 : (int)sec;

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
  for (size_t i = 1; i < star; ++i) chk ^= (unsigned char)line[i];
  char* end = nullptr;
  unsigned want = std::strtoul(line.c_str() + star + 1, &end, 16);
  return end == line.c_str() + star + 3 && chk == want;
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

static bool parse_nmea_hms(const std::string& s, uint8_t& hour, uint8_t& min,
                           uint8_t& sec, uint32_t& nanos) {
  if (s.size() < 6) return false;
  hour = (uint8_t)std::atoi(s.substr(0, 2).c_str());
  min  = (uint8_t)std::atoi(s.substr(2, 2).c_str());
  double sec_f = std::strtod(s.c_str() + 4, nullptr);
  if (hour > 23 || min > 59 || sec_f < 0.0 || sec_f >= 61.0) return false;
  sec = (uint8_t)std::floor(sec_f);
  nanos = (uint32_t) llround((sec_f - (double)sec) * 1e9);
  if (nanos >= 1000000000U) { nanos -= 1000000000U; sec = (uint8_t)(sec + 1); }
  return true;
}

static bool parse_nmea_date_ddmmyy(const std::string& s, uint16_t& year,
                                   uint8_t& mon, uint8_t& day) {
  if (s.size() != 6) return false;
  day = (uint8_t)std::atoi(s.substr(0, 2).c_str());
  mon = (uint8_t)std::atoi(s.substr(2, 2).c_str());
  int yy = std::atoi(s.substr(4, 2).c_str());
  year = (uint16_t)((yy >= 80) ? (1900 + yy) : (2000 + yy));
  return true;
}

static bool parse_nmea_latlon(const std::string& value, const std::string& hemi,
                              bool is_lat, double& deg_out) {
  int deg_digits = is_lat ? 2 : 3;
  if (value.size() < (size_t)(deg_digits + 3) || hemi.empty()) return false;
  double raw = std::strtod(value.c_str(), nullptr);
  double scale = is_lat ? 100.0 : 100.0;
  int deg = (int)(raw / scale);
  double minutes = raw - (double)deg * scale;
  deg_out = (double)deg + minutes / 60.0;
  char h = hemi[0];
  if (h == 'S' || h == 'W') deg_out = -deg_out;
  return h == 'N' || h == 'S' || h == 'E' || h == 'W';
}

// ---------------- IMU thread ----------------
struct ImuConfig {
  std::string dev = "/dev/ttyUSB0";
  int baud = 115200;
  double win_sec = 10.0;
};

static void imu_thread_main(const ImuConfig cfg, ImuRing* ring, GnssRing* gnss_ring) {
  const double bits_per_byte = 10.0;
  const uint64_t byte_time_ns = (uint64_t) llround(1e9 * bits_per_byte / (double)cfg.baud);

  int fd = ::open(cfg.dev.c_str(), O_RDONLY | O_NOCTTY);
  if (fd < 0) { std::perror("IMU open"); return; }
  if (!set_serial(fd, cfg.baud)) {
    std::cerr << "[IMU] Failed to set serial\n";
    ::close(fd);
    return;
  }
  tcflush(fd, TCIFLUSH);

  std::deque<uint8_t>  buf;
  std::deque<uint64_t> tbuf;
  std::vector<uint8_t> tmp(4096);

  bool have_stf = false;
  uint32_t last_stf32 = 0;
  uint64_t stf_ext = 0;

  LinFitWindow win;
  win.window_sec = cfg.win_sec;
  double a = 1.0, b = 0.0;
  bool have_ab = false;
  TimeMapWindow utc_map;
  utc_map.reset(cfg.win_sec);

  uint16_t pc_last = 0;
  bool nmea_collect = false;
  std::string nmea_line;
  uint16_t nmea_year = 0;
  uint8_t nmea_mon = 0, nmea_day = 0;
  bool have_nmea_date = false;

  auto update_utc_sync_stats = [&](uint64_t master_ns, uint64_t utc_ns) {
    utc_map.push(master_ns, utc_ns);
    std::lock_guard<std::mutex> lk(g_utc_stats.m);
    g_utc_stats.have_origin = utc_map.have_origin;
    g_utc_stats.have_map = utc_map.have_fit || utc_map.have_origin;
    g_utc_stats.x0 = utc_map.x0;
    g_utc_stats.y0 = utc_map.y0;
    g_utc_stats.a = utc_map.a;
    g_utc_stats.b = utc_map.b;
    g_utc_stats.win_n = utc_map.fit.size();
    g_utc_stats.last_master_ns = master_ns;
    g_utc_stats.last_utc_ns = utc_ns;
  };

  auto process_nmea_sentence = [&](const std::string& line, uint64_t t_rx_ns) {
    if (!nmea_checksum_ok(line)) return;
    size_t star = line.find('*');
    std::string body = line.substr(1, star - 1);
    std::vector<std::string> f = split_csv(body);
    if (f.empty()) return;

    if (f[0].size() >= 5 && f[0].substr(f[0].size() - 3) == "RMC") {
      if (f.size() < 10 || f[2] != "A") return;
      double lat_deg = 0.0, lon_deg = 0.0;
      if (!parse_nmea_latlon(f[3], f[4], true, lat_deg)) return;
      if (!parse_nmea_latlon(f[5], f[6], false, lon_deg)) return;

      uint8_t hour = 0, min = 0, sec = 0;
      uint32_t nanos = 0;
      uint16_t year = 0;
      uint8_t mon = 0, day = 0;
      if (!parse_nmea_hms(f[1], hour, min, sec, nanos)) return;
      if (!parse_nmea_date_ddmmyy(f[9], year, mon, day)) return;
      uint64_t utc_ns = 0;
      if (!xsens_utc_to_ns(year, mon, day, hour, min, sec, nanos, utc_ns)) return;

      have_nmea_date = true;
      nmea_year = year;
      nmea_mon = mon;
      nmea_day = day;

      GnssSample gs;
      gs.t_master_ns = t_rx_ns;
      gs.t_utc_ns = utc_ns;
      gs.pc = pc_last;
      gs.have_utc = true;
      gs.utc_valid = 0x03;
      gs.lat_deg = lat_deg;
      gs.lon_deg = lon_deg;
      gs.have_pos = true;
      if (f.size() > 7 && !f[7].empty()) {
        gs.vel_mps[0] = std::strtod(f[7].c_str(), nullptr) * 0.514444;
        gs.have_vel = true;
      }
      gnss_ring->push(gs);
      update_utc_sync_stats(t_rx_ns, utc_ns);
      return;
    }

    if (f[0].size() >= 5 && f[0].substr(f[0].size() - 3) == "GGA") {
      if (f.size() < 10) return;
      int fix_quality = f[6].empty() ? 0 : std::atoi(f[6].c_str());
      if (fix_quality <= 0) return;
      double lat_deg = 0.0, lon_deg = 0.0;
      if (!parse_nmea_latlon(f[2], f[3], true, lat_deg)) return;
      if (!parse_nmea_latlon(f[4], f[5], false, lon_deg)) return;

      GnssSample gs;
      gs.t_master_ns = t_rx_ns;
      gs.pc = pc_last;
      gs.lat_deg = lat_deg;
      gs.lon_deg = lon_deg;
      gs.have_pos = true;
      if (!f[9].empty()) {
        gs.alt_m = std::strtod(f[9].c_str(), nullptr);
        gs.have_alt = true;
      }

      if (have_nmea_date) {
        uint8_t hour = 0, min = 0, sec = 0;
        uint32_t nanos = 0;
        uint64_t utc_ns = 0;
        if (parse_nmea_hms(f[1], hour, min, sec, nanos) &&
            xsens_utc_to_ns(nmea_year, nmea_mon, nmea_day, hour, min, sec, nanos, utc_ns)) {
          gs.t_utc_ns = utc_ns;
          gs.have_utc = true;
          gs.utc_valid = 0x03;
          update_utc_sync_stats(t_rx_ns, utc_ns);
        }
      }

      gnss_ring->push(gs);
    }
  };

  while (!g_stop.load()) {
    int n = ::read(fd, tmp.data(), (int)tmp.size());
    if (n > 0) {
      uint64_t t_read_ns = now_master_ns();
      for (int i = 0; i < n; i++) {
        uint64_t t_i = t_read_ns - (uint64_t)((n - 1 - i) * byte_time_ns);
        uint8_t ch = tmp[i];
        if (!nmea_collect) {
          if (ch == '$') {
            nmea_collect = true;
            nmea_line.clear();
            nmea_line.push_back('$');
          }
        } else {
          if (ch == '$') {
            nmea_line.clear();
            nmea_line.push_back('$');
          } else if (ch == '\n') {
            process_nmea_sentence(nmea_line, t_i);
            nmea_collect = false;
            nmea_line.clear();
          } else if (ch != '\r') {
            if (nmea_line.size() < 160) {
              nmea_line.push_back((char)ch);
            } else {
              nmea_collect = false;
              nmea_line.clear();
            }
          }
        }
        buf.push_back(tmp[i]);
        tbuf.push_back(t_i);
      }
    } else {
      usleep(1000);
    }

    if (buf.size() > (1u << 20)) { buf.clear(); tbuf.clear(); }

    while (true) {
      while (!buf.empty() && buf.front() != 0xFA) { buf.pop_front(); tbuf.pop_front(); }
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
        buf.pop_front(); tbuf.pop_front();
        continue;
      }

      uint64_t t_rx_ns = tbuf[frame_len - 1];

      static std::vector<uint8_t> frame;
      frame.resize(frame_len);
      for (size_t i = 0; i < frame_len; i++) frame[i] = buf[i];
      for (size_t i = 0; i < frame_len; i++) { buf.pop_front(); tbuf.pop_front(); }

      if (mid != 0x36) continue; // MTData2 only

      const uint8_t* payload = frame.data() + hdr;
      size_t ppos = 0;

      bool got_stf=false, got_pc=false, got_a=false, got_g=false;
      bool got_latlon=false, got_alt=false, got_vel=false, got_utc=false;
      uint32_t stf32 = 0;
      uint16_t pc = 0;
      float acc[3]{}, gyro[3]{};
      double lat_deg=0.0, lon_deg=0.0, alt_m=0.0;
      double vel_mps[3]{0,0,0};
      uint16_t utc_year = 0;
      uint8_t utc_mon = 0, utc_day = 0, utc_hour = 0, utc_min = 0, utc_sec = 0, utc_valid = 0;
      uint32_t utc_nanos = 0;
      std::ostringstream mtdata_dump;
      bool need_mtdata_dump = g_dbg_mtdata.load();
      bool mtdata_first = true;

      while (ppos + 3 <= data_len) {
        uint16_t dataId = be_u16(payload + ppos);
        uint8_t  sz     = payload[ppos + 2];
        ppos += 3;
        if (ppos + sz > data_len) break;

        uint16_t id_mask = dataId & 0xFFF0;
        if (need_mtdata_dump) {
          if (!mtdata_first) mtdata_dump << " ";
          mtdata_dump << "0x" << std::hex << std::setw(4) << std::setfill('0') << dataId
                      << std::dec << "/" << (unsigned)sz;
          mtdata_first = false;
        }

        if (dataId == 0x1060 && sz == 4) {           // SampleTimeFine
          stf32 = be_u32(payload + ppos);
          got_stf = true;
        } else if (dataId == 0x1020 && sz == 2) {    // PacketCounter
          pc = be_u16(payload + ppos);
          got_pc = true;
        } else if (id_mask == 0x4020 && sz == 12) {  // Acc (float32)
          for (int i = 0; i < 3; i++) acc[i] = be_f32(payload + ppos + 4*i);
          got_a = true;
        } else if (id_mask == 0x8020 && sz == 12) {  // Gyro (float32)
          for (int i = 0; i < 3; i++) gyro[i] = be_f32(payload + ppos + 4*i);
          got_g = true;
        } else if (dataId == 0x1010 && sz >= 12) {   // UTC Time
          utc_nanos = be_u32(payload + ppos);
          utc_year  = be_u16(payload + ppos + 4);
          utc_mon   = payload[ppos + 6];
          utc_day   = payload[ppos + 7];
          utc_hour  = payload[ppos + 8];
          utc_min   = payload[ppos + 9];
          utc_sec   = payload[ppos + 10];
          utc_valid = payload[ppos + 11];
          got_utc = true;
        } else if (dataId == 0x5040 && sz == 16) {   // LatLon double
          lat_deg = be_f64(payload + ppos);
          lon_deg = be_f64(payload + ppos + 8);
          got_latlon = true;
        } else if (dataId == 0x5041 && sz == 8) {    // LatLon fixed 12.20
          lat_deg = (double)be_i32(payload + ppos) / 1048576.0;
          lon_deg = (double)be_i32(payload + ppos + 4) / 1048576.0;
          got_latlon = true;
        } else if (dataId == 0x5042 && sz == 12) {   // LatLon fixed 16.32 (6 bytes each)
          lat_deg = (double)xsens_fp1632_i64(payload + ppos) / 4294967296.0;
          lon_deg = (double)xsens_fp1632_i64(payload + ppos + 6) / 4294967296.0;
          got_latlon = true;
        } else if (dataId == 0x5020 && sz == 8) {    // Altitude double
          alt_m = be_f64(payload + ppos);
          got_alt = true;
        } else if (dataId == 0x5021 && sz == 4) {    // Altitude float
          alt_m = (double)be_f32(payload + ppos);
          got_alt = true;
        } else if (dataId == 0x5022 && sz == 4) {    // Altitude fixed 12.20
          alt_m = (double)be_i32(payload + ppos) / 1048576.0;
          got_alt = true;
        } else if (dataId == 0x5022 && sz == 6) {    // Altitude fixed 16.32 (6 bytes)
          alt_m = (double)xsens_fp1632_i64(payload + ppos) / 4294967296.0;
          got_alt = true;
        } else if (dataId == 0xD010 && sz == 12) {   // Velocity float32
          for (int i = 0; i < 3; i++) vel_mps[i] = (double)be_f32(payload + ppos + 4*i);
          got_vel = true;
        } else if (dataId == 0xD011 && sz == 24) {   // Velocity double
          for (int i = 0; i < 3; i++) vel_mps[i] = be_f64(payload + ppos + 8*i);
          got_vel = true;
        } else if (dataId == 0xD012 && sz == 18) {   // Velocity fixed 16.32 (6 bytes each)
          for (int i = 0; i < 3; i++) vel_mps[i] = (double)xsens_fp1632_i64(payload + ppos + 6*i) / 4294967296.0;
          got_vel = true;
        }

        ppos += sz;
      }

      if (need_mtdata_dump) {
        std::cout << "[MTDATA] ids=" << mtdata_dump.str()
                  << " got_stf=" << got_stf
                  << " got_pc=" << got_pc
                  << " got_acc=" << got_a
                  << " got_gyro=" << got_g
                  << " got_utc=" << got_utc
                  << " got_latlon=" << got_latlon
                  << " got_alt=" << got_alt
                  << " got_vel=" << got_vel
                  << "\n";
      }

      if (!got_stf) continue;

      if (!have_stf) {
        have_stf = true;
        last_stf32 = stf32;
        stf_ext = stf32;
      } else {
        uint32_t delta = stf32 - last_stf32;
        stf_ext += (uint64_t)delta;
        last_stf32 = stf32;
      }

      if (got_pc) pc_last = pc;

      double t_sensor = (double)stf_ext / 10000.0;
      double t_host   = (double)t_rx_ns * 1e-9;

      win.push(t_sensor, t_host);
      double aa=0.0, bb=0.0;
      if (win.estimate(aa, bb)) {
        a = aa; b = bb; have_ab = true;
      }

      {
        std::lock_guard<std::mutex> lk(g_imu_stats.m);
        g_imu_stats.have_ab = have_ab;
        g_imu_stats.a = a;
        g_imu_stats.b = b;
        g_imu_stats.win_n = win.size();
        g_imu_stats.last_rx_ns = t_rx_ns;
      }

      double t_est = have_ab ? (a * t_sensor + b) : t_host;
      uint64_t t_master_ns = (uint64_t) llround(t_est * 1e9);
      uint64_t t_utc_ns = 0;

      if (got_utc && (utc_valid & 0x03) == 0x03) {
        uint64_t utc_from_packet = 0;
        if (xsens_utc_to_ns(utc_year, utc_mon, utc_day, utc_hour, utc_min, utc_sec, utc_nanos, utc_from_packet)) {
          t_utc_ns = utc_from_packet;
          update_utc_sync_stats(t_master_ns, utc_from_packet);
        }
      } else {
        uint64_t utc_est = 0;
        if (utc_map.map(t_master_ns, utc_est)) t_utc_ns = utc_est;
      }

      ImuSample s;
      s.t_master_ns = t_master_ns;
      s.t_utc_ns = t_utc_ns;
      s.pc = pc_last;
      if (got_a) { std::memcpy(s.acc, acc, sizeof(acc)); s.have_acc = true; }
      if (got_g) { std::memcpy(s.gyro, gyro, sizeof(gyro)); s.have_gyro = true; }
      ring->push(s);

      if (got_latlon || got_alt || got_vel || got_utc) {
        GnssSample gs;
        gs.t_master_ns = t_master_ns;
        gs.t_utc_ns = t_utc_ns;
        gs.pc = pc_last;
        gs.utc_valid = utc_valid;
        gs.have_utc = got_utc && t_utc_ns != 0;
        if (got_latlon) {
          gs.lat_deg = lat_deg;
          gs.lon_deg = lon_deg;
          gs.have_pos = true;
        }
        if (got_alt) {
          gs.alt_m = alt_m;
          gs.have_alt = true;
        }
        if (got_vel) {
          for (int i = 0; i < 3; i++) gs.vel_mps[i] = vel_mps[i];
          gs.have_vel = true;
        }
        gnss_ring->push(gs);
      }
    }
  }

  ::close(fd);
}

// ---------------- Camera capture ----------------
struct FrameMeta {
  uint64_t v4l2_ts_ns = 0;     // raw from V4L2 (timeval->ns)
  uint64_t master_ts_ns = 0;   // mapped to MASTER domain via rtcpu offset
  uint32_t seq = 0;
  uint32_t flags = 0;
  std::vector<uint8_t> bytes;
};

struct MMapBuf {
  void*  start = nullptr;
  size_t length = 0;
};

class CameraDev {
public:
  explicit CameraDev(std::string path) : path_(std::move(path)) {}

  bool open_and_init() {
    fd_ = ::open(path_.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd_ < 0) {
      std::perror(("open " + path_).c_str());
      return false;
    }

    type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (!get_format()) {
      type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (!get_format()) {
        std::cerr << path_ << ": VIDIOC_G_FMT failed for both CAPTURE and CAPTURE_MPLANE\n";
        return false;
      }
    }

    if (!init_mmap(4)) return false;
    if (!stream_on()) return false;
    return true;
  }

  void close_all() {
    if (fd_ >= 0) {
      stream_off();
      for (auto& b : bufs_) {
        if (b.start && b.length) munmap(b.start, b.length);
      }
      ::close(fd_);
      fd_ = -1;
    }
  }

  int fd() const { return fd_; }
  const std::string& path() const { return path_; }
  uint32_t width() const { return width_; }
  uint32_t height() const { return height_; }
  uint32_t pixfmt() const { return pixfmt_; }

  bool dequeue_one(FrameMeta& out, uint64_t rtcpu_minus_master_offset_ns) {
    if (fd_ < 0) return false;

    v4l2_buffer buf{};
    v4l2_plane planes[VIDEO_MAX_PLANES]{};

    buf.type = type_;
    buf.memory = V4L2_MEMORY_MMAP;

    if (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
      buf.m.planes = planes;
      buf.length = num_planes_;
    }

    if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
      if (errno == EAGAIN) return false;
      std::perror((path_ + " VIDIOC_DQBUF").c_str());
      return false;
    }

    uint64_t ts_ns = uint64_t(buf.timestamp.tv_sec) * 1000000000ULL
                   + uint64_t(buf.timestamp.tv_usec) * 1000ULL;

    out.v4l2_ts_ns = ts_ns;
    out.seq = buf.sequence;
    out.flags = buf.flags;

    out.master_ts_ns = (uint64_t)((int64_t)ts_ns - (int64_t)rtcpu_minus_master_offset_ns);

    size_t bytes_used = 0;
    if (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
      bytes_used = buf.m.planes[0].bytesused;
    } else {
      bytes_used = buf.bytesused;
    }
    if (bytes_used > bufs_[buf.index].length) bytes_used = bufs_[buf.index].length;
    out.bytes.resize(bytes_used);
    if (bytes_used > 0) {
      std::memcpy(out.bytes.data(), bufs_[buf.index].start, bytes_used);
    }

    if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
      std::perror((path_ + " VIDIOC_QBUF").c_str());
      return false;
    }

    return true;
  }

private:
  bool get_format() {
    v4l2_format fmt{};
    fmt.type = type_;
    if (xioctl(fd_, VIDIOC_G_FMT, &fmt) < 0) return false;

    if (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
      width_  = fmt.fmt.pix_mp.width;
      height_ = fmt.fmt.pix_mp.height;
      pixfmt_ = fmt.fmt.pix_mp.pixelformat;
      num_planes_ = (int)fmt.fmt.pix_mp.num_planes;
      if (num_planes_ <= 0) num_planes_ = 1;
    } else {
      width_  = fmt.fmt.pix.width;
      height_ = fmt.fmt.pix.height;
      pixfmt_ = fmt.fmt.pix.pixelformat;
      num_planes_ = 1;
    }
    return true;
  }

  bool init_mmap(uint32_t count) {
    v4l2_requestbuffers req{};
    req.count = count;
    req.type = type_;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
      std::perror((path_ + " VIDIOC_REQBUFS").c_str());
      return false;
    }
    if (req.count < 2) {
      std::cerr << path_ << ": insufficient buffer count=" << req.count << "\n";
      return false;
    }

    bufs_.resize(req.count);

    for (uint32_t i = 0; i < req.count; i++) {
      v4l2_buffer buf{};
      v4l2_plane planes[VIDEO_MAX_PLANES]{};

      buf.type = type_;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;

      if (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        buf.m.planes = planes;
        buf.length = num_planes_;
      }

      if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
        std::perror((path_ + " VIDIOC_QUERYBUF").c_str());
        return false;
      }

      size_t len = 0;
      off_t  off = 0;

      if (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        len = buf.m.planes[0].length;
        off = (off_t)buf.m.planes[0].m.mem_offset;
      } else {
        len = buf.length;
        off = (off_t)buf.m.offset;
      }

      void* start = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, off);
      if (start == MAP_FAILED) {
        std::perror((path_ + " mmap").c_str());
        return false;
      }
      bufs_[i].start = start;
      bufs_[i].length = len;

      if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
        std::perror((path_ + " VIDIOC_QBUF(init)").c_str());
        return false;
      }
    }

    return true;
  }

  bool stream_on() {
    if (xioctl(fd_, VIDIOC_STREAMON, &type_) < 0) {
      std::perror((path_ + " VIDIOC_STREAMON").c_str());
      return false;
    }
    return true;
  }

  void stream_off() {
    if (fd_ >= 0) xioctl(fd_, VIDIOC_STREAMOFF, &type_);
  }

private:
  std::string path_;
  int fd_{-1};
  v4l2_buf_type type_{V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE};
  uint32_t width_{0}, height_{0}, pixfmt_{0};
  int num_planes_{1};
  std::vector<MMapBuf> bufs_;
};

static std::string fourcc_to_string(uint32_t pixfmt) {
  char s[5];
  s[0] = char(pixfmt & 0xFF);
  s[1] = char((pixfmt >> 8) & 0xFF);
  s[2] = char((pixfmt >> 16) & 0xFF);
  s[3] = char((pixfmt >> 24) & 0xFF);
  s[4] = '\0';
  for (int i = 0; i < 4; ++i) {
    if (s[i] < 32 || s[i] > 126) s[i] = '_';
  }
  return std::string(s);
}

static std::string frame_file_ext(uint32_t pixfmt) {
  if (pixfmt == V4L2_PIX_FMT_MJPEG || pixfmt == V4L2_PIX_FMT_JPEG) return ".jpg";
  return ".raw";
}

static bool write_binary_file(const std::filesystem::path& path, const std::vector<uint8_t>& bytes) {
  std::ofstream ofs(path, std::ios::binary);
  if (!ofs) return false;
  if (!bytes.empty()) ofs.write(reinterpret_cast<const char*>(bytes.data()), (std::streamsize)bytes.size());
  return ofs.good();
}

static bool save_group_bundle(const std::string& save_dir,
                              uint64_t group_seq,
                              uint64_t group_utc_ns,
                              bool have_group_utc,
                              const FrameMeta (&frames)[4],
                              const uint64_t (&aligned_ns)[4],
                              const uint64_t (&cam_utc_ns)[4],
                              bool have_cam_utc,
                              const CameraDev* camdev,
                              const std::vector<ImuSample>& imu3,
                              bool have_imu3,
                              const GnssSample* gnss,
                              bool have_gnss) {
  namespace fs = std::filesystem;

  std::ostringstream name;
  name << "sample_" << std::setw(6) << std::setfill('0') << group_seq;
  fs::path sample_dir = fs::path(save_dir) / name.str();
  std::error_code ec;
  fs::create_directories(sample_dir, ec);
  if (ec) {
    std::cerr << "[SAVE] create_directories failed: " << sample_dir << " (" << ec.message() << ")\n";
    return false;
  }

  for (int i = 0; i < 4; ++i) {
    fs::path frame_path = sample_dir / ("cam" + std::to_string(i) + frame_file_ext(camdev[i].pixfmt()));
    if (!write_binary_file(frame_path, frames[i].bytes)) {
      std::cerr << "[SAVE] failed to write " << frame_path << "\n";
      return false;
    }
  }

  fs::path meta_path = sample_dir / "meta.json";
  std::ofstream meta(meta_path);
  if (!meta) {
    std::cerr << "[SAVE] failed to open " << meta_path << "\n";
    return false;
  }

  meta << "{\n";
  meta << "  \"seq\": " << group_seq << ",\n";
  meta << "  \"group_master_ns\": " << aligned_ns[0] << ",\n";
  meta << "  \"group_utc_ns\": " << (have_group_utc ? std::to_string(group_utc_ns) : "null") << ",\n";
  meta << "  \"cams\": [\n";
  for (int i = 0; i < 4; ++i) {
    meta << "    {\n";
    meta << "      \"id\": " << i << ",\n";
    meta << "      \"dev\": \"" << camdev[i].path() << "\",\n";
    meta << "      \"seq\": " << frames[i].seq << ",\n";
    meta << "      \"v4l2_ts_ns\": " << frames[i].v4l2_ts_ns << ",\n";
    meta << "      \"master_ts_ns\": " << frames[i].master_ts_ns << ",\n";
    meta << "      \"aligned_ns\": " << aligned_ns[i] << ",\n";
    meta << "      \"utc_ns\": " << (have_cam_utc ? std::to_string(cam_utc_ns[i]) : "null") << ",\n";
    meta << "      \"width\": " << camdev[i].width() << ",\n";
    meta << "      \"height\": " << camdev[i].height() << ",\n";
    meta << "      \"pixfmt\": \"" << fourcc_to_string(camdev[i].pixfmt()) << "\",\n";
    meta << "      \"bytes\": " << frames[i].bytes.size() << ",\n";
    meta << "      \"file\": \"cam" << i << frame_file_ext(camdev[i].pixfmt()) << "\"\n";
    meta << "    }" << (i == 3 ? "\n" : ",\n");
  }
  meta << "  ],\n";
  meta << "  \"imu\": [\n";
  if (have_imu3) {
    for (size_t i = 0; i < imu3.size(); ++i) {
      const auto& s = imu3[i];
      meta << "    {\n";
      meta << "      \"idx\": " << i << ",\n";
      meta << "      \"master_ts_ns\": " << s.t_master_ns << ",\n";
      meta << "      \"utc_ns\": " << (s.t_utc_ns ? std::to_string(s.t_utc_ns) : "null") << ",\n";
      meta << "      \"pc\": " << s.pc << ",\n";
      meta << "      \"acc\": [" << s.acc[0] << ", " << s.acc[1] << ", " << s.acc[2] << "],\n";
      meta << "      \"gyro\": [" << s.gyro[0] << ", " << s.gyro[1] << ", " << s.gyro[2] << "]\n";
      meta << "    }" << (i + 1 == imu3.size() ? "\n" : ",\n");
    }
  }
  meta << "  ],\n";
  meta << "  \"gnss\": ";
  if (have_gnss && gnss) {
    meta << "{\n";
    meta << "    \"master_ts_ns\": " << gnss->t_master_ns << ",\n";
    meta << "    \"utc_ns\": " << (gnss->t_utc_ns ? std::to_string(gnss->t_utc_ns) : "null") << ",\n";
    meta << "    \"pc\": " << gnss->pc << ",\n";
    meta << "    \"utc_valid\": " << (unsigned)gnss->utc_valid << ",\n";
    meta << "    \"lat_deg\": " << (gnss->have_pos ? std::to_string(gnss->lat_deg) : "null") << ",\n";
    meta << "    \"lon_deg\": " << (gnss->have_pos ? std::to_string(gnss->lon_deg) : "null") << ",\n";
    meta << "    \"alt_m\": " << (gnss->have_alt ? std::to_string(gnss->alt_m) : "null") << ",\n";
    meta << "    \"vel_mps\": " << (gnss->have_vel
                                       ? ("[" + std::to_string(gnss->vel_mps[0]) + ", " +
                                          std::to_string(gnss->vel_mps[1]) + ", " +
                                          std::to_string(gnss->vel_mps[2]) + "]")
                                       : "null") << "\n";
    meta << "  }\n";
  } else {
    meta << "null\n";
  }
  meta << "}\n";
  return meta.good();
}

// ---------------- Drift model for cam offsets ----------------
// Model: off_ns(x) = p*x + q, where x is seconds since first sample (t0).
struct CamDriftModel {
  LinFitWindow fit;
  double p = 0.0; // ns/sec
  double q = 0.0; // ns
  bool have = false;

  bool have_t0 = false;
  double t0 = 0.0;

  void reset(double window_sec) {
    fit.window_sec = window_sec;
    fit.clear();
    p = 0.0; q = 0.0; have = false;
    have_t0 = false;
    t0 = 0.0;
  }

  void push(double t_sec, double off_ns) {
    if (!have_t0) { t0 = t_sec; have_t0 = true; }
    double x = t_sec - t0;
    fit.push(x, off_ns);

    double a=0.0, b=0.0;
    if (fit.estimate(a, b)) {
      p = a;
      q = b;
      have = true;
    }
  }

  int64_t corr_ns(double t_sec) const {
    if (!have) return 0;
    double x = have_t0 ? (t_sec - t0) : 0.0;
    double off = p * x + q;
    return (int64_t) llround(-off);
  }
};

// ---------------- Logging config ----------------
enum class LogLevel { PROD, DEBUG };

struct LogConfig {
  LogLevel level = LogLevel::PROD;
  bool dbg_drift = false;
  bool dbg_v4l2 = false;
  bool dbg_queue = false;
  bool dbg_imu_fit = false;
  bool dbg_mtdata = false;
};

// ---------------- CLI ----------------
static void usage(const char* argv0) {
  std::cerr <<
    "Usage:\n"
    "  " << argv0 << " [options] /dev/videoA /dev/videoB /dev/videoC /dev/videoD\n"
    "\n"
    "Core options:\n"
    "  --imu-dev PATH           default /dev/ttyUSB0\n"
    "  --imu-baud N             default 115200\n"
    "  --count N                default 0 (0=run forever)\n"
    "  --warmup N               default 20 groups\n"
    "  --calib-groups N         default 200 groups\n"
    "  --group-thresh-us N      default 1000 (after calib)\n"
    "  --calib-thresh-us N      default 50000 (during warmup+calib)\n"
    "  --rtcpu-offset-ns N      default 0 (auto-estimate)\n"
    "  --assume-latency-us N    default 33000 (used only for auto-estimate)\n"
    "  --print-every N          default 1\n"
    "  --quiet                  no per-group print\n"
    "  --save-dir PATH          save each grouped sample under PATH/sample_XXXXXX\n"
    "  --drift-win-sec N        default 30 (drift fit window after calib)\n"
    "  --max-corr-step-us N     default 50 (limit correction change per group)\n"
    "\n"
    "Logging:\n"
    "  --log prod|debug         default prod\n"
    "  --dbg-drift              show drift p/q per cam\n"
    "  --dbg-v4l2               show v4l2_ts/master_ts/aligned per cam\n"
    "  --dbg-queue              show queue sizes and drop counters\n"
    "  --dbg-imu-fit            show IMU fit stats (a,b, win size)\n"
    "  --dbg-mtdata             dump MTData2 data ids/sizes seen on serial\n"
    "\n";
}

static bool parse_u64(const char* s, uint64_t& out) {
  char* end=nullptr;
  unsigned long long v = strtoull(s, &end, 10);
  if (!end || *end != '\0') return false;
  out = (uint64_t)v;
  return true;
}
static bool parse_i64(const char* s, int64_t& out) {
  char* end=nullptr;
  long long v = strtoll(s, &end, 10);
  if (!end || *end != '\0') return false;
  out = (int64_t)v;
  return true;
}
static bool parse_double(const char* s, double& out) {
  char* end=nullptr;
  double v = strtod(s, &end);
  if (!end || *end != '\0') return false;
  out = v;
  return true;
}
static bool parse_loglevel(const std::string& s, LogLevel& out) {
  if (s == "prod" || s == "production") { out = LogLevel::PROD; return true; }
  if (s == "debug") { out = LogLevel::DEBUG; return true; }
  return false;
}

int main(int argc, char** argv) {
  signal(SIGINT, on_sigint);
  signal(SIGTERM, on_sigint);

  const uint64_t master_anchor_ns = now_master_ns();
  const uint64_t realtime_anchor_ns = now_realtime_ns();

  ImuConfig imu_cfg;
  uint64_t count = 0;
  int warmup = 20;
  int calib_groups = 200;
  int group_thresh_us = 1000;
  int calib_thresh_us = 50000;
  int print_every = 1;
  bool quiet = false;
  std::string save_dir;

  int64_t rtcpu_offset_ns_in = 0;
  int assume_latency_us = 33000;

  double drift_win_sec = 30.0;
  int max_corr_step_us = 50;

  LogConfig logcfg;
  std::vector<std::string> cams;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--help" || a == "-h") { usage(argv[0]); return 0; }
    else if (a == "--imu-dev" && i + 1 < argc) imu_cfg.dev = argv[++i];
    else if (a == "--imu-baud" && i + 1 < argc) imu_cfg.baud = std::atoi(argv[++i]);
    else if (a == "--count" && i + 1 < argc) { uint64_t v=0; if(!parse_u64(argv[++i], v)) return 2; count=v; }
    else if (a == "--warmup" && i + 1 < argc) warmup = std::atoi(argv[++i]);
    else if (a == "--calib-groups" && i + 1 < argc) calib_groups = std::atoi(argv[++i]);
    else if (a == "--group-thresh-us" && i + 1 < argc) group_thresh_us = std::atoi(argv[++i]);
    else if (a == "--calib-thresh-us" && i + 1 < argc) calib_thresh_us = std::atoi(argv[++i]);
    else if (a == "--rtcpu-offset-ns" && i + 1 < argc) { int64_t v=0; if(!parse_i64(argv[++i], v)) return 2; rtcpu_offset_ns_in=v; }
    else if (a == "--assume-latency-us" && i + 1 < argc) assume_latency_us = std::atoi(argv[++i]);
    else if (a == "--print-every" && i + 1 < argc) print_every = std::max(1, std::atoi(argv[++i]));
    else if (a == "--quiet") quiet = true;
    else if (a == "--save-dir" && i + 1 < argc) save_dir = argv[++i];
    else if (a == "--drift-win-sec" && i + 1 < argc) { double v=0; if(!parse_double(argv[++i], v)) return 2; drift_win_sec = v; }
    else if (a == "--max-corr-step-us" && i + 1 < argc) max_corr_step_us = std::atoi(argv[++i]);
    else if (a == "--log" && i + 1 < argc) {
      LogLevel ll;
      if (!parse_loglevel(argv[++i], ll)) { std::cerr << "Invalid --log value\n"; return 2; }
      logcfg.level = ll;
    }
    else if (a == "--dbg-drift") logcfg.dbg_drift = true;
    else if (a == "--dbg-v4l2") logcfg.dbg_v4l2 = true;
    else if (a == "--dbg-queue") logcfg.dbg_queue = true;
    else if (a == "--dbg-imu-fit") logcfg.dbg_imu_fit = true;
    else if (a == "--dbg-mtdata") logcfg.dbg_mtdata = true;
    else if (a.rfind("/dev/video", 0) == 0) cams.push_back(a);
    else {
      std::cerr << "Unknown arg: " << a << "\n";
      usage(argv[0]);
      return 2;
    }
  }

  if (cams.size() != 4) {
    std::cerr << "Need exactly 4 camera devices.\n";
    usage(argv[0]);
    return 2;
  }

  // If log=debug and user didn't explicitly toggle, default to "all on"
  if (logcfg.level == LogLevel::DEBUG &&
      !logcfg.dbg_drift && !logcfg.dbg_v4l2 && !logcfg.dbg_queue && !logcfg.dbg_imu_fit &&
      !logcfg.dbg_mtdata) {
    logcfg.dbg_drift = logcfg.dbg_v4l2 = logcfg.dbg_queue = logcfg.dbg_imu_fit = true;
  }

  std::cout << "MASTER CLOCK = CLOCK_MONOTONIC_RAW (anchor master_ns=" << master_anchor_ns
            << ", approx wall=" << ns_to_wall_string(realtime_anchor_ns) << ")\n";
  std::cout << "IMU dev=" << imu_cfg.dev << " baud=" << imu_cfg.baud << "\n";
  std::cout << "Cameras: " << cams[0] << " " << cams[1] << " " << cams[2] << " " << cams[3] << "\n";
  std::cout << "Drift fit window=" << drift_win_sec << " sec, max_corr_step=" << max_corr_step_us << " us/group\n";
  if (!save_dir.empty()) std::cout << "Save dir=" << save_dir << "\n";
  std::cout << "Log level=" << (logcfg.level == LogLevel::DEBUG ? "debug" : "prod")
            << " (dbg_drift=" << logcfg.dbg_drift
            << " dbg_v4l2=" << logcfg.dbg_v4l2
            << " dbg_queue=" << logcfg.dbg_queue
            << " dbg_imu_fit=" << logcfg.dbg_imu_fit
            << " dbg_mtdata=" << logcfg.dbg_mtdata
            << ")\n";
  g_dbg_mtdata.store(logcfg.dbg_mtdata);

  ImuRing imu_ring(2000);
  GnssRing gnss_ring(512);
  std::thread imu_th(imu_thread_main, imu_cfg, &imu_ring, &gnss_ring);

  std::vector<CameraDev> camdev;
  camdev.reserve(4);
  for (auto& c : cams) camdev.emplace_back(c);

  for (auto& c : camdev) {
    if (!c.open_and_init()) {
      std::cerr << "Failed to init " << c.path() << "\n";
      g_stop.store(true);
      imu_th.join();
      return 1;
    }
    std::cout << c.path() << ": CAPTURE " << c.width() << "x" << c.height() << " pixfmt=0x"
              << std::hex << c.pixfmt() << std::dec << "\n";
  }

  std::deque<FrameMeta> q[4];
  const size_t QMAX = 8;

  // Drop counters
  uint64_t drop_due_to_mismatch[4]{0,0,0,0};
  uint64_t drop_due_to_overflow[4]{0,0,0,0};

  // Drift models per cam (relative to cam0)
  CamDriftModel drift[4];
  for (int i=0;i<4;i++) drift[i].reset(drift_win_sec);

  // rtcpu offset (ns)
  int64_t rtcpu_minus_master_offset_ns = rtcpu_offset_ns_in;

  // Auto-estimate if not provided.
  if (rtcpu_minus_master_offset_ns == 0) {
    std::vector<int64_t> candidates;
    candidates.reserve(300);

    std::cout << "Auto-estimating rtcpu_minus_master_offset_ns ... (assume_latency_us=" << assume_latency_us << ")\n";
    uint64_t t_deadline = now_master_ns() + 3ULL * 1000000000ULL;
    while (!g_stop.load() && candidates.size() < 200 && now_master_ns() < t_deadline) {
      std::vector<pollfd> pfds(4);
      for (int i=0;i<4;i++) { pfds[i].fd = camdev[i].fd(); pfds[i].events = POLLIN; pfds[i].revents = 0; }
      int pr = poll(pfds.data(), pfds.size(), 200);
      if (pr <= 0) continue;

      for (int i=0;i<4;i++) {
        if (!(pfds[i].revents & POLLIN)) continue;
        FrameMeta fm{};
        uint64_t dq_now = now_master_ns();
        if (camdev[i].dequeue_one(fm, /*offset*/0)) {
          int64_t v4l2_ts = (int64_t)fm.v4l2_ts_ns;
          int64_t est = v4l2_ts - ( (int64_t)dq_now - (int64_t)assume_latency_us * 1000 );
          candidates.push_back(est);
        }
      }
    }

    if (candidates.size() < 50) {
      std::cerr << "Auto-estimate failed (not enough samples). You can pass --rtcpu-offset-ns manually.\n";
    } else {
      std::nth_element(candidates.begin(), candidates.begin() + candidates.size()/2, candidates.end());
      rtcpu_minus_master_offset_ns = candidates[candidates.size()/2];
      std::cout << "Estimated rtcpu_minus_master_offset_ns (median) = " << rtcpu_minus_master_offset_ns << " ns\n";
    }
  } else {
    std::cout << "Using rtcpu_minus_master_offset_ns = " << rtcpu_minus_master_offset_ns << " ns (from CLI)\n";
  }

  // FPS stats
  uint64_t last_fps_ns = now_master_ns();
  uint64_t frame_cnt[4]{0,0,0,0};
  uint64_t last_frame_cnt[4]{0,0,0,0};

  // Initial calibration accumulators (mean offsets relative to cam0)
  int total_groups = 0;
  int calib_collected = 0;
  double sum_off_ns[4]{0,0,0,0}; // off_i = cam_i - cam0 (ns)
  bool calib_done = false;

  int active_thresh_us = calib_thresh_us;

  std::cout << "Running... warmup=" << warmup
            << " calib_groups=" << calib_groups
            << " (during warm+cal thresh_us=" << calib_thresh_us
            << ", after calib thresh_us=" << group_thresh_us << ")\n";

  uint64_t group_seq = 0;

  // group forming uses current drift model (no clamp) - OK for grouping; clamp applied for printed aligned.
  auto aligned_ns_for_grouping = [&](int cam_i, const FrameMeta& fm)->uint64_t {
    if (cam_i == 0) return fm.master_ts_ns;
    double t_sec = (double)fm.master_ts_ns * 1e-9;
    int64_t corr = drift[cam_i].corr_ns(t_sec);
    return (uint64_t)((int64_t)fm.master_ts_ns + corr);
  };

  // clamp state for printed correction stability
  int64_t corr_ns_last[4] = {0,0,0,0};
  const int64_t max_step_ns = (int64_t)max_corr_step_us * 1000;

  while (!g_stop.load()) {
    if (count > 0 && group_seq >= count) break;

    std::vector<pollfd> pfds(4);
    for (int i=0;i<4;i++) { pfds[i].fd = camdev[i].fd(); pfds[i].events = POLLIN; pfds[i].revents = 0; }

    int pr = poll(pfds.data(), pfds.size(), 500);
    if (pr < 0) {
      if (errno == EINTR) continue;
      std::perror("poll");
      break;
    }

    if (pr > 0) {
      for (int i=0;i<4;i++) {
        if (!(pfds[i].revents & POLLIN)) continue;

        FrameMeta fm{};
        if (camdev[i].dequeue_one(fm, (uint64_t)rtcpu_minus_master_offset_ns)) {
          q[i].push_back(fm);
          frame_cnt[i]++;

          while (q[i].size() > QMAX) {
            q[i].pop_front();
            drop_due_to_overflow[i]++;
          }
        }
      }
    }

    while (!g_stop.load()) {
      bool all = true;
      for (int i=0;i<4;i++) if (q[i].empty()) { all = false; break; }
      if (!all) break;

      uint64_t tsA[4];
      for (int i=0;i<4;i++) tsA[i] = aligned_ns_for_grouping(i, q[i].front());

      uint64_t mn = *std::min_element(tsA, tsA+4);
      uint64_t mx = *std::max_element(tsA, tsA+4);
      uint64_t delta_ns = mx - mn;

      if ((int64_t)delta_ns <= (int64_t)active_thresh_us * 1000) {
        FrameMeta g[4];
        for (int i=0;i<4;i++) { g[i] = q[i].front(); q[i].pop_front(); }

        group_seq++;
        total_groups++;

        // Initial calibration: constant offsets
        if (!calib_done) {
          if (total_groups > warmup) {
            double base = (double)g[0].master_ts_ns;
            for (int i=0;i<4;i++) sum_off_ns[i] += ((double)g[i].master_ts_ns - base);
            calib_collected++;

            if (calib_collected >= calib_groups) {
              double mean_off_ns[4]{0,0,0,0};
              for (int i=0;i<4;i++) mean_off_ns[i] = sum_off_ns[i] / (double)calib_collected;

              for (int i=0;i<4;i++) {
                drift[i].reset(drift_win_sec);
                drift[i].p = 0.0;
                drift[i].q = mean_off_ns[i];
                drift[i].have = (i != 0);
              }
              drift[0].have = false;

              corr_ns_last[0] = 0;
              for (int i=1;i<4;i++) corr_ns_last[i] = (int64_t) llround(-mean_off_ns[i]);

              calib_done = true;
              active_thresh_us = group_thresh_us;

              std::cout << "\n[CALIB DONE] mean offsets (ns) relative to cam0:\n";
              for (int i=0;i<4;i++) {
                std::cout << "  cam" << i << " mean_off_ns=" << mean_off_ns[i]
                          << " (" << (mean_off_ns[i]/1000.0) << " us)\n";
              }
              std::cout << "Now tracking drift with window=" << drift_win_sec << " sec.\n\n";
            }
          }
        }

        if (calib_done) {
          double t_sec = (double)g[0].master_ts_ns * 1e-9;
          double base_master = (double)g[0].master_ts_ns;

          // update drift fit
          for (int i=1;i<4;i++) {
            double off_ns = ((double)g[i].master_ts_ns - base_master);
            drift[i].push(t_sec, off_ns);
          }

          // build clamped corrections for printing/association
          int64_t corr_ns_used[4] = {0,0,0,0};
          corr_ns_used[0] = 0;
          for (int i=1;i<4;i++) {
            int64_t want = drift[i].corr_ns(t_sec);
            int64_t step = want - corr_ns_last[i];
            if (step >  max_step_ns) want = corr_ns_last[i] + max_step_ns;
            if (step < -max_step_ns) want = corr_ns_last[i] - max_step_ns;
            corr_ns_last[i] = want;
            corr_ns_used[i] = want;
          }

          auto aligned_ns_print = [&](int cam_i, const FrameMeta& fm)->uint64_t {
            return (uint64_t)((int64_t)fm.master_ts_ns + corr_ns_used[cam_i]);
          };

          uint64_t a0 = aligned_ns_print(0, g[0]);
          uint64_t a1 = aligned_ns_print(1, g[1]);
          uint64_t a2 = aligned_ns_print(2, g[2]);
          uint64_t a3 = aligned_ns_print(3, g[3]);

          uint64_t arr[4] = {a0,a1,a2,a3};
          uint64_t mn2 = *std::min_element(arr, arr+4);
          uint64_t mx2 = *std::max_element(arr, arr+4);
          double aligned_delta_us = (double)(mx2 - mn2) / 1000.0;

          uint64_t t_frame = a0;
          uint64_t t_frame_utc_ns = 0;
          bool have_frame_utc = map_master_to_utc_from_stats(t_frame, t_frame_utc_ns);

          std::vector<ImuSample> imu3;
          bool ok3 = imu_ring.get_last_n_before(t_frame, 3, imu3);
          GnssSample gnss{};
          bool have_gnss = gnss_ring.get_last_before(t_frame, gnss);
          uint64_t aligned_ns_all[4] = {a0, a1, a2, a3};
          uint64_t cam_utc_ns_all[4] = {0, 0, 0, 0};
          if (have_frame_utc) {
            cam_utc_ns_all[0] = t_frame_utc_ns;
            if (!map_master_to_utc_from_stats(a1, cam_utc_ns_all[1])) have_frame_utc = false;
            if (!map_master_to_utc_from_stats(a2, cam_utc_ns_all[2])) have_frame_utc = false;
            if (!map_master_to_utc_from_stats(a3, cam_utc_ns_all[3])) have_frame_utc = false;
          }
          if (!have_frame_utc) {
            t_frame_utc_ns = 0;
            cam_utc_ns_all[0] = 0;
            cam_utc_ns_all[1] = 0;
            cam_utc_ns_all[2] = 0;
            cam_utc_ns_all[3] = 0;
          }

          uint64_t master_now = now_master_ns();
          bool do_print = !quiet && (print_every > 0) && ((group_seq % (uint64_t)print_every) == 0);

          if (!save_dir.empty()) {
            save_group_bundle(save_dir, group_seq, t_frame_utc_ns, have_frame_utc,
                              g, aligned_ns_all, cam_utc_ns_all, have_frame_utc,
                              camdev.data(), imu3, ok3, &gnss, have_gnss);
          }

          if (do_print) {
            // ---------- PROD: no MASTER_SINCE_START_S and no WALL ----------
            std::cout << "SEQ=" << group_seq
                    << " MASTER_NOW_NS=" << master_now
                    << " aligned_delta_us=" << aligned_delta_us
                    << "\n";

            std::cout << "cam0 aligned_ns=" << a0;
            if (have_frame_utc) std::cout << " utc_ns=" << t_frame_utc_ns;
            std::cout << " seq=" << g[0].seq << "\n";

            std::cout << "cam1 aligned_ns=" << a1;
            if (have_frame_utc) std::cout << " utc_ns=" << cam_utc_ns_all[1];
            std::cout << " seq=" << g[1].seq << "\n";

            std::cout << "cam2 aligned_ns=" << a2;
            if (have_frame_utc) std::cout << " utc_ns=" << cam_utc_ns_all[2];
            std::cout << " seq=" << g[2].seq << "\n";

            std::cout << "cam3 aligned_ns=" << a3;
            if (have_frame_utc) std::cout << " utc_ns=" << cam_utc_ns_all[3];
            std::cout << " seq=" << g[3].seq << "\n";

            // PROD: IMU 3 samples (oldest -> newest)
            if (ok3) {
              for (int k=0;k<3;k++) {
                const auto& s = imu3[k];
                double dt_us = ((int64_t)s.t_master_ns - (int64_t)t_frame) / 1000.0;
                std::cout << "  IMU" << k
                          << " ts=" << s.t_master_ns
                          << " utc_ns=" << s.t_utc_ns
                          << " dt_us=" << dt_us;
                if (s.have_acc)  std::cout << " acc=[" << s.acc[0] << "," << s.acc[1] << "," << s.acc[2] << "]";
                if (s.have_gyro) std::cout << " gyro=[" << s.gyro[0] << "," << s.gyro[1] << "," << s.gyro[2] << "]";
                std::cout << "\n";
              }
            } else {
              std::cout << "  [IMU] not enough samples yet\n";
            }

            if (have_gnss) {
              double gnss_dt_us = ((int64_t)gnss.t_master_ns - (int64_t)t_frame) / 1000.0;
              std::cout << "  GNSS ";
              if (gnss.have_pos) {
                std::cout << std::fixed << std::setprecision(7)
                          << "lat=" << gnss.lat_deg
                          << " lon=" << gnss.lon_deg
                          << std::defaultfloat;
              } else {
                std::cout << "lat=NA lon=NA";
              }
              std::cout << " ts=" << gnss.t_master_ns
                        << " utc_ns=" << gnss.t_utc_ns
                        << " dt_us=" << gnss_dt_us
                        << " pc=" << gnss.pc;
              if (gnss.have_alt) std::cout << " alt_m=" << gnss.alt_m;
              if (gnss.have_vel) std::cout << " vel=[" << gnss.vel_mps[0] << "," << gnss.vel_mps[1] << "," << gnss.vel_mps[2] << "]";
              if (gnss.have_utc) std::cout << " utc_valid=0x" << std::hex << (unsigned)gnss.utc_valid << std::dec;
              std::cout << "\n";
            } else {
              std::cout << "  [GNSS] no sample yet\n";
            }

            // ---------- DEBUG extras ----------
            if (logcfg.level == LogLevel::DEBUG) {
              double master_since_start_s = (double)(master_now - master_anchor_ns) * 1e-9;
              uint64_t wall_now = realtime_anchor_ns + (master_now - master_anchor_ns);

              // time line (debug-only)
              std::cout << "  [TIME] MASTER_SINCE_START_S=" << master_since_start_s
                        << " WALL~=" << ns_to_wall_string(wall_now) << "\n";

              if (logcfg.dbg_drift) {
                for (int i=1;i<4;i++) {
                  if (drift[i].have) {
                    double ppm = drift[i].p / 1e9 * 1e6;
                    std::cout << "  [DRIFT] cam" << i
                              << " p_ns_per_s=" << drift[i].p
                              << " (~" << ppm << " ppm)"
                              << " q_ns=" << drift[i].q
                              << " corr_used_ns=" << corr_ns_used[i]
                              << "\n";
                  } else {
                    std::cout << "  [DRIFT] cam" << i << " (no-fit-yet)\n";
                  }
                }
              }

              if (logcfg.dbg_v4l2) {
                std::cout << "  [V4L2] cam0 v4l2_ns=" << g[0].v4l2_ts_ns << " master_ns=" << g[0].master_ts_ns << " aligned_ns=" << a0 << " flags=0x" << std::hex << g[0].flags << std::dec << "\n";
                std::cout << "  [V4L2] cam1 v4l2_ns=" << g[1].v4l2_ts_ns << " master_ns=" << g[1].master_ts_ns << " aligned_ns=" << a1 << " flags=0x" << std::hex << g[1].flags << std::dec << "\n";
                std::cout << "  [V4L2] cam2 v4l2_ns=" << g[2].v4l2_ts_ns << " master_ns=" << g[2].master_ts_ns << " aligned_ns=" << a2 << " flags=0x" << std::hex << g[2].flags << std::dec << "\n";
                std::cout << "  [V4L2] cam3 v4l2_ns=" << g[3].v4l2_ts_ns << " master_ns=" << g[3].master_ts_ns << " aligned_ns=" << a3 << " flags=0x" << std::hex << g[3].flags << std::dec << "\n";
              }

              if (logcfg.dbg_queue) {
                std::cout << "  [QUEUE] qsize=[" << q[0].size() << "," << q[1].size() << "," << q[2].size() << "," << q[3].size() << "]"
                          << " drop_mismatch=[" << drop_due_to_mismatch[0] << "," << drop_due_to_mismatch[1] << "," << drop_due_to_mismatch[2] << "," << drop_due_to_mismatch[3] << "]"
                          << " drop_overflow=[" << drop_due_to_overflow[0] << "," << drop_due_to_overflow[1] << "," << drop_due_to_overflow[2] << "," << drop_due_to_overflow[3] << "]"
                          << "\n";
              }

              if (logcfg.dbg_imu_fit) {
                bool imu_have_ab;
                double imu_a, imu_b;
                size_t imu_win_n;
                uint64_t imu_last_rx_ns;
                {
                  std::lock_guard<std::mutex> lk(g_imu_stats.m);
                  imu_have_ab = g_imu_stats.have_ab;
                  imu_a = g_imu_stats.a;
                  imu_b = g_imu_stats.b;
                  imu_win_n = g_imu_stats.win_n;
                  imu_last_rx_ns = g_imu_stats.last_rx_ns;
                }
                std::cout << "  [IMU_FIT] have_ab=" << imu_have_ab
                          << " a=" << imu_a
                          << " b=" << imu_b
                          << " win_n=" << imu_win_n
                          << " last_rx_ns=" << imu_last_rx_ns
                          << "\n";

                bool utc_have_map;
                double utc_a, utc_b;
                size_t utc_win_n;
                uint64_t utc_last_master_ns, utc_last_ns;
                {
                  std::lock_guard<std::mutex> lk(g_utc_stats.m);
                  utc_have_map = g_utc_stats.have_map;
                  utc_a = g_utc_stats.a;
                  utc_b = g_utc_stats.b;
                  utc_win_n = g_utc_stats.win_n;
                  utc_last_master_ns = g_utc_stats.last_master_ns;
                  utc_last_ns = g_utc_stats.last_utc_ns;
                }
                std::cout << "  [UTC_SYNC] have_map=" << utc_have_map
                          << " a=" << utc_a
                          << " b=" << utc_b
                          << " win_n=" << utc_win_n
                          << " last_master_ns=" << utc_last_master_ns
                          << " last_utc_ns=" << utc_last_ns
                          << "\n";
              }
            }
          }
        }

        break;
      } else {
        // drop earliest (by aligned time) to try to re-sync
        int min_i = 0;
        uint64_t min_ts = aligned_ns_for_grouping(0, q[0].front());
        for (int i=1;i<4;i++) {
          uint64_t t = aligned_ns_for_grouping(i, q[i].front());
          if (t < min_ts) { min_ts = t; min_i = i; }
        }
        q[min_i].pop_front();
        drop_due_to_mismatch[min_i]++;
        continue;
      }
    }

    // FPS print every ~1s
    uint64_t now = now_master_ns();
    if (now - last_fps_ns >= 1000000000ULL) {
      double dt = (double)(now - last_fps_ns) * 1e-9;
      for (int i=0;i<4;i++) {
        double fps = (double)(frame_cnt[i] - last_frame_cnt[i]) / dt;
        std::cout << "[FPS] cam" << i << ": " << fps << " (total=" << frame_cnt[i] << ")\n";
        last_frame_cnt[i] = frame_cnt[i];
      }
      last_fps_ns = now;
    }
  }

  g_stop.store(true);

  for (auto& c : camdev) c.close_all();
  if (imu_th.joinable()) imu_th.join();

  std::cout << "Done.\n";
  return 0;
}
