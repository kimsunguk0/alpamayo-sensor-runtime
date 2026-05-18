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

static std::atomic<bool> g_stop{false};

static void on_sigint(int) { g_stop.store(true); }

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
static inline float be_f32(const uint8_t* p) {
  uint32_t u = be_u32(p);
  float f;
  std::memcpy(&f, &u, sizeof(float));
  return f;
}
static bool checksum_ok_deque(const std::deque<uint8_t>& buf, size_t frame_len) {
  // Xsens: sum(all bytes excluding preamble) & 0xFF == 0 (checksum included)
  uint32_t sum = 0;
  for (size_t i = 1; i < frame_len; i++) sum += buf[i];
  return (sum & 0xFF) == 0;
}

// Sliding window linear regression: t_host = a * t_sensor + b
// where t_sensor = SampleTimeFine / 10000.0 (sec), t_host = master clock seconds
struct FitWindow {
  struct XY { double x; double y; };
  std::deque<XY> q;
  double sum_x=0, sum_y=0, sum_xx=0, sum_xy=0;
  double window_sec = 10.0;

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
    if (n < 20) return false;
    double denom = (double)n * sum_xx - sum_x * sum_x;
    if (std::abs(denom) < 1e-12) return false;
    a = ((double)n * sum_xy - sum_x * sum_y) / denom;
    b = (sum_y - a * sum_x) / (double)n;
    return true;
  }

  size_t size() const { return q.size(); }
};

// ---------------- IMU sample + ring buffer ----------------
struct ImuSample {
  uint64_t t_master_ns = 0;   // mapped to MASTER CLOCK (MONOTONIC_RAW) domain
  uint16_t pc = 0;
  float acc[3]{0,0,0};
  float gyro[3]{0,0,0};
  bool have_acc=false;
  bool have_gyro=false;
};

class ImuRing {
public:
  explicit ImuRing(size_t cap) : cap_(cap), buf_(cap) {}

  void push(const ImuSample& s) {
    std::lock_guard<std::mutex> lk(m_);
    buf_[head_] = s;
    head_ = (head_ + 1) % cap_;
    if (count_ < cap_) count_++;
    // no condition_variable needed for now (camera loop is slow)
  }

  // Get exactly N samples with ts <= t_ref, newest-first then returned oldest->newest.
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

// ---------------- IMU thread ----------------
struct ImuConfig {
  std::string dev = "/dev/ttyUSB0";
  int baud = 115200;
  double win_sec = 10.0;
};

static void imu_thread_main(const ImuConfig cfg, ImuRing* ring) {
  // byte time estimation
  const double bits_per_byte = 10.0;
  const uint64_t byte_time_ns = (uint64_t) llround(1e9 * bits_per_byte / (double)cfg.baud);

  int fd = ::open(cfg.dev.c_str(), O_RDONLY | O_NOCTTY);
  if (fd < 0) {
    std::perror("IMU open");
    return;
  }
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

  FitWindow win;
  win.window_sec = cfg.win_sec;
  double a = 1.0, b = 0.0;
  bool have_ab = false;

  uint16_t pc_last = 0;

  while (!g_stop.load()) {
    int n = ::read(fd, tmp.data(), (int)tmp.size());
    if (n > 0) {
      uint64_t t_read_ns = now_master_ns();
      for (int i = 0; i < n; i++) {
        uint64_t t_i = t_read_ns - (uint64_t)((n - 1 - i) * byte_time_ns);
        buf.push_back(tmp[i]);
        tbuf.push_back(t_i);
      }
    } else {
      // prevent busy-spin
      usleep(1000);
    }

    // safety cap (avoid runaway on garbage stream)
    if (buf.size() > (1u << 20)) {
      buf.clear();
      tbuf.clear();
    }

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

      // copy payload bytes into a small contiguous buffer (reuse)
      static std::vector<uint8_t> frame;
      frame.resize(frame_len);
      for (size_t i = 0; i < frame_len; i++) frame[i] = buf[i];
      for (size_t i = 0; i < frame_len; i++) { buf.pop_front(); tbuf.pop_front(); }

      if (mid != 0x36) continue; // MTData2 only

      const uint8_t* payload = frame.data() + hdr;
      size_t p = 0;

      bool got_stf=false, got_pc=false, got_a=false, got_g=false;
      uint32_t stf32 = 0;
      uint16_t pc = 0;
      float acc[3]{}, gyro[3]{};

      while (p + 3 <= data_len) {
        uint16_t dataId = be_u16(payload + p);
        uint8_t  sz     = payload[p + 2];
        p += 3;
        if (p + sz > data_len) break;

        uint16_t id_mask = dataId & 0xFFF0;

        if (dataId == 0x1060 && sz == 4) {           // SampleTimeFine
          stf32 = be_u32(payload + p);
          got_stf = true;
        } else if (dataId == 0x1020 && sz == 2) {    // PacketCounter
          pc = be_u16(payload + p);
          got_pc = true;
        } else if (id_mask == 0x4020 && sz == 12) {  // Acc (float32)
          for (int i = 0; i < 3; i++) acc[i] = be_f32(payload + p + 4*i);
          got_a = true;
        } else if (id_mask == 0x8020 && sz == 12) {  // Gyro (float32)
          for (int i = 0; i < 3; i++) gyro[i] = be_f32(payload + p + 4*i);
          got_g = true;
        }

        p += sz;
      }

      if (!got_stf) continue;

      // extend STF with wrap handling (uint32 wrap-safe delta)
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

      // sensor time in seconds
      double t_sensor = (double)stf_ext / 10000.0;
      double t_host   = (double)t_rx_ns * 1e-9;

      win.push(t_sensor, t_host);
      have_ab = win.estimate(a, b);

      double t_est = have_ab ? (a * t_sensor + b) : t_host;
      uint64_t t_master_ns = (uint64_t) llround(t_est * 1e9);

      ImuSample s;
      s.t_master_ns = t_master_ns;
      s.pc = pc_last;
      if (got_a) { std::memcpy(s.acc, acc, sizeof(acc)); s.have_acc = true; }
      if (got_g) { std::memcpy(s.gyro, gyro, sizeof(gyro)); s.have_gyro = true; }

      ring->push(s);
    }
  }

  ::close(fd);
}

// ---------------- Camera capture ----------------
struct FrameMeta {
  uint64_t v4l2_ts_ns = 0;    // raw from V4L2 (timeval->ns)
  uint64_t master_ts_ns = 0;  // mapped to MASTER domain via rtcpu offset
  uint64_t aligned_ns = 0;    // master + per-cam correction
  uint32_t seq = 0;
  uint32_t flags = 0;
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

    // Try MPLANE first (Jetson commonly uses it)
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

  bool dequeue_one(FrameMeta& out, uint64_t rtcpu_minus_master_offset_ns, int64_t cam_corr_ns) {
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

    // timestamp timeval -> ns
    uint64_t ts_ns = uint64_t(buf.timestamp.tv_sec) * 1000000000ULL
                   + uint64_t(buf.timestamp.tv_usec) * 1000ULL;

    out.v4l2_ts_ns = ts_ns;
    out.seq = buf.sequence;
    out.flags = buf.flags;

    // master mapping: master = rtcpu_ts - offset
    out.master_ts_ns = (uint64_t)((int64_t)ts_ns - (int64_t)rtcpu_minus_master_offset_ns);
    out.aligned_ns   = (uint64_t)((int64_t)out.master_ts_ns + cam_corr_ns);

    // re-queue immediately (we do not use image data here)
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

      // queue buffer
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

// ---------------- CLI ----------------
static void usage(const char* argv0) {
  std::cerr <<
    "Usage:\n"
    "  " << argv0 << " [options] /dev/videoA /dev/videoB /dev/videoC /dev/videoD\n"
    "\n"
    "Options:\n"
    "  --imu-dev PATH           default /dev/ttyUSB0\n"
    "  --imu-baud N             default 115200\n"
    "  --count N                default 0 (0=run forever)\n"
    "  --warmup N               default 20 groups\n"
    "  --calib-groups N         default 200 groups\n"
    "  --group-thresh-us N      default 1000 (after calib)\n"
    "  --calib-thresh-us N      default 50000 (during warmup+calib)\n"
    "  --rtcpu-offset-ns N      default 0 (auto-estimate)\n"
    "  --assume-latency-us N    default 33000 (used only for auto-estimate)\n"
    "  --print-every N          default 1 (print every group)\n"
    "  --quiet                  no per-group print (still prints calib + FPS)\n"
    "\n"
    "IMU association (no interpolation): for each camera group time = cam0 aligned,\n"
    "attach the last 3 IMU samples with imu_ts <= frame_ts.\n";
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

int main(int argc, char** argv) {
  signal(SIGINT, on_sigint);
  signal(SIGTERM, on_sigint);

  ImuConfig imu_cfg;
  uint64_t count = 0;
  int warmup = 20;
  int calib_groups = 200;
  int group_thresh_us = 1000;
  int calib_thresh_us = 50000;
  int print_every = 1;
  bool quiet = false;

  int64_t rtcpu_offset_ns_in = 0;
  int assume_latency_us = 33000;

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

  std::cout << "MASTER CLOCK = CLOCK_MONOTONIC_RAW\n";
  std::cout << "IMU dev=" << imu_cfg.dev << " baud=" << imu_cfg.baud << "\n";
  std::cout << "Cameras: " << cams[0] << " " << cams[1] << " " << cams[2] << " " << cams[3] << "\n";

  // IMU ring buffer (10 seconds @100Hz ~ 1000; use 2000)
  ImuRing imu_ring(2000);
  std::thread imu_th(imu_thread_main, imu_cfg, &imu_ring);

  // Open cameras
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

  // Per-cam queues (bounded)
  std::deque<FrameMeta> q[4];
  const size_t QMAX = 8;

  // Per-cam correction (ns). cam0 is always 0.
  int64_t cam_corr_ns[4]{0,0,0,0};

  // rtcpu offset (ns)
  int64_t rtcpu_minus_master_offset_ns = rtcpu_offset_ns_in;

  // Auto-estimate rtcpu offset if not provided.
  // We approximate: offset ≈ median( v4l2_ts - (dq_master_now - assume_latency) )
  if (rtcpu_minus_master_offset_ns == 0) {
    std::vector<int64_t> candidates;
    candidates.reserve(300);

    std::cout << "Auto-estimating rtcpu_minus_master_offset_ns ... (assume_latency_us=" << assume_latency_us << ")\n";
    uint64_t t_deadline = now_master_ns() + 3ULL * 1000000000ULL; // ~3s max
    while (!g_stop.load() && candidates.size() < 200 && now_master_ns() < t_deadline) {
      std::vector<pollfd> pfds(4);
      for (int i=0;i<4;i++) { pfds[i].fd = camdev[i].fd(); pfds[i].events = POLLIN; pfds[i].revents = 0; }
      int pr = poll(pfds.data(), pfds.size(), 200);
      if (pr <= 0) continue;

      for (int i=0;i<4;i++) {
        if (!(pfds[i].revents & POLLIN)) continue;
        FrameMeta fm{};
        uint64_t dq_now = now_master_ns();
        // temporarily treat cam_corr=0 for estimation
        if (camdev[i].dequeue_one(fm, /*offset*/0, /*corr*/0)) {
          // fm.master_ts_ns currently equals v4l2_ts (because offset=0 used)
          int64_t v4l2_ts = (int64_t)fm.v4l2_ts_ns;
          int64_t est = v4l2_ts - ( (int64_t)dq_now - (int64_t)assume_latency_us * 1000 );
          candidates.push_back(est);
        }
      }
    }

    if (candidates.size() < 50) {
      std::cerr << "Auto-estimate failed (not enough samples). You can pass --rtcpu-offset-ns manually.\n";
      // fallback: keep 0 -> will still run but camera/IMU absolute alignment will be wrong
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

  // Calibration accumulators (mean offsets relative to cam0)
  int total_groups = 0;
  int calib_collected = 0;
  double sum_off_us[4]{0,0,0,0}; // off_i = cam_i - cam0 (in us)
  bool calib_done = false;

  const int total_warm_cal = warmup + calib_groups;
  int active_thresh_us = calib_done ? group_thresh_us : calib_thresh_us;

  std::cout << "Running... warmup=" << warmup
            << " calib_groups=" << calib_groups
            << " (during warm+cal thresh_us=" << calib_thresh_us
            << ", after calib thresh_us=" << group_thresh_us << ")\n";

  uint64_t group_seq = 0;

  while (!g_stop.load()) {
    // stop by count (groups)
    if (count > 0 && group_seq >= count) break;

    // poll cameras
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
        if (camdev[i].dequeue_one(fm, (uint64_t)rtcpu_minus_master_offset_ns, cam_corr_ns[i])) {
          q[i].push_back(fm);
          frame_cnt[i]++;
          while (q[i].size() > QMAX) q[i].pop_front();
        }
      }
    }

    // try form groups
    bool made = false;
    while (!g_stop.load()) {
      bool all = true;
      for (int i=0;i<4;i++) if (q[i].empty()) { all = false; break; }
      if (!all) break;

      uint64_t ts[4];
      for (int i=0;i<4;i++) ts[i] = q[i].front().aligned_ns;

      uint64_t mn = *std::min_element(ts, ts+4);
      uint64_t mx = *std::max_element(ts, ts+4);
      uint64_t delta_ns = mx - mn;

      if ((int64_t)delta_ns <= (int64_t)active_thresh_us * 1000) {
        // group formed
        FrameMeta g[4];
        for (int i=0;i<4;i++) { g[i] = q[i].front(); q[i].pop_front(); }
        made = true;

        group_seq++;
        total_groups++;

        // Calibration logic
        if (!calib_done) {
          if (total_groups <= warmup) {
            // warmup: do nothing
          } else {
            // collect offsets for calib
            double base = (double)g[0].master_ts_ns;
            for (int i=0;i<4;i++) {
              double off_us = ((double)g[i].master_ts_ns - base) / 1000.0;
              sum_off_us[i] += off_us;
            }
            calib_collected++;

            if (calib_collected >= calib_groups) {
              // compute mean offsets
              double mean_off_us[4]{0,0,0,0};
              for (int i=0;i<4;i++) mean_off_us[i] = sum_off_us[i] / (double)calib_collected;

              // cam correction = -mean_offset
              cam_corr_ns[0] = 0;
              for (int i=1;i<4;i++) cam_corr_ns[i] = (int64_t) llround((-mean_off_us[i]) * 1000.0);

              calib_done = true;
              active_thresh_us = group_thresh_us;

              std::cout << "\n[CALIB DONE] cam EOF mean offsets (us) relative to cam0:\n";
              for (int i=0;i<4;i++) {
                std::cout << "  cam" << i << " off_us=" << mean_off_us[i] << "\n";
              }
              std::cout << "Applying corrections (ns): ["
                        << cam_corr_ns[0] << ", " << cam_corr_ns[1] << ", "
                        << cam_corr_ns[2] << ", " << cam_corr_ns[3] << "]\n\n";
            }
          }
        }

        // After calib (or even during), output association with IMU
        if (calib_done) {
          uint64_t t_frame = g[0].aligned_ns; // cam0 aligned = reference
          std::vector<ImuSample> imu3;
          bool ok3 = imu_ring.get_last_n_before(t_frame, 3, imu3);

          if (!ok3) {
            // not enough IMU samples yet
            break;
          }

          uint64_t aligned_ts[4] = {g[0].aligned_ns, g[1].aligned_ns, g[2].aligned_ns, g[3].aligned_ns};
          uint64_t mn2 = *std::min_element(aligned_ts, aligned_ts+4);
          uint64_t mx2 = *std::max_element(aligned_ts, aligned_ts+4);
          double aligned_delta_us = (double)(mx2 - mn2) / 1000.0;

          if (!quiet && (print_every > 0) && ((group_seq % (uint64_t)print_every) == 0)) {
            std::cout << "SEQ=" << group_seq
                      << " FRAME_NS=" << t_frame
                      << " aligned_delta_us=" << aligned_delta_us
                      << " | cam0(aligned=" << g[0].aligned_ns << ")"
                      << " cam1(aligned=" << g[1].aligned_ns << ", r_us=" << ((int64_t)g[1].aligned_ns - (int64_t)g[0].aligned_ns)/1000.0 << ")"
                      << " cam2(aligned=" << g[2].aligned_ns << ", r_us=" << ((int64_t)g[2].aligned_ns - (int64_t)g[0].aligned_ns)/1000.0 << ")"
                      << " cam3(aligned=" << g[3].aligned_ns << ", r_us=" << ((int64_t)g[3].aligned_ns - (int64_t)g[0].aligned_ns)/1000.0 << ")\n";

            // IMU 3 samples (oldest -> newest)
            for (int k=0;k<3;k++) {
              const auto& s = imu3[k];
              double dt_us = ((int64_t)s.t_master_ns - (int64_t)t_frame) / 1000.0;
              std::cout << "  IMU" << k
                        << " ts=" << s.t_master_ns
                        << " dt_us=" << dt_us;
              if (s.have_acc) {
                std::cout << " acc=[" << s.acc[0] << "," << s.acc[1] << "," << s.acc[2] << "]";
              }
              if (s.have_gyro) {
                std::cout << " gyro=[" << s.gyro[0] << "," << s.gyro[1] << "," << s.gyro[2] << "]";
              }
              std::cout << "\n";
            }
          }
        }

        break;
      } else {
        // drop earliest
        int min_i = 0;
        for (int i=1;i<4;i++) if (q[i].front().aligned_ns < q[min_i].front().aligned_ns) min_i = i;
        q[min_i].pop_front();
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

    // if nothing happened, loop continues
    (void)made;
  }

  g_stop.store(true);

  for (auto& c : camdev) c.close_all();
  if (imu_th.joinable()) imu_th.join();

  std::cout << "Done.\n";
  return 0;
}
