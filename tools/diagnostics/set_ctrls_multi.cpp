#include <linux/videodev2.h>

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <condition_variable>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

static int xioctl(int fd, unsigned long req, void* arg) {
  int r;
  do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
  return r;
}

static uint32_t ctrl_class(uint32_t id) { return id & 0x0fff0000U; }

// From your `v4l2-ctl -l` output:
static constexpr uint32_t ID_SENSOR_MODE = 0x009a2008; // int64
static constexpr uint32_t ID_TRIG_PIN    = 0x009a2084; // u32 + has-payload
static constexpr uint32_t ID_TRIG_MODE   = 0x009a2085; // u32 + has-payload

static bool query_ctrl_exists(int fd, uint32_t id) {
#ifdef VIDIOC_QUERY_EXT_CTRL
  v4l2_query_ext_ctrl q{};
  q.id = id;
  if (xioctl(fd, VIDIOC_QUERY_EXT_CTRL, &q) == 0) return true;
#endif
  v4l2_queryctrl qc{};
  qc.id = id;
  return xioctl(fd, VIDIOC_QUERYCTRL, &qc) == 0;
}

static size_t payload_size_bytes(int fd, uint32_t id, size_t fallback) {
#ifdef VIDIOC_QUERY_EXT_CTRL
  v4l2_query_ext_ctrl q{};
  q.id = id;
  if (xioctl(fd, VIDIOC_QUERY_EXT_CTRL, &q) == 0) {
    // For has-payload controls, size should be elems * elem_size
    if (q.flags & V4L2_CTRL_FLAG_HAS_PAYLOAD) {
      size_t n = (size_t)q.elems * (size_t)q.elem_size;
      if (n > 0) return n;
    }
  }
#endif
  return fallback;
}

static bool set_ext_ctrls_camera_class(int fd, v4l2_ext_control* ctrls, uint32_t count) {
  v4l2_ext_controls ecs{};
  ecs.ctrl_class = V4L2_CTRL_CLASS_CAMERA; // 0x009a0000
  ecs.count = count;
  ecs.controls = ctrls;

  if (xioctl(fd, VIDIOC_S_EXT_CTRLS, &ecs) < 0) {
    std::fprintf(stderr, "VIDIOC_S_EXT_CTRLS failed: %s\n", std::strerror(errno));
    return false;
  }
  return true;
}

static bool get_ext_ctrls_camera_class(int fd, v4l2_ext_control* ctrls, uint32_t count) {
  v4l2_ext_controls ecs{};
  ecs.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
  ecs.count = count;
  ecs.controls = ctrls;

  if (xioctl(fd, VIDIOC_G_EXT_CTRLS, &ecs) < 0) {
    std::fprintf(stderr, "VIDIOC_G_EXT_CTRLS failed: %s\n", std::strerror(errno));
    return false;
  }
  return true;
}

struct Barrier {
  explicit Barrier(int n) : total_(n), count_(n) {}
  void arrive_and_wait() {
    std::unique_lock<std::mutex> lk(m_);
    if (--count_ == 0) {
      count_ = total_;
      gen_++;
      cv_.notify_all();
    } else {
      int g = gen_;
      cv_.wait(lk, [&]{ return gen_ != g; });
    }
  }
private:
  int total_;
  int count_;
  int gen_{0};
  std::mutex m_;
  std::condition_variable cv_;
};

static std::vector<std::string> split(const std::string& s, char delim) {
  std::vector<std::string> out;
  std::string cur;
  std::istringstream iss(s);
  while (std::getline(iss, cur, delim)) if (!cur.empty()) out.push_back(cur);
  return out;
}

static bool parse_int_list(const std::string& s, std::vector<int>& out) {
  out.clear();
  for (auto& tok : split(s, ',')) {
    try { out.push_back(std::stoi(tok)); }
    catch (...) { return false; }
  }
  return true;
}

static bool parse_u32_list_hex(const std::string& s, std::vector<uint32_t>& out) {
  out.clear();
  for (auto& tok : split(s, ',')) {
    std::string t = tok;
    if (t.rfind("0x", 0) == 0 || t.rfind("0X", 0) == 0) t = t.substr(2);
    char* end = nullptr;
    unsigned long v = std::strtoul(t.c_str(), &end, 16);
    if (!end || *end != '\0') return false;
    out.push_back(static_cast<uint32_t>(v));
  }
  return true;
}

static uint32_t parse_hex_u32(std::string s) {
  if (s.rfind("0x", 0) == 0 || s.rfind("0X", 0) == 0) s = s.substr(2);
  return static_cast<uint32_t>(std::strtoul(s.c_str(), nullptr, 16));
}

static void usage(const char* argv0) {
  std::fprintf(stderr,
    "Usage:\n"
    "  %s [options] [/dev/videoX ...]\n"
    "\n"
    "Options (defaults if omitted):\n"
    "  --sensor-mode N          default 3\n"
    "  --trig-mode N            default 2\n"
    "  --trig-pin HEX           default 0xffff0007\n"
    "  --print                  read back after set\n"
    "\n"
    "Per-device overrides (comma-separated, must match device count):\n"
    "  --sensor-modes a,b,c,d\n"
    "  --trig-modes   a,b,c,d\n"
    "  --trig-pins    h1,h2,h3,h4\n"
    "\n"
    "Devices:\n"
    "  If none given: /dev/video0 /dev/video1 /dev/video2 /dev/video3\n",
    argv0);
}

struct DevCfg {
  std::string dev;
  int sensor_mode = 3;
  int trig_mode   = 2;
  uint32_t trig_pin_u = 0xffff0007u;
};

int main(int argc, char** argv) {
  int sensor_mode_default = 3;
  int trig_mode_default   = 2;
  uint32_t trig_pin_default = 0xffff0007u;

  std::vector<int> sensor_modes;
  std::vector<int> trig_modes;
  std::vector<uint32_t> trig_pins;

  std::vector<std::string> devs;
  bool do_print = false;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
    if (a == "--print") { do_print = true; continue; }

    if (a == "--sensor-mode" && i + 1 < argc) sensor_mode_default = std::stoi(argv[++i]);
    else if (a == "--trig-mode" && i + 1 < argc) trig_mode_default = std::stoi(argv[++i]);
    else if (a == "--trig-pin" && i + 1 < argc) trig_pin_default = parse_hex_u32(argv[++i]);

    else if (a == "--sensor-modes" && i + 1 < argc) {
      if (!parse_int_list(argv[++i], sensor_modes)) { usage(argv[0]); return 2; }
    } else if (a == "--trig-modes" && i + 1 < argc) {
      if (!parse_int_list(argv[++i], trig_modes)) { usage(argv[0]); return 2; }
    } else if (a == "--trig-pins" && i + 1 < argc) {
      if (!parse_u32_list_hex(argv[++i], trig_pins)) { usage(argv[0]); return 2; }
    } else if (a.rfind("/dev/video", 0) == 0) {
      devs.push_back(a);
    } else {
      std::fprintf(stderr, "Unknown arg: %s\n", a.c_str());
      usage(argv[0]);
      return 2;
    }
  }

  if (devs.empty()) devs = {"/dev/video0", "/dev/video1", "/dev/video2", "/dev/video3"};

  const int n = (int)devs.size();
  if (!sensor_modes.empty() && (int)sensor_modes.size() != n) {
    std::fprintf(stderr, "--sensor-modes count must match devices (%d)\n", n);
    return 2;
  }
  if (!trig_modes.empty() && (int)trig_modes.size() != n) {
    std::fprintf(stderr, "--trig-modes count must match devices (%d)\n", n);
    return 2;
  }
  if (!trig_pins.empty() && (int)trig_pins.size() != n) {
    std::fprintf(stderr, "--trig-pins count must match devices (%d)\n", n);
    return 2;
  }

  std::vector<DevCfg> cfgs(n);
  for (int i = 0; i < n; ++i) {
    cfgs[i].dev = devs[i];
    cfgs[i].sensor_mode = sensor_modes.empty() ? sensor_mode_default : sensor_modes[i];
    cfgs[i].trig_mode   = trig_modes.empty()   ? trig_mode_default   : trig_modes[i];
    cfgs[i].trig_pin_u  = trig_pins.empty()    ? trig_pin_default    : trig_pins[i];
  }

  std::vector<int> fds(n, -1);
  for (int i = 0; i < n; ++i) {
    fds[i] = open(cfgs[i].dev.c_str(), O_RDWR | O_NONBLOCK);
    if (fds[i] < 0) {
      std::fprintf(stderr, "open %s failed: %s\n", cfgs[i].dev.c_str(), std::strerror(errno));
      for (int j = 0; j < n; ++j) if (fds[j] >= 0) close(fds[j]);
      return 1;
    }
    if (!query_ctrl_exists(fds[i], ID_SENSOR_MODE) ||
        !query_ctrl_exists(fds[i], ID_TRIG_MODE) ||
        !query_ctrl_exists(fds[i], ID_TRIG_PIN)) {
      std::fprintf(stderr, "%s: expected NVIDIA camera ctrls not found (0x009a2008/0x009a2085/0x009a2084)\n",
                   cfgs[i].dev.c_str());
      for (int j = 0; j < n; ++j) close(fds[j]);
      return 3;
    }
  }

  Barrier barrier(n);
  std::atomic<int> fail{0};
  std::vector<std::thread> threads;
  threads.reserve(n);

  for (int i = 0; i < n; ++i) {
    threads.emplace_back([&, i] {
      barrier.arrive_and_wait();

      // payload sizes (usually 4 bytes, but query to be safe)
      size_t sz_mode = payload_size_bytes(fds[i], ID_TRIG_MODE, sizeof(uint32_t));
      size_t sz_pin  = payload_size_bytes(fds[i], ID_TRIG_PIN,  sizeof(uint32_t));

      std::vector<uint8_t> mode_payload(sz_mode, 0);
      std::vector<uint8_t> pin_payload(sz_pin, 0);

      uint32_t trig_mode_u = (uint32_t)cfgs[i].trig_mode;
      uint32_t trig_pin_u  = cfgs[i].trig_pin_u;

      std::memcpy(mode_payload.data(), &trig_mode_u, std::min(sz_mode, sizeof(trig_mode_u)));
      std::memcpy(pin_payload.data(),  &trig_pin_u,  std::min(sz_pin,  sizeof(trig_pin_u)));

      v4l2_ext_control ctrls[3]{};

      // sensor_mode: int64 -> value64 (no payload)
      ctrls[0].id = ID_SENSOR_MODE;
      ctrls[0].value64 = (int64_t)cfgs[i].sensor_mode;

      // trig_mode: payload
      ctrls[1].id = ID_TRIG_MODE;
      ctrls[1].size = (uint32_t)sz_mode;
      ctrls[1].ptr = mode_payload.data();

      // trig_pin: payload
      ctrls[2].id = ID_TRIG_PIN;
      ctrls[2].size = (uint32_t)sz_pin;
      ctrls[2].ptr = pin_payload.data();

      bool ok = set_ext_ctrls_camera_class(fds[i], ctrls, 3);
      if (!ok) {
        std::fprintf(stderr, "%s: SET FAILED\n", cfgs[i].dev.c_str());
        fail++;
        return;
      }

      std::printf("%s: SET OK sensor_mode=%d trig_mode=%u trig_pin=0x%08x\n",
                  cfgs[i].dev.c_str(), cfgs[i].sensor_mode, trig_mode_u, trig_pin_u);

      if (do_print) {
        // readback buffers
        std::vector<uint8_t> mode_rb(sz_mode, 0);
        std::vector<uint8_t> pin_rb(sz_pin, 0);

        v4l2_ext_control r[3]{};
        r[0].id = ID_SENSOR_MODE; // value64
        r[1].id = ID_TRIG_MODE; r[1].size = (uint32_t)sz_mode; r[1].ptr = mode_rb.data();
        r[2].id = ID_TRIG_PIN;  r[2].size = (uint32_t)sz_pin;  r[2].ptr = pin_rb.data();

        bool rok = get_ext_ctrls_camera_class(fds[i], r, 3);
        if (!rok) {
          std::fprintf(stderr, "%s: READBACK FAILED\n", cfgs[i].dev.c_str());
          return;
        }

        int64_t sensor_rb = r[0].value64;
        uint32_t trig_mode_rb = 0, trig_pin_rb = 0;
        std::memcpy(&trig_mode_rb, mode_rb.data(), std::min(sz_mode, sizeof(trig_mode_rb)));
        std::memcpy(&trig_pin_rb,  pin_rb.data(),  std::min(sz_pin,  sizeof(trig_pin_rb)));

        std::printf("%s: READBACK sensor_mode=%lld trig_mode=%u trig_pin=0x%08x\n",
                    cfgs[i].dev.c_str(), (long long)sensor_rb, trig_mode_rb, trig_pin_rb);
      }
    });
  }

  for (auto& t : threads) t.join();
  for (int i = 0; i < n; ++i) close(fds[i]);

  return (fail.load() == 0) ? 0 : 4;
}
