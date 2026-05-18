#include <linux/videodev2.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>

static std::atomic<bool> g_run{true};
static void on_sigint(int) { g_run = false; }

static int xioctl(int fd, unsigned long req, void* arg) {
  int r;
  do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
  return r;
}

static void die(const char* msg) {
  std::perror(msg);
  std::exit(1);
}

static uint64_t mono_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

struct PlaneMap {
  void*  ptr{nullptr};
  size_t len{0};
};

struct BufMap {
  std::vector<PlaneMap> planes;
};

struct Cam {
  std::string dev;
  int fd{-1};

  v4l2_buf_type type{V4L2_BUF_TYPE_VIDEO_CAPTURE};
  bool mplane{false};
  uint32_t width{0}, height{0}, pixfmt{0};
  uint32_t num_planes{1};

  std::vector<BufMap> bufs;           // [buf_index][plane]
  std::vector<v4l2_plane> dq_planes;  // reused for DQBUF if mplane

  uint32_t frames{0};
  uint32_t win_frames{0};
};

static bool try_get_fmt(int fd, v4l2_buf_type t, v4l2_format& fmt) {
  std::memset(&fmt, 0, sizeof(fmt));
  fmt.type = t;
  return xioctl(fd, VIDIOC_G_FMT, &fmt) == 0;
}

static void open_init_cam(Cam& c, int reqbufs) {
  c.fd = open(c.dev.c_str(), O_RDWR | O_NONBLOCK);
  if (c.fd < 0) die(("open " + c.dev).c_str());

  v4l2_capability cap{};
  if (xioctl(c.fd, VIDIOC_QUERYCAP, &cap) < 0) die("VIDIOC_QUERYCAP");
  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    std::fprintf(stderr, "%s: not STREAMING capable\n", c.dev.c_str());
    std::exit(2);
  }

  v4l2_format fmt{};
  if (try_get_fmt(c.fd, V4L2_BUF_TYPE_VIDEO_CAPTURE, fmt)) {
    c.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    c.mplane = false;
    c.width = fmt.fmt.pix.width;
    c.height = fmt.fmt.pix.height;
    c.pixfmt = fmt.fmt.pix.pixelformat;
    c.num_planes = 1;
  } else if (try_get_fmt(c.fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, fmt)) {
    c.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    c.mplane = true;
    c.width = fmt.fmt.pix_mp.width;
    c.height = fmt.fmt.pix_mp.height;
    c.pixfmt = fmt.fmt.pix_mp.pixelformat;
    c.num_planes = fmt.fmt.pix_mp.num_planes ? fmt.fmt.pix_mp.num_planes : 1;
  } else {
    die("VIDIOC_G_FMT (capture & mplane failed)");
  }

  std::printf("%s: %s %ux%u %c%c%c%c planes=%u\n",
              c.dev.c_str(),
              c.mplane ? "MPLANE" : "CAPTURE",
              c.width, c.height,
              c.pixfmt & 0xFF, (c.pixfmt >> 8) & 0xFF, (c.pixfmt >> 16) & 0xFF, (c.pixfmt >> 24) & 0xFF,
              c.num_planes);

  // request buffers
  v4l2_requestbuffers req{};
  req.count = reqbufs;
  req.type = c.type;
  req.memory = V4L2_MEMORY_MMAP;

  if (xioctl(c.fd, VIDIOC_REQBUFS, &req) < 0) die("VIDIOC_REQBUFS");
  if (req.count < 2) {
    std::fprintf(stderr, "%s: insufficient reqbufs: %u\n", c.dev.c_str(), req.count);
    std::exit(3);
  }

  c.bufs.resize(req.count);
  if (c.mplane) c.dq_planes.resize(c.num_planes);

  // map + queue
  for (uint32_t i = 0; i < req.count; ++i) {
    v4l2_buffer b{};
    b.type = c.type;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = i;

    std::vector<v4l2_plane> planes;
    if (c.mplane) {
      planes.resize(c.num_planes);
      b.m.planes = planes.data();
      b.length = c.num_planes;
    }

    if (xioctl(c.fd, VIDIOC_QUERYBUF, &b) < 0) die("VIDIOC_QUERYBUF");

    c.bufs[i].planes.clear();

    if (!c.mplane) {
      void* ptr = mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, c.fd, b.m.offset);
      if (ptr == MAP_FAILED) die("mmap");
      c.bufs[i].planes.push_back({ptr, (size_t)b.length});
    } else {
      c.bufs[i].planes.resize(c.num_planes);
      for (uint32_t p = 0; p < c.num_planes; ++p) {
        void* ptr = mmap(nullptr, planes[p].length, PROT_READ | PROT_WRITE,
                         MAP_SHARED, c.fd, planes[p].m.mem_offset);
        if (ptr == MAP_FAILED) die("mmap plane");
        c.bufs[i].planes[p] = {ptr, (size_t)planes[p].length};
      }
    }

    // queue buffer
    v4l2_buffer qb{};
    qb.type = c.type;
    qb.memory = V4L2_MEMORY_MMAP;
    qb.index = i;

    std::vector<v4l2_plane> qplanes;
    if (c.mplane) {
      qplanes.resize(c.num_planes);
      for (uint32_t p = 0; p < c.num_planes; ++p) qplanes[p].length = c.bufs[i].planes[p].len;
      qb.m.planes = qplanes.data();
      qb.length = c.num_planes;
    }

    if (xioctl(c.fd, VIDIOC_QBUF, &qb) < 0) die("VIDIOC_QBUF(init)");
  }
}

static void stream_on(Cam& c) {
  if (xioctl(c.fd, VIDIOC_STREAMON, &c.type) < 0) die("VIDIOC_STREAMON");
}

static void stream_off_close(Cam& c) {
  if (c.fd < 0) return;
  xioctl(c.fd, VIDIOC_STREAMOFF, &c.type);
  for (auto& bm : c.bufs) {
    for (auto& pl : bm.planes) {
      if (pl.ptr && pl.ptr != MAP_FAILED) munmap(pl.ptr, pl.len);
    }
  }
  close(c.fd);
  c.fd = -1;
}

struct FrameInfo {
  uint32_t seq{0};
  uint32_t idx{0};
  uint64_t ts_ns{0};
  uint32_t bytes{0};
  uint32_t flags{0};
};

static bool dq_one(Cam& c, FrameInfo& out) {
  v4l2_buffer b{};
  b.type = c.type;
  b.memory = V4L2_MEMORY_MMAP;

  if (c.mplane) {
    std::fill(c.dq_planes.begin(), c.dq_planes.end(), v4l2_plane{});
    b.m.planes = c.dq_planes.data();
    b.length = c.num_planes;
  }

  if (xioctl(c.fd, VIDIOC_DQBUF, &b) < 0) {
    if (errno == EAGAIN) return false;
    die("VIDIOC_DQBUF");
  }

  uint64_t ts_ns = (uint64_t)b.timestamp.tv_sec * 1000000000ULL +
                   (uint64_t)b.timestamp.tv_usec * 1000ULL;

  uint32_t bytes = 0;
  if (!c.mplane) bytes = b.bytesused;
  else {
    for (uint32_t p = 0; p < c.num_planes; ++p) bytes += c.dq_planes[p].bytesused;
  }

  out.seq = b.sequence;
  out.idx = b.index;
  out.ts_ns = ts_ns;
  out.bytes = bytes;
  out.flags = b.flags;
  return true;
}

static void q_index(Cam& c, uint32_t idx) {
  v4l2_buffer b{};
  b.type = c.type;
  b.memory = V4L2_MEMORY_MMAP;
  b.index = idx;

  std::vector<v4l2_plane> planes;
  if (c.mplane) {
    planes.resize(c.num_planes);
    for (uint32_t p = 0; p < c.num_planes; ++p) planes[p].length = c.bufs[idx].planes[p].len;
    b.m.planes = planes.data();
    b.length = c.num_planes;
  }

  if (xioctl(c.fd, VIDIOC_QBUF, &b) < 0) die("VIDIOC_QBUF");
}

struct Group {
  std::vector<uint64_t> ts;
  uint32_t mask{0};
  Group() = default;
  explicit Group(int n) : ts(n, 0), mask(0) {}
};

static void usage(const char* argv0) {
  std::fprintf(stderr,
    "Usage:\n"
    "  %s /dev/video0 /dev/video1 /dev/video2 /dev/video3 [--count N] [--quiet]\n"
    "If no devices given, defaults to /dev/video0..3\n"
    "  --count N : frames PER camera (default 300)\n"
    "  --quiet   : don't print every frame (only FPS + delta when available)\n",
    argv0);
}

int main(int argc, char** argv) {
  signal(SIGINT, on_sigint);

  std::vector<std::string> devs;
  int count = 300;
  bool quiet = false;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a.rfind("/dev/video", 0) == 0) devs.push_back(a);
    else if (a == "--count" && i + 1 < argc) count = std::atoi(argv[++i]);
    else if (a == "--quiet") quiet = true;
    else if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
    else { std::fprintf(stderr, "Unknown arg: %s\n", a.c_str()); usage(argv[0]); return 2; }
  }

  if (devs.empty()) devs = {"/dev/video0", "/dev/video1", "/dev/video2", "/dev/video3"};
  if (devs.size() != 4) {
    std::fprintf(stderr, "This sample expects exactly 4 devices for now. Got %zu\n", devs.size());
    return 2;
  }

  const int N = (int)devs.size();
  std::vector<Cam> cams(N);
  for (int i = 0; i < N; ++i) cams[i].dev = devs[i];

  const int REQ_BUFS = 4;

  // init all
  for (int i = 0; i < N; ++i) open_init_cam(cams[i], REQ_BUFS);

  // streamon all (as close as possible)
  for (int i = 0; i < N; ++i) stream_on(cams[i]);

  std::vector<pollfd> pfds(N);
  for (int i = 0; i < N; ++i) {
    pfds[i].fd = cams[i].fd;
    pfds[i].events = POLLIN;
  }

  const uint32_t all_mask = (1u << N) - 1u;
  std::unordered_map<uint32_t, Group> groups;

  uint64_t t_report0 = mono_ns();

  std::printf("Capturing 4 cams. Ctrl-C to stop. target=%d frames/cam\n", count);
  std::printf("Tip: if you set trig_mode=2 but no trigger, you'll see timeouts.\n");

  while (g_run) {
    // stop when all cams reached target
    bool done = true;
    for (int i = 0; i < N; ++i) if ((int)cams[i].frames < count) done = false;
    if (done) break;

    int r = poll(pfds.data(), pfds.size(), 2000);
    if (r < 0) {
      if (errno == EINTR) continue;
      die("poll");
    }
    if (r == 0) {
      std::fprintf(stderr, "poll timeout (no frames) - check trigger mode / signal\n");
      continue;
    }

    for (int i = 0; i < N; ++i) {
      if (!(pfds[i].revents & POLLIN)) continue;

      // drain available buffers
      while (g_run) {
        FrameInfo fi{};
        if (!dq_one(cams[i], fi)) break;

        cams[i].frames++;
        cams[i].win_frames++;

        if (!quiet) {
          std::printf("cam%d seq=%u ts=%llu ns \n",
                      i, fi.seq, (unsigned long long)fi.ts_ns);
        }

        // delta by sequence (works best when trigger-synced)
        auto it = groups.find(fi.seq);
        if (it == groups.end()) it = groups.emplace(fi.seq, Group(N)).first;
        it->second.ts[i] = fi.ts_ns;
        it->second.mask |= (1u << i);

        if (it->second.mask == all_mask) {
          auto& v = it->second.ts;
          auto [mn, mx] = std::minmax_element(v.begin(), v.end());
          uint64_t delta = *mx - *mn;
          std::printf(">>> seq=%u ALL cams: delta=%llu ns\n",
                      fi.seq, (unsigned long long)delta);
          groups.erase(it);
        } else {
          if (groups.size() > 4000) groups.clear();
        }

        // requeue
        q_index(cams[i], fi.idx);
      }
    }

    // 1초마다 FPS 리포트
    uint64_t t_now = mono_ns();
    if (t_now - t_report0 >= 1000000000ULL) {
      double dt = (t_now - t_report0) / 1e9;
      for (int i = 0; i < N; ++i) {
        double fps = cams[i].win_frames / dt;
        std::printf("[FPS] cam%d: %.2f (total=%u)\n", i, fps, cams[i].frames);
        cams[i].win_frames = 0;
      }
      t_report0 = t_now;
    }
  }

  for (auto& c : cams) stream_off_close(c);
  std::printf("Done.\n");
  return 0;
}