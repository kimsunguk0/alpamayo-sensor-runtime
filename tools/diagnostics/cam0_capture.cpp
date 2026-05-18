#include <linux/videodev2.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>   // 추가

static int xioctl(int fd, unsigned long req, void* arg) {
  int r;
  do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
  return r;
}

static void die(const char* msg) {
  perror(msg);
  exit(1);
}

static uint64_t mono_ns() {           // 추가
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

int main(int argc, char** argv) {
  const char* dev = (argc >= 2) ? argv[1] : "/dev/video0";
  int frame_count = (argc >= 3) ? atoi(argv[2]) : 200;

  int fd = open(dev, O_RDWR | O_NONBLOCK);
  if (fd < 0) die("open");

  struct v4l2_capability cap;
  memset(&cap, 0, sizeof(cap));
  if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) die("VIDIOC_QUERYCAP");
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf(stderr, "%s: not a VIDEO_CAPTURE device\n", dev);
    return 2;
  }

  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_G_FMT, &fmt) < 0) die("VIDIOC_G_FMT");

  uint32_t pf = fmt.fmt.pix.pixelformat;
  printf("Device %s format: %ux%u %c%c%c%c\n",
         dev, fmt.fmt.pix.width, fmt.fmt.pix.height,
         pf & 0xFF, (pf >> 8) & 0xFF, (pf >> 16) & 0xFF, (pf >> 24) & 0xFF);

  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) die("VIDIOC_REQBUFS");
  if (req.count < 2) {
    fprintf(stderr, "Insufficient buffers: %u\n", req.count);
    return 3;
  }

  void* bufs[8] = {0};
  size_t lens[8] = {0};

  for (uint32_t i = 0; i < req.count; ++i) {
    struct v4l2_buffer b;
    memset(&b, 0, sizeof(b));
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = i;

    if (xioctl(fd, VIDIOC_QUERYBUF, &b) < 0) die("VIDIOC_QUERYBUF");

    bufs[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
    if (bufs[i] == MAP_FAILED) die("mmap");
    lens[i] = b.length;

    if (xioctl(fd, VIDIOC_QBUF, &b) < 0) die("VIDIOC_QBUF");
  }

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) die("VIDIOC_STREAMON");

  struct pollfd pfd;
  memset(&pfd, 0, sizeof(pfd));
  pfd.fd = fd;
  pfd.events = POLLIN;

  // FPS 측정용 변수들 (추가)
  uint64_t t_start = mono_ns();
  uint64_t t_last  = t_start;
  uint64_t t_win0  = t_start;
  int win_frames = 0;

  int got = 0;
  while (got < frame_count) {
    int pr = poll(&pfd, 1, 2000);
    if (pr < 0) {
      if (errno == EINTR) continue;
      die("poll");
    }
    if (pr == 0) {
      fprintf(stderr, "poll timeout (no frame)\n");
      continue;
    }

    struct v4l2_buffer b;
    memset(&b, 0, sizeof(b));
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd, VIDIOC_DQBUF, &b) < 0) {
      if (errno == EAGAIN) continue;
      die("VIDIOC_DQBUF");
    }

    uint64_t ts_ns = (uint64_t)b.timestamp.tv_sec * 1000000000ULL +
                     (uint64_t)b.timestamp.tv_usec * 1000ULL;

    // FPS 계산 (추가)
    uint64_t t_now = mono_ns();
    double inst_fps = 0.0;
    uint64_t dt = t_now - t_last;
    if (got > 0 && dt > 0) inst_fps = 1e9 / (double)dt;
    t_last = t_now;

    win_frames++;

    // 1초마다 이동평균 FPS 출력
    double win_fps = -1.0;
    uint64_t win_dt = t_now - t_win0;
    if (win_dt >= 1000000000ULL) {
      win_fps = win_frames * 1e9 / (double)win_dt;
      t_win0 = t_now;
      win_frames = 0;
    }

    // 전체 평균 FPS
    double avg_fps = (got + 1) * 1e9 / (double)(t_now - t_start);

    if (win_fps > 0) {
      printf("seq=%u ts=%llu ns | avg=%.2f fps\n",
             b.sequence, (unsigned long long)ts_ns, b.bytesused, b.flags,
             inst_fps, win_fps, avg_fps);
    } else {
      printf("seq=%u ts=%llu ns | avg=%.2f fps\n",
             b.sequence, (unsigned long long)ts_ns, b.bytesused, b.flags,
             inst_fps, avg_fps);
    }

    if (xioctl(fd, VIDIOC_QBUF, &b) < 0) die("VIDIOC_QBUF");
    got++;
  }

  xioctl(fd, VIDIOC_STREAMOFF, &type);

  for (uint32_t i = 0; i < req.count; ++i) {
    if (bufs[i] && bufs[i] != MAP_FAILED) munmap(bufs[i], lens[i]);
  }
  close(fd);
  printf("Done.\n");
  return 0;
}
