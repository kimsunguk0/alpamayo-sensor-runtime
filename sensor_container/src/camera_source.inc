class CameraSource {
 public:
  CameraSource(CameraConfig cfg, UtcMapper* utc_mapper)
      : cfg_(std::move(cfg)), utc_mapper_(utc_mapper), ring_(16) {}

  ~CameraSource() { stop(); }

  bool start() {
    if (cfg_.backend == "v4l2") {
      if (!start_v4l2_mmap()) {
        has_error_.store(true);
        if (last_error_.empty()) last_error_ = "failed to start V4L2 MMAP capture";
        log_error("camera_" + cfg_.name + " failed to start: " + last_error_);
        return false;
      }
      start_master_ns_ = now_master_ns();
      log_info("camera_" + cfg_.name + " started on " + cfg_.device + " backend=" + cfg_.backend);
      return true;
    }

    std::ostringstream pipe;
    if (cfg_.backend == "v4l2_preproc") {
      pipe << "v4l2src device=" << cfg_.device << " io-mode=2 do-timestamp=true ! "
           << "video/x-raw,format=" << cfg_.pixel_format
           << ",width=" << cfg_.source_width
           << ",height=" << cfg_.source_height
           << ",framerate=" << cfg_.source_fps << "/1 ! "
           << "queue max-size-buffers=4 leaky=downstream ! "
           << "videorate drop-only=true ! "
           << "video/x-raw,framerate=" << cfg_.planner_ingest_fps << "/1 ! "
           << "queue max-size-buffers=2 leaky=downstream ! "
           << "nvvidconv compute-hw=2 interpolation-method=4 ! "
           << "video/x-raw,format=RGBA,width=" << cfg_.planner_width
           << ",height=" << cfg_.planner_height << " ! "
           << "appsink name=framesink emit-signals=true sync=false max-buffers=2 drop=true";
    } else if (cfg_.backend == "v4l2_nvmm") {
      pipe << "v4l2src device=" << cfg_.device << " io-mode=2 do-timestamp=true ! "
           << "video/x-raw,format=" << cfg_.pixel_format
           << ",width=" << cfg_.source_width
           << ",height=" << cfg_.source_height
           << ",framerate=" << cfg_.source_fps << "/1 ! "
           << "queue max-size-buffers=4 leaky=downstream ! "
           << "nvvideoconvert compute-hw=2 nvbuf-memory-type=4 interpolation-method=4 ! "
           << "video/x-raw(memory:NVMM),format=RGBA,width=" << cfg_.planner_width
           << ",height=" << cfg_.planner_height << " ! "
           << "appsink name=framesink emit-signals=true sync=false max-buffers=2 drop=true";
    } else if (cfg_.backend == "v4l2_cpu") {
      pipe << "v4l2src device=" << cfg_.device << " io-mode=2 do-timestamp=true ! "
           << "video/x-raw,format=" << cfg_.pixel_format
           << ",width=" << cfg_.source_width
           << ",height=" << cfg_.source_height
           << ",framerate=" << cfg_.source_fps << "/1 ! "
           << "queue max-size-buffers=4 leaky=downstream ! "
           << "videorate drop-only=true ! "
           << "video/x-raw,framerate=15/1 ! "
           << "videoconvert ! "
           << "videoscale method=8 n-threads=2 ! "
           << "video/x-raw,format=RGB,width=" << cfg_.planner_width
           << ",height=" << cfg_.planner_height << " ! "
           << "appsink name=framesink emit-signals=true sync=false max-buffers=2 drop=true";
    } else if (cfg_.backend == "nvsiplsrc") {
      std::ostringstream msg;
      msg << "nvsiplsrc backend is declared in the contract but not implemented in this first skeleton for camera_" << cfg_.name;
      last_error_ = msg.str();
      has_error_.store(true);
      log_error(last_error_);
      return false;
    } else {
      last_error_ = "unsupported camera backend: " + cfg_.backend;
      has_error_.store(true);
      log_error(last_error_);
      return false;
    }

    GError* err = nullptr;
    pipeline_ = gst_parse_launch(pipe.str().c_str(), &err);
    if (!pipeline_ || err) {
      last_error_ = err ? err->message : "gst_parse_launch failed";
      if (err) g_error_free(err);
      has_error_.store(true);
      log_error("camera_" + cfg_.name + " pipeline creation failed: " + last_error_);
      return false;
    }
    bus_ = gst_element_get_bus(pipeline_);
    GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline_), "framesink");
    if (!sink) {
      last_error_ = "appsink not found";
      has_error_.store(true);
      return false;
    }
    appsink_ = GST_APP_SINK(sink);
    gst_app_sink_set_emit_signals(appsink_, TRUE);
    gst_app_sink_set_drop(appsink_, TRUE);
    gst_app_sink_set_max_buffers(appsink_, 2);
    g_signal_connect(appsink_, "new-sample", G_CALLBACK(&CameraSource::on_new_sample_static), this);

    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      last_error_ = "failed to go to PLAYING";
      has_error_.store(true);
      log_error("camera_" + cfg_.name + " failed to start");
      return false;
    }
    start_master_ns_ = now_master_ns();
    log_info("camera_" + cfg_.name + " started on " + cfg_.device + " backend=" + cfg_.backend);
    return true;
  }

  void stop() {
    stop_v4l2_mmap();
    if (pipeline_) {
      gst_element_send_event(pipeline_, gst_event_new_eos());
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (appsink_) {
      gst_object_unref(appsink_);
      appsink_ = nullptr;
    }
    if (bus_) {
      gst_object_unref(bus_);
      bus_ = nullptr;
    }
    if (pipeline_) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

  void poll_bus() {
    if (cfg_.backend == "v4l2") return;
    if (!bus_) return;
    while (GstMessage* msg = gst_bus_pop(bus_)) {
      switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
          GError* err = nullptr;
          gchar* dbg = nullptr;
          gst_message_parse_error(msg, &err, &dbg);
          std::ostringstream os;
          os << "camera_" << cfg_.name << " gst error: " << (err ? err->message : "unknown");
          if (dbg && *dbg) os << " debug=" << dbg;
          last_error_ = os.str();
          has_error_.store(true);
          log_error(last_error_);
          if (err) g_error_free(err);
          if (dbg) g_free(dbg);
          break;
        }
        case GST_MESSAGE_EOS:
          log_warn("camera_" + cfg_.name + " received EOS");
          break;
        default:
          break;
      }
      gst_message_unref(msg);
    }
  }

  bool has_error() const { return has_error_.load(); }
  std::string error_message() const { return last_error_; }
  const FrameRing& ring() const { return ring_; }
  uint64_t latest_utc_us() const { return ring_.latest_utc_us(); }
  uint64_t frame_count() const { return frame_count_.load(); }
  double fps_estimate() const {
    uint64_t elapsed_ns = now_master_ns() - start_master_ns_;
    if (elapsed_ns == 0) return 0.0;
    return static_cast<double>(frame_count_.load()) / (static_cast<double>(elapsed_ns) * 1e-9);
  }
  const CameraConfig& config() const { return cfg_; }
  std::string status_string() const {
    std::ostringstream os;
    os << cfg_.name << "=" << frame_count() << "frames(" << std::fixed << std::setprecision(2) << fps_estimate() << "fps)";
    return os.str();
  }

 private:
  struct MMapBuf {
    void* start = nullptr;
    size_t length = 0;
  };

  struct HeldBuffer {
    uint32_t index = 0;
  };

  struct RequeueContext {
    std::atomic<bool> active{false};
    int fd = -1;
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int num_planes = 1;

    void requeue(uint32_t index) {
      if (!active.load() || fd < 0) return;
      v4l2_buffer buf{};
      v4l2_plane planes[VIDEO_MAX_PLANES]{};
      buf.type = type;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = index;
      if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        buf.m.planes = planes;
        buf.length = num_planes;
      }
      xioctl(fd, VIDIOC_QBUF, &buf);
    }
  };

  static uint32_t v4l2_fourcc_from_string(const std::string& format) {
    const std::string lower = to_lower_copy(format);
    if (lower == "uyvy") return V4L2_PIX_FMT_UYVY;
    if (lower == "yuyv" || lower == "yuy2") return V4L2_PIX_FMT_YUYV;
    if (lower == "rgb") return V4L2_PIX_FMT_RGB24;
    return V4L2_PIX_FMT_UYVY;
  }

  static int default_stride_for(uint32_t pixfmt, uint32_t width) {
    switch (pixfmt) {
      case V4L2_PIX_FMT_UYVY:
      case V4L2_PIX_FMT_YUYV:
        return static_cast<int>(width) * 2;
      case V4L2_PIX_FMT_RGB24:
        return static_cast<int>(width) * 3;
      default:
        return static_cast<int>(width) * 2;
    }
  }

  static ImagePixelFormat image_pixel_format_for_v4l2(uint32_t pixfmt) {
    switch (pixfmt) {
      case V4L2_PIX_FMT_UYVY:
      case V4L2_PIX_FMT_YUYV:
        return ImagePixelFormat::kUyvy422;
      case V4L2_PIX_FMT_RGB24:
        return ImagePixelFormat::kRgb24;
      default:
        return ImagePixelFormat::kUnknown;
    }
  }

  bool configure_v4l2_type(v4l2_buf_type candidate) {
    v4l2_format fmt{};
    fmt.type = candidate;
    const uint32_t want_fourcc = v4l2_fourcc_from_string(cfg_.pixel_format);
    if (candidate == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
      fmt.fmt.pix_mp.width = cfg_.source_width;
      fmt.fmt.pix_mp.height = cfg_.source_height;
      fmt.fmt.pix_mp.pixelformat = want_fourcc;
      fmt.fmt.pix_mp.num_planes = 1;
      fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    } else {
      fmt.fmt.pix.width = cfg_.source_width;
      fmt.fmt.pix.height = cfg_.source_height;
      fmt.fmt.pix.pixelformat = want_fourcc;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
    }
    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) return false;

    std::memset(&fmt, 0, sizeof(fmt));
    fmt.type = candidate;
    if (xioctl(fd_, VIDIOC_G_FMT, &fmt) < 0) return false;

    type_ = candidate;
    if (candidate == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
      width_ = fmt.fmt.pix_mp.width;
      height_ = fmt.fmt.pix_mp.height;
      pixfmt_v4l2_ = fmt.fmt.pix_mp.pixelformat;
      num_planes_ = static_cast<int>(fmt.fmt.pix_mp.num_planes);
      if (num_planes_ <= 0) num_planes_ = 1;
      stride_ = fmt.fmt.pix_mp.plane_fmt[0].bytesperline ? static_cast<int>(fmt.fmt.pix_mp.plane_fmt[0].bytesperline)
                                                         : default_stride_for(pixfmt_v4l2_, width_);
    } else {
      width_ = fmt.fmt.pix.width;
      height_ = fmt.fmt.pix.height;
      pixfmt_v4l2_ = fmt.fmt.pix.pixelformat;
      num_planes_ = 1;
      stride_ = fmt.fmt.pix.bytesperline ? static_cast<int>(fmt.fmt.pix.bytesperline)
                                         : default_stride_for(pixfmt_v4l2_, width_);
    }

    v4l2_streamparm parm{};
    parm.type = candidate;
    if (candidate == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
      parm.parm.capture.timeperframe.numerator = 1;
      parm.parm.capture.timeperframe.denominator = std::max(1, cfg_.source_fps);
    } else {
      parm.parm.capture.timeperframe.numerator = 1;
      parm.parm.capture.timeperframe.denominator = std::max(1, cfg_.source_fps);
    }
    xioctl(fd_, VIDIOC_S_PARM, &parm);
    return true;
  }

  bool init_v4l2_mmap(uint32_t count) {
    v4l2_requestbuffers req{};
    req.count = count;
    req.type = type_;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
      last_error_ = "VIDIOC_REQBUFS failed for " + cfg_.device + " errno=" + std::to_string(errno);
      return false;
    }
    if (req.count < 4) {
      last_error_ = "insufficient V4L2 buffers: " + std::to_string(req.count);
      return false;
    }

    bufs_.resize(req.count);
    for (uint32_t i = 0; i < req.count; ++i) {
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
        last_error_ = "VIDIOC_QUERYBUF failed for " + cfg_.device + " errno=" + std::to_string(errno);
        return false;
      }

      size_t len = 0;
      off_t off = 0;
      if (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        len = buf.m.planes[0].length;
        off = static_cast<off_t>(buf.m.planes[0].m.mem_offset);
      } else {
        len = buf.length;
        off = static_cast<off_t>(buf.m.offset);
      }
      void* start = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, off);
      if (start == MAP_FAILED) {
        last_error_ = "mmap failed for " + cfg_.device + " errno=" + std::to_string(errno);
        return false;
      }
      bufs_[i].start = start;
      bufs_[i].length = len;

      if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
        last_error_ = "VIDIOC_QBUF(init) failed for " + cfg_.device + " errno=" + std::to_string(errno);
        return false;
      }
    }
    return true;
  }

  bool stream_on_v4l2() {
    if (xioctl(fd_, VIDIOC_STREAMON, &type_) < 0) {
      last_error_ = "VIDIOC_STREAMON failed for " + cfg_.device + " errno=" + std::to_string(errno);
      return false;
    }
    return true;
  }

  void stream_off_v4l2() {
    if (fd_ >= 0) xioctl(fd_, VIDIOC_STREAMOFF, &type_);
  }

  bool start_v4l2_mmap() {
    fd_ = ::open(cfg_.device.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd_ < 0) {
      last_error_ = "open failed for " + cfg_.device + " errno=" + std::to_string(errno);
      return false;
    }

    if (!configure_v4l2_type(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) &&
        !configure_v4l2_type(V4L2_BUF_TYPE_VIDEO_CAPTURE)) {
      last_error_ = "failed to configure V4L2 capture type for " + cfg_.device;
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    if (image_pixel_format_for_v4l2(pixfmt_v4l2_) == ImagePixelFormat::kUnknown) {
      last_error_ = "unsupported V4L2 pixel format for " + cfg_.device;
      ::close(fd_);
      fd_ = -1;
      return false;
    }

    if (!init_v4l2_mmap(20)) {
      stop_v4l2_mmap();
      return false;
    }
    if (!stream_on_v4l2()) {
      stop_v4l2_mmap();
      return false;
    }

    requeue_ctx_ = std::make_shared<RequeueContext>();
    requeue_ctx_->fd = fd_;
    requeue_ctx_->type = type_;
    requeue_ctx_->num_planes = num_planes_;
    requeue_ctx_->active.store(true);

    capture_running_.store(true);
    capture_thread_ = std::thread([this]() { capture_loop(); });
    return true;
  }

  void stop_v4l2_mmap() {
    capture_running_.store(false);
    if (capture_thread_.joinable()) capture_thread_.join();
    if (requeue_ctx_) requeue_ctx_->active.store(false);
    ring_.clear();
    if (fd_ >= 0) {
      stream_off_v4l2();
      for (auto& b : bufs_) {
        if (b.start && b.length) munmap(b.start, b.length);
      }
      bufs_.clear();
      ::close(fd_);
      fd_ = -1;
    }
    requeue_ctx_.reset();
  }

  void capture_loop() {
    while (capture_running_.load() && !g_stop.load()) {
      pollfd pfd{};
      pfd.fd = fd_;
      pfd.events = POLLIN | POLLPRI;
      int pr = ::poll(&pfd, 1, 200);
      if (pr < 0) {
        if (errno == EINTR) continue;
        last_error_ = "poll failed for " + cfg_.device + " errno=" + std::to_string(errno);
        has_error_.store(true);
        return;
      }
      if (pr == 0 || !(pfd.revents & (POLLIN | POLLPRI))) continue;

      while (capture_running_.load()) {
        v4l2_buffer buf{};
        v4l2_plane planes[VIDEO_MAX_PLANES]{};
        buf.type = type_;
        buf.memory = V4L2_MEMORY_MMAP;
        if (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
          buf.m.planes = planes;
          buf.length = num_planes_;
        }
        if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
          if (errno == EAGAIN) break;
          last_error_ = "VIDIOC_DQBUF failed for " + cfg_.device + " errno=" + std::to_string(errno);
          has_error_.store(true);
          return;
        }

        const size_t bytes_used = std::min<size_t>(
            (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) ? buf.m.planes[0].bytesused : buf.bytesused,
            bufs_[buf.index].length);
        uint64_t master_ns = now_master_ns();
        uint64_t utc_ns = 0;
        if (!utc_mapper_->map(master_ns, utc_ns)) utc_ns = utc_mapper_->fallback_from_anchor(master_ns);

        auto holder = std::shared_ptr<HeldBuffer>(
            new HeldBuffer{buf.index},
            [ctx = requeue_ctx_](HeldBuffer* held) {
              if (ctx) ctx->requeue(held->index);
              delete held;
            });

        ImageFrame image_frame;
        image_frame.seq = frame_count_.fetch_add(1) + 1;
        image_frame.master_ns = master_ns;
        image_frame.utc_us = utc_ns / 1000ULL;
        image_frame.width = static_cast<int>(width_);
        image_frame.height = static_cast<int>(height_);
        image_frame.stride = stride_;
        image_frame.format = image_pixel_format_for_v4l2(pixfmt_v4l2_);
        image_frame.hold_ref = holder;
        image_frame.data = static_cast<const uint8_t*>(bufs_[buf.index].start);
        image_frame.data_size = bytes_used;
        ring_.push(std::move(image_frame));
      }
    }
  }

  static GstFlowReturn on_new_sample_static(GstAppSink* sink, gpointer user_data) {
    return static_cast<CameraSource*>(user_data)->on_new_sample(sink);
  }

  static bool caps_use_nvmm(GstCaps* caps) {
    if (!caps || gst_caps_is_empty(caps)) return false;
    GstCapsFeatures* features = gst_caps_get_features(caps, 0);
    return features && gst_caps_features_contains(features, "memory:NVMM");
  }

  bool copy_rgb_from_nvmm(GstBuffer* buffer, int& width, int& height, std::shared_ptr<std::vector<uint8_t>>& rgb) {
    GstMapInfo inmap = GST_MAP_INFO_INIT;
    if (!gst_buffer_map(buffer, &inmap, GST_MAP_READ)) return false;

    bool ok = false;
    auto* surf = reinterpret_cast<NvBufSurface*>(inmap.data);
    if (surf && surf->numFilled > 0 && NvBufSurfaceMap(surf, 0, 0, NVBUF_MAP_READ) == 0) {
      auto& sl = surf->surfaceList[0];
      if (NvBufSurfaceSyncForCpu(surf, 0, 0) == 0 && sl.mappedAddr.addr[0] && sl.pitch > 0) {
        width = static_cast<int>(sl.width);
        height = static_cast<int>(sl.height);
        const int pitch = static_cast<int>(sl.pitch);
        const uint8_t* src = static_cast<const uint8_t*>(sl.mappedAddr.addr[0]);
        rgb = std::make_shared<std::vector<uint8_t>>(static_cast<size_t>(width) * height * 3);
        for (int y = 0; y < height; ++y) {
          const uint8_t* row = src + static_cast<size_t>(y) * pitch;
          uint8_t* dst = rgb->data() + static_cast<size_t>(y) * width * 3;
          for (int x = 0; x < width; ++x) {
            const uint8_t* px = row + static_cast<size_t>(x) * 4;
            uint8_t* out = dst + static_cast<size_t>(x) * 3;
            out[0] = px[0];
            out[1] = px[1];
            out[2] = px[2];
          }
        }
        ok = true;
      }
      NvBufSurfaceUnMap(surf, 0, 0);
    }

    gst_buffer_unmap(buffer, &inmap);
    return ok;
  }

  bool copy_rgb_from_system_memory(GstCaps* caps, GstBuffer* buffer, int& width, int& height,
                                   std::shared_ptr<std::vector<uint8_t>>& rgb) {
    GstVideoInfo info;
    if (!gst_video_info_from_caps(&info, caps)) return false;

    GstVideoFrame frame;
    if (!gst_video_frame_map(&frame, &info, buffer, GST_MAP_READ)) return false;

    width = GST_VIDEO_FRAME_WIDTH(&frame);
    height = GST_VIDEO_FRAME_HEIGHT(&frame);
    const int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    const uint8_t* src = static_cast<const uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
    rgb = std::make_shared<std::vector<uint8_t>>(static_cast<size_t>(width) * height * 3);
    GstVideoFormat fmt = GST_VIDEO_INFO_FORMAT(&info);

    for (int y = 0; y < height; ++y) {
      const uint8_t* row = src + static_cast<size_t>(y) * stride;
      uint8_t* dst = rgb->data() + static_cast<size_t>(y) * width * 3;
      if (fmt == GST_VIDEO_FORMAT_RGB) {
        std::memcpy(dst, row, static_cast<size_t>(width) * 3);
      } else if (fmt == GST_VIDEO_FORMAT_RGBA || fmt == GST_VIDEO_FORMAT_RGBx) {
        for (int x = 0; x < width; ++x) {
          const uint8_t* px = row + static_cast<size_t>(x) * 4;
          uint8_t* out = dst + static_cast<size_t>(x) * 3;
          out[0] = px[0];
          out[1] = px[1];
          out[2] = px[2];
        }
      } else if (fmt == GST_VIDEO_FORMAT_BGR) {
        for (int x = 0; x < width; ++x) {
          const uint8_t* px = row + static_cast<size_t>(x) * 3;
          uint8_t* out = dst + static_cast<size_t>(x) * 3;
          out[0] = px[2];
          out[1] = px[1];
          out[2] = px[0];
        }
      } else if (fmt == GST_VIDEO_FORMAT_BGRA || fmt == GST_VIDEO_FORMAT_BGRx) {
        for (int x = 0; x < width; ++x) {
          const uint8_t* px = row + static_cast<size_t>(x) * 4;
          uint8_t* out = dst + static_cast<size_t>(x) * 3;
          out[0] = px[2];
          out[1] = px[1];
          out[2] = px[0];
        }
      } else {
        gst_video_frame_unmap(&frame);
        return false;
      }
    }

    gst_video_frame_unmap(&frame);
    return true;
  }

  bool copy_frame_from_system_memory(GstCaps* caps, GstBuffer* buffer, int& width, int& height, int& stride,
                                     ImagePixelFormat& format,
                                     std::shared_ptr<std::vector<uint8_t>>& bytes) {
    GstVideoInfo info;
    if (!gst_video_info_from_caps(&info, caps)) return false;

    GstVideoFrame frame;
    if (!gst_video_frame_map(&frame, &info, buffer, GST_MAP_READ)) return false;

    width = GST_VIDEO_FRAME_WIDTH(&frame);
    height = GST_VIDEO_FRAME_HEIGHT(&frame);
    stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    const uint8_t* src = static_cast<const uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
    GstVideoFormat fmt = GST_VIDEO_INFO_FORMAT(&info);
    int bytes_per_pixel = 0;
    switch (fmt) {
      case GST_VIDEO_FORMAT_UYVY:
        format = ImagePixelFormat::kUyvy422;
        bytes_per_pixel = 2;
        break;
      case GST_VIDEO_FORMAT_RGB:
        format = ImagePixelFormat::kRgb24;
        bytes_per_pixel = 3;
        break;
      case GST_VIDEO_FORMAT_RGBA:
      case GST_VIDEO_FORMAT_RGBx:
        format = ImagePixelFormat::kRgba32;
        bytes_per_pixel = 4;
        break;
      default:
        gst_video_frame_unmap(&frame);
        return false;
    }

    bytes = std::make_shared<std::vector<uint8_t>>(static_cast<size_t>(width) * height * bytes_per_pixel);
    for (int y = 0; y < height; ++y) {
      std::memcpy(bytes->data() + static_cast<size_t>(y) * width * bytes_per_pixel,
                  src + static_cast<size_t>(y) * stride,
                  static_cast<size_t>(width) * bytes_per_pixel);
    }
    stride = width * bytes_per_pixel;
    gst_video_frame_unmap(&frame);
    return true;
  }

  GstFlowReturn on_new_sample(GstAppSink* sink) {
    GstSample* sample = gst_app_sink_pull_sample(sink);
    if (!sample) return GST_FLOW_OK;
    GstCaps* caps = gst_sample_get_caps(sample);
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    if (!caps || !buffer) {
      gst_sample_unref(sample);
      return GST_FLOW_OK;
    }

    int width = 0;
    int height = 0;
    int stride = 0;
    ImagePixelFormat format = ImagePixelFormat::kUnknown;
    std::shared_ptr<std::vector<uint8_t>> bytes;
    bool mapped = false;
    if (caps_use_nvmm(caps)) {
      std::shared_ptr<std::vector<uint8_t>> rgb;
      mapped = copy_rgb_from_nvmm(buffer, width, height, rgb);
      if (mapped) {
        stride = width * 3;
        format = ImagePixelFormat::kRgb24;
        bytes = std::move(rgb);
      }
    } else {
      mapped = copy_frame_from_system_memory(caps, buffer, width, height, stride, format, bytes);
    }

    if (!mapped || !bytes) {
      gst_sample_unref(sample);
      return GST_FLOW_OK;
    }

    uint64_t master_ns = now_master_ns();
    uint64_t utc_ns = 0;
    if (!utc_mapper_->map(master_ns, utc_ns)) utc_ns = utc_mapper_->fallback_from_anchor(master_ns);

    ImageFrame image_frame;
    image_frame.seq = frame_count_.fetch_add(1) + 1;
    image_frame.master_ns = master_ns;
    image_frame.utc_us = utc_ns / 1000ULL;
    image_frame.width = width;
    image_frame.height = height;
    image_frame.stride = stride;
    image_frame.format = format;
    image_frame.bytes = std::move(bytes);
    image_frame.hold_ref = image_frame.bytes;
    image_frame.data = image_frame.bytes ? image_frame.bytes->data() : nullptr;
    image_frame.data_size = image_frame.bytes ? image_frame.bytes->size() : 0;
    ring_.push(std::move(image_frame));

    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  CameraConfig cfg_;
  UtcMapper* utc_mapper_;
  int fd_{-1};
  v4l2_buf_type type_{V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE};
  uint32_t width_{0};
  uint32_t height_{0};
  uint32_t pixfmt_v4l2_{0};
  int num_planes_{1};
  int stride_{0};
  GstElement* pipeline_{nullptr};
  GstBus* bus_{nullptr};
  GstAppSink* appsink_{nullptr};
  std::vector<MMapBuf> bufs_;
  std::shared_ptr<RequeueContext> requeue_ctx_;
  std::atomic<bool> capture_running_{false};
  std::thread capture_thread_;
  FrameRing ring_;
  std::atomic<uint64_t> frame_count_{0};
  std::atomic<bool> has_error_{false};
  std::string last_error_;
  uint64_t start_master_ns_{0};
};
