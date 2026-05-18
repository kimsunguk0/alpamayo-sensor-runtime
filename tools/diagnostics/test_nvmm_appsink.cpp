#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <iostream>
#include <cstring>
int main(int argc, char** argv) {
  gst_init(&argc, &argv);
  const char* pipe =
    "v4l2src device=/dev/video2 io-mode=2 do-timestamp=true ! "
    "video/x-raw,format=UYVY,width=3840,height=2160,framerate=30/1 ! "
    "nvvideoconvert compute-hw=2 interpolation-method=4 ! "
    "video/x-raw(memory:NVMM),format=RGBA,width=576,height=320 ! "
    "appsink name=s emit-signals=false sync=false max-buffers=1 drop=true";
  GError* err = nullptr;
  GstElement* pipeline = gst_parse_launch(pipe, &err);
  if (!pipeline || err) {
    std::cerr << "parse fail: " << (err ? err->message : "unknown") << std::endl;
    return 1;
  }
  GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "s");
  GstAppSink* appsink = GST_APP_SINK(sink);
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  GstSample* sample = gst_app_sink_try_pull_sample(appsink, 5 * GST_SECOND);
  if (!sample) {
    std::cerr << "no sample" << std::endl;
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(appsink);
    gst_object_unref(pipeline);
    return 2;
  }
  GstCaps* caps = gst_sample_get_caps(sample);
  GstBuffer* buffer = gst_sample_get_buffer(sample);
  gchar* caps_str = gst_caps_to_string(caps);
  std::cout << "caps=" << (caps_str ? caps_str : "null") << std::endl;
  if (caps_str) g_free(caps_str);
  GstMapInfo info;
  gboolean ok = gst_buffer_map(buffer, &info, GST_MAP_READ);
  std::cout << "gst_buffer_map=" << ok << " size=" << (ok ? info.size : 0) << std::endl;
  if (ok) gst_buffer_unmap(buffer, &info);
  GstVideoInfo vinfo;
  gboolean vinfo_ok = gst_video_info_from_caps(&vinfo, caps);
  std::cout << "video_info=" << vinfo_ok << " width=" << (vinfo_ok ? GST_VIDEO_INFO_WIDTH(&vinfo) : 0)
            << " height=" << (vinfo_ok ? GST_VIDEO_INFO_HEIGHT(&vinfo) : 0) << std::endl;
  GstVideoFrame vframe;
  gboolean vf_ok = vinfo_ok && gst_video_frame_map(&vframe, &vinfo, buffer, GST_MAP_READ);
  std::cout << "video_frame_map=" << vf_ok;
  if (vf_ok) {
    std::cout << " stride0=" << GST_VIDEO_FRAME_PLANE_STRIDE(&vframe, 0)
              << " plane0=" << static_cast<void*>(GST_VIDEO_FRAME_PLANE_DATA(&vframe, 0));
    gst_video_frame_unmap(&vframe);
  }
  std::cout << std::endl;
  gst_sample_unref(sample);
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(appsink);
  gst_object_unref(pipeline);
  return 0;
}
