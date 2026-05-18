#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <iostream>
#include <cstring>
#include <nvbufsurface.h>
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
  GstBuffer* buffer = gst_sample_get_buffer(sample);
  GstMapInfo inmap = GST_MAP_INFO_INIT;
  if (!gst_buffer_map(buffer, &inmap, GST_MAP_READ)) {
    std::cerr << "gst_buffer_map failed" << std::endl;
    return 3;
  }
  auto* surf = reinterpret_cast<NvBufSurface*>(inmap.data);
  std::cout << "surf=" << surf << " numFilled=" << surf->numFilled << " batchSize=" << surf->batchSize << std::endl;
  int rc = NvBufSurfaceMap(surf, -1, -1, NVBUF_MAP_READ);
  std::cout << "NvBufSurfaceMap rc=" << rc << std::endl;
  if (rc == 0) {
    rc = NvBufSurfaceSyncForCpu(surf, -1, -1);
    std::cout << "NvBufSurfaceSyncForCpu rc=" << rc << std::endl;
    auto &sl = surf->surfaceList[0];
    std::cout << "w=" << sl.width << " h=" << sl.height << " pitch=" << sl.pitch
              << " colorFormat=" << sl.colorFormat << " mapped0=" << sl.mappedAddr.addr[0]
              << " dataSize=" << sl.dataSize << std::endl;
    if (sl.mappedAddr.addr[0]) {
      unsigned char* p = static_cast<unsigned char*>(sl.mappedAddr.addr[0]);
      std::cout << "bytes=" << int(p[0]) << "," << int(p[1]) << "," << int(p[2]) << "," << int(p[3]) << std::endl;
    }
    NvBufSurfaceUnMap(surf, -1, -1);
  }
  gst_buffer_unmap(buffer, &inmap);
  gst_sample_unref(sample);
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(appsink);
  gst_object_unref(pipeline);
  return 0;
}
