#include "app_runtime.h"
#include "params.h"
#include "globals.h"
#include "callbacks.h"

#include <unitree/robot/channel/channel_factory.hpp>

#include <string>

using namespace unitree::robot;

bool initAppRuntime(AppRuntime &rt, const char *eth_if)
{
    rt.sc.SetTimeout(10.0f);
    rt.sc.Init();
    rt.sub_range.InitChannel(rangeCB);
    rt.sub_state.InitChannel(rt.stateCB);
    rt.sc.BalanceStand();
    px0 = px;
    py0 = py;
    yaw0 = yaw;
    const std::string gst_front =
        std::string("udpsrc address=230.1.1.1 port=1720 multicast-iface=") + eth_if +
        " ! application/x-rtp, media=video, encoding-name=H264 "
        "! rtph264depay ! h264parse ! avdec_h264 ! videoconvert "
        "! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1";
    return rt.cap.open(gst_front, cv::CAP_GSTREAMER);
}