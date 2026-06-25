#pragma once

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <opencv2/opencv.hpp>

// =============================================================================
// 纯巡线接口：不包含跳跃/启动逻辑，只做摄像头PID巡线
// 返回:
//   0 → 继续巡线中
//   1 → 雷达/跳变触发退出 (调用方自行决定去 case1/case2)
// =============================================================================
int pureLineFollow(unitree::robot::go2::SportClient &sc,
                   const cv::Mat &undist,
                   double lx, double ly, double dyaw,
                   int fcount,
                   bool is_second_pass);