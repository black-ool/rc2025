#pragma once

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <opencv2/opencv.hpp>

// =============================================================================
// case0：前进 0.2m → 起跳 → 稳定 → 巡线 → 触发切换
// 与 rc2025.cpp Flag_Task=0 完全一致
// 返回值:
//   0 → 继续巡线
//   1 → 切换到 case1 (避障)
//   2 → 切换到 case2 (ArUco, 仅第二段巡线)
// =============================================================================
int case0_tick(unitree::robot::go2::SportClient &sc,
               const cv::Mat &undist,
               const unitree_go::msg::dds_::SportModeState_ &state,
               int fcount);

void case0_reset_statics();