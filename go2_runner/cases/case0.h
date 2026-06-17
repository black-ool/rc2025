#pragma once

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <opencv2/opencv.hpp>

// =============================================================================
// case0：前进 0.1m → 起跳 → 站直后稳定等待 → 巡线 → 检测窄过道进入 case1
// 返回 true 表示已触发切换到 case1
// =============================================================================
bool case0_tick(unitree::robot::go2::SportClient &sc,
                const cv::Mat &undist,
                const unitree_go::msg::dds_::SportModeState_ &state,
                double lx);