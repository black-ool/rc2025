#pragma once

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <opencv2/opencv.hpp>

// =============================================================================
// case0：前进 0.2m → 起跳 → 稳定 → 巡线 → 雷达触发进入避障
// 与 rc2025.cpp Flag_Task=0 完全一致
// 返回 true 表示已触发切换到 case1（避障）
// =============================================================================
bool case0_tick(unitree::robot::go2::SportClient &sc,
                const cv::Mat &undist,
                const unitree_go::msg::dds_::SportModeState_ &state,
                int fcount);

void case0_reset_statics();
