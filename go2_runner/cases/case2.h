#pragma once

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <opencv2/opencv.hpp>

// =============================================================================
// case2：巡线准备过台阶（含占位停车逻辑）
// =============================================================================
void case2_tick(unitree::robot::go2::SportClient &sc,
                const cv::Mat &undist);