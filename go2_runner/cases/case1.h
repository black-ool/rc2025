#pragma once

#include <unitree/robot/go2/sport/sport_client.hpp>

// =============================================================================
// case1：迷宫 —— 两次 180° 掉头 + 一次左转 90° 弧线
// 返回 true 表示已触发切换到 case2
// =============================================================================
bool case1_tick(unitree::robot::go2::SportClient &sc);