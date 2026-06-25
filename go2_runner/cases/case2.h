#pragma once

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

// =============================================================================
// case2：ArUco 检测 → 左转 90° (与 rc2025.cpp Flag_Task=2 一致)
// 返回 true 表示转弯完成，切换到 case3
// =============================================================================
bool case2_tick(unitree::robot::go2::SportClient &sc);

void case2_reset();