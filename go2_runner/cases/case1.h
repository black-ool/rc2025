#pragma once

#include <unitree/robot/go2/sport/sport_client.hpp>

// =============================================================================
// case1：S型走廊避障 (与 rc2025.cpp Flag_Task=1 完全一致)
// 返回 true 表示已完成 S 型序列，回到巡线 (Flag_Task=0)
// =============================================================================
bool case1_tick(unitree::robot::go2::SportClient &sc,
                int fcount,
                double lx,
                double ly,
                double yaw_now);

void case1_reset_statics();
