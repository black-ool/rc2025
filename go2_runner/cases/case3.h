#pragma once

#include <unitree/robot/go2/sport/sport_client.hpp>

// =============================================================================
// case3：Flag_Task=4~8 序列
//   4: 右转90° → 5: FreeWalk过台阶 → 6: StaticWalk前进
//   → 7: 左转90° → 8: 跳进终点区
// 与 rc2025.cpp Flag_Task=4~8 完全一致
// 返回:
//   1 → Flag_Task=9 (完成终点跳)
//   0 → 继续进行中
// =============================================================================
bool case3_tick(unitree::robot::go2::SportClient &sc,
                double lx,
                double ly,
                double dyaw);

void case3_reset();