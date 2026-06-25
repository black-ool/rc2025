#pragma once

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>

// =============================================================================
// case4：任务完成 —— 停止 + 恢复遥控器 (与 rc2025.cpp Flag_Task=9 完全一致)
// 返回 true 表示主循环应退出
// =============================================================================
bool case4_tick(unitree::robot::go2::SportClient &sc,
                unitree::robot::go2::ObstaclesAvoidClient &avoid_client);