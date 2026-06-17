#pragma once

#include <unitree/robot/go2/sport/sport_client.hpp>

// =============================================================================
// case4：任务结束 —— 停止并输出完成信息
// 返回 true 表示主循环应退出
// =============================================================================
bool case4_tick(unitree::robot::go2::SportClient &sc);