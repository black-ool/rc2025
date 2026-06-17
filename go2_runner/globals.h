#pragma once

#include <atomic>

// =============================================================================
// 全局传感器与位姿状态
// =============================================================================
extern float ob_x, ob_y, ob_z;       // 雷达原始测距
extern float ob_x_f, ob_y_f, ob_z_f; // 滤波后测距
extern double px, py, yaw;           // 机体世界系位姿
extern double px0, py0, yaw0;        // 程序启动时刻位姿

// =============================================================================
// 任务状态机
// =============================================================================
extern int Flag_Task;                // 主状态机 case 编号

extern int start_jump_times;
extern int end_jump_times;
extern bool found_turn;

// =============================================================================
// case1 迷宫子状态
// =============================================================================
extern int g_maze_nav;
extern double g_maze_turn_target;
extern int g_maze_junc_stable;
extern int g_maze_turn_cd;
extern int g_maze_turn_frm;
extern int g_case1_seg;
extern bool g_case1_left90_done;
extern int g_case1_post90_stable;
extern int g_case1_entry_delay_frm;
extern int g_case1_coord_w_sign;
extern bool g_case2_post_maze_placeholder;

// =============================================================================
// ArUco
// =============================================================================
extern std::atomic<int> g_last_aruco_id;

// =============================================================================
// GUI 开关
// =============================================================================
extern bool g_enable_gui;