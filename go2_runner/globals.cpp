#include "globals.h"

// 传感器
float ob_x = 0, ob_y = 0, ob_z = 0;
float ob_x_f = 0, ob_y_f = 0, ob_z_f = 0;
double px = 0, py = 0, yaw = 0;
double px0 = 0, py0 = 0, yaw0 = 0;

// 任务状态机
int Flag_Task = 0;
int start_jump_times = 0;
int end_jump_times = 0;
bool found_turn = false;

// case1 迷宫子状态
int g_maze_nav = 0;
double g_maze_turn_target = 0.;
int g_maze_junc_stable = 0;
int g_maze_turn_cd = 0;
int g_maze_turn_frm = 0;
int g_case1_seg = 0;
bool g_case1_left90_done = false;
int g_case1_post90_stable = 0;
int g_case1_entry_delay_frm = 0;
int g_case1_coord_w_sign = 0;
bool g_case2_post_maze_placeholder = false;

// ArUco
std::atomic<int> g_last_aruco_id(-1);

// GUI
bool g_enable_gui = false;