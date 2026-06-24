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

// 避障阶段
int obstacle_avoidance_state = 0;

// ArUco
std::atomic<int> g_last_aruco_id(-1);

// GUI
bool g_enable_gui = false;