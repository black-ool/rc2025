#include "case2.h"
#include "../globals.h"

#include <cmath>
#include <iostream>

using namespace unitree::robot;
using namespace std;

// =============================================================================
// case2: ArUco 检测 → 左转 90° (与 rc2025.cpp Flag_Task=2 完全一致)
// =============================================================================

static bool turned = false;
static double yaw_at_turn_start = 0;

void case2_reset()
{
    turned = false;
    yaw_at_turn_start = 0;
}

bool case2_tick(go2::SportClient &sc)
{
    if (!turned)
    {
        if (yaw_at_turn_start == 0)
        {
            sc.StopMove();
            yaw_at_turn_start = yaw;
        }

        double yaw_diff = yaw - yaw_at_turn_start;
        if (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
        if (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;

        if (yaw_diff < -M_PI / 2 * 0.9)  // 90% of 90 degrees left
        {
            sc.StopMove();
            turned = true;
            cout << "[case2] Left turn 90° complete (yaw_diff=" << yaw_diff*180/M_PI << "deg)" << endl;
            return true;  // → Flag_Task = 3
        }
        else
        {
            sc.Move(0, 0, 0.15);  // Turn left in place
        }
        return false;
    }
    else
    {
        // Check for aruco marker 0
        if (g_last_aruco_id == 0)
        {
            cout << "[case2] ArUco marker 0 detected!" << endl;
            return true;  // → Flag_Task = 3
        }
        else
        {
            sc.Move(0.15, 0, 0);  // Move forward slowly
        }
        return false;
    }
}