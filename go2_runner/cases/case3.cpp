#include "case3.h"
#include "../globals.h"

#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>

using namespace unitree::robot;
using namespace std;

// =============================================================================
// case3：覆盖旧代码 Flag_Task=4~8
//
// 内部子状态:
//   0: 右转90° (Flag_Task=4)
//   1: FreeWalk过台阶 (Flag_Task=5)
//   2: StaticWalk前进 (Flag_Task=6)
//   3: 左转90° (Flag_Task=7)
//   4: 前跳 → 前进 → 切换到 Flag_Task=9 (Flag_Task=8)
// =============================================================================

static int case3_phase = 0;
static double case3_yaw_start = 0;
static double case3_lx_anchor = 0;
static bool case3_lx_anchor_set = false;

void case3_reset()
{
    case3_phase = 0;
    case3_yaw_start = 0;
    case3_lx_anchor = 0;
    case3_lx_anchor_set = false;
}

bool case3_tick(go2::SportClient &sc,
                double lx,
                double ly,
                double dyaw)
{
    (void)ly;
    (void)dyaw;

    // ---- Phase 0: 右转90° (Flag_Task=4) ----
    if (case3_phase == 0)
    {
        if (case3_yaw_start == 0)
        {
            sc.StopMove();
            case3_yaw_start = yaw;
        }

        double yaw_diff = yaw - case3_yaw_start;
        if (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
        if (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;

        if (yaw_diff > M_PI / 2 * 0.9)  // Right turn 90°
        {
            sc.StopMove();
            case3_phase = 1;
            case3_yaw_start = 0;
            case3_lx_anchor_set = false;
            cout << "[case3] Phase 0 DONE: Right turn 90° → FreeWalk stairs" << endl;
        }
        else
        {
            sc.Move(0, 0, -0.15);
        }
        return false;
    }

    // ---- Phase 1: FreeWalk 过台阶 (Flag_Task=5) ----
    if (case3_phase == 1)
    {
        sc.FreeWalk();
        sc.Move(0.15, 0, 0);

        if (!case3_lx_anchor_set)
        {
            case3_lx_anchor = lx;
            case3_lx_anchor_set = true;
        }
        double lx_since = lx - case3_lx_anchor;

        // 走 2.0m 后切到下一个阶段
        const double kStairsDist = 0.8;  // 使用旧代码的 obstacle_trigger_px + 2.0 ≈ 2.8m total, but from this anchor
        if (lx_since > kStairsDist)
        {
            case3_phase = 2;
            case3_lx_anchor_set = false;
            cout << "[case3] Phase 1 DONE: Stairs traversed (lx_since=" << lx_since << "m) → StaticWalk forward" << endl;
        }
        return false;
    }

    // ---- Phase 2: StaticWalk 前进 (Flag_Task=6) ----
    if (case3_phase == 2)
    {
        sc.StaticWalk();
        sc.Move(0.2, 0, 0);

        // 检测到 ArUco marker → 进入左转阶段
        if (g_last_aruco_id > 0)
        {
            case3_phase = 3;
            case3_yaw_start = 0;
            cout << "[case3] Phase 2 DONE: ArUco marker " << g_last_aruco_id.load() << " detected → Left turn 90°" << endl;
        }
        return false;
    }

    // ---- Phase 3: 左转90° (Flag_Task=7) ----
    if (case3_phase == 3)
    {
        if (case3_yaw_start == 0)
        {
            sc.StopMove();
            case3_yaw_start = yaw;
        }

        double yaw_diff = yaw - case3_yaw_start;
        if (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
        if (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;

        if (yaw_diff < -M_PI / 2 * 0.9)  // Left turn 90°
        {
            sc.StopMove();
            case3_phase = 4;
            case3_yaw_start = 0;
            end_jump_times = 0;
            cout << "[case3] Phase 3 DONE: Left turn 90° → Jump into finish area" << endl;
        }
        else
        {
            sc.Move(0, 0, 0.15);
        }
        return false;
    }

    // ---- Phase 4: 前跳 + 前进 (Flag_Task=8 → Flag_Task=9) ----
    if (case3_phase == 4)
    {
        if (end_jump_times == 0)
        {
            sc.FrontJump();
            end_jump_times++;
            cout << "[case3] Phase 4: Front jump → switching to Flag_Task=9 (finish)" << endl;
        }
        sc.Move(0.2, 0, 0);
        // 跳完后直接切换到 case4 (Flag_Task=9)
        return true;  // → Flag_Task = 9
    }

    return false;
}