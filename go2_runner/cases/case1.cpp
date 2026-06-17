#include "case1.h"
#include "../params.h"
#include "../globals.h"
#include "../utils.h"

#include <cmath>
#include <iostream>

using namespace unitree::robot;
using namespace std;

// =============================================================================
// 内部辅助（仅 case1 使用）
// =============================================================================

/** 开始一次 180° 掉头 */
static void case1BeginUTurn(go2::SportClient &sc, bool turn_left)
{
    g_maze_turn_target = wrapAngle(yaw + (turn_left ? M_PI : -M_PI));
    g_case1_coord_w_sign = turn_left ? 1 : -1;
    g_maze_nav = 1;
    g_maze_turn_frm = 0;
    (void)sc;
}

/** 开始左转 90° 弧线 */
static void case1BeginLeft90Arc(go2::SportClient &sc)
{
    g_maze_turn_target = wrapAngle(yaw + M_PI / 2.0);
    g_case1_coord_w_sign = 1;
    g_maze_nav = 2;
    g_maze_turn_frm = 0;
    (void)sc;
}

/** 第二次掉头前的路口触发判定 */
static bool isCase1SecondJunctionTrigger(float front, float left, float right)
{
    return isMazeJunctionCandidate(front, left, right);
}

/** 巡航一帧 */
static void case1CorridorFollowFrame(go2::SportClient &sc, float left, float right, float side_sum)
{
    sc.StaticWalk();
    float vx = kMazeForwardVx;
    float vy = 0.f;
    if (side_sum > 0.05f)
        vy = std::max(-kMazeCorridorVyClamp,
                      std::min(kMazeCorridorVyClamp, (left - right) * kMazeCorridorVyGain));
    applyRangeClearance(ob_x, ob_y, ob_z, vx, vy);
    sc.Move(vx, vy, 0.f);
}

/**
 * 180° 协调掉头单帧
 * @return 0 进行中；1 到位；2 超时
 */
static int case1UTurnTickFrame(go2::SportClient &sc)
{
    sc.StaticWalk();
    const double err = wrapAngle(g_maze_turn_target - yaw);
    ++g_maze_turn_frm;

    if (std::fabs(err) <= kYawTurnDoneErrRad)
    {
        sc.StopMove();
        g_maze_turn_frm = 0;
        g_maze_turn_cd = kMazeTurnCooldownFrames;
        g_maze_nav = 0;
        g_maze_junc_stable = 0;
        return 1;
    }
    if (g_maze_turn_frm > kMazeTurnTimeoutFrames)
    {
        sc.StopMove();
        g_maze_nav = 0;
        g_maze_turn_frm = 0;
        g_maze_turn_cd = kMazeTurnCooldownFrames * 2;
        return 2;
    }

    float vx = kCase1UTurnForwardVx;
    if (g_case1_seg == 0)
        vx *= kCase1FirstUTurnVxScale;
    else if (g_case1_seg == 1)
        vx *= kCase1SecondUTurnVxScale;
    applyRangeClearanceFrontOnly(ob_x, vx);
    const double vx_use = std::max((double)vx, 1e-4);
    double R_eff = kCase1UTurnRadius_m;
    if (g_case1_seg == 0)
        R_eff *= kCase1FirstUTurnRadiusScale;
    else if (g_case1_seg == 1)
        R_eff *= kCase1SecondUTurnRadiusScale;
    double w_mag = vx_use / R_eff;
    if (w_mag < kYawTurnCmdFloor)
        w_mag = kYawTurnCmdFloor;
    w_mag = std::min(w_mag, kYawTurnCmdClamp);
    const int w_dir = (g_case1_coord_w_sign >= 0 ? 1 : -1);
    float vy = 0.f;
    if (g_case1_seg == 0 && g_case1_coord_w_sign > 0)
        vy = kCase1FirstUTurnVyBias;
    else if (g_case1_seg == 1 && g_case1_coord_w_sign < 0)
        vy = kCase1SecondUTurnVyBias;
    const double w_cmd = (double)w_dir * w_mag;
    sc.Move(vx, vy, w_cmd);
    return 0;
}

/**
 * 左转 90° 协调弧线单帧
 * @return 0 进行中；1 到位；2 超时
 */
static int case1Arc90LeftTickFrame(go2::SportClient &sc)
{
    sc.StaticWalk();
    const double err = wrapAngle(g_maze_turn_target - yaw);
    ++g_maze_turn_frm;

    if (std::fabs(err) <= kYawTurnDoneErrRad)
    {
        sc.StopMove();
        g_maze_turn_frm = 0;
        g_maze_turn_cd = kMazeTurnCooldownFrames;
        g_maze_nav = 0;
        g_maze_junc_stable = 0;
        g_case1_post90_stable = 0;
        g_case1_left90_done = true;
        return 1;
    }
    if (g_maze_turn_frm > kMazeArc90TimeoutFrames)
    {
        sc.StopMove();
        g_maze_nav = 0;
        g_maze_turn_frm = 0;
        g_maze_turn_cd = kMazeTurnCooldownFrames * 2;
        g_case1_left90_done = true;
        return 2;
    }

    float vx = kCase1UTurnForwardVx;
    applyRangeClearanceFrontOnly(ob_x, vx);
    const double vx_use = std::max((double)vx, 1e-4);
    double w_mag = vx_use / kCase1UTurnRadius_m;
    if (w_mag < kYawTurnCmdFloor)
        w_mag = kYawTurnCmdFloor;
    w_mag = std::min(w_mag, kYawTurnCmdClamp);
    const int w_dir = (g_case1_coord_w_sign >= 0 ? 1 : -1);
    const double w_cmd = (double)w_dir * w_mag;
    sc.Move(vx, 0.f, w_cmd);
    return 0;
}

// =============================================================================
// case1 主入口
// =============================================================================
bool case1_tick(go2::SportClient &sc)
{
    const float front = safeRange(ob_x_f);
    const float left = safeRange(ob_y_f);
    const float right = safeRange(ob_z_f);
    const float side_sum = left + right;

    sc.Euler(0, 0, 0);

    if (g_maze_nav == 0)
    {
        if (g_maze_turn_cd > 0)
            --g_maze_turn_cd;

        if (g_case1_entry_delay_frm > 0)
        {
            --g_case1_entry_delay_frm;
            g_maze_junc_stable = 0;
        }

        const bool cd_ok = (g_maze_turn_cd == 0);
        bool started_maneuver = false;

        const bool first_uturn_armed =
            (g_case1_seg > 0) || (g_case1_entry_delay_frm == 0);
        const bool detect_junction =
            (g_case1_seg < kCase1NumUTurns) && cd_ok && first_uturn_armed;

        const bool cand_first =
            detect_junction && (g_case1_seg == 0) && isMazeJunctionCandidate(front, left, right);
        const bool cand_second =
            detect_junction && (g_case1_seg == 1) && isCase1SecondJunctionTrigger(front, left, right);

        if (cand_first || cand_second)
            ++g_maze_junc_stable;
        else
            g_maze_junc_stable = 0;

        if (detect_junction && g_maze_junc_stable >= kMazeJunctionStableFrames)
        {
            sc.StopMove();
            g_maze_junc_stable = 0;
            switch (g_case1_seg)
            {
            case 0:
                case1BeginUTurn(sc, true);
                started_maneuver = true;
                cout << "[case1] 第1次路口触发 (front=" << front << " L=" << left << " R=" << right
                     << ") → 180° R=" << kCase1UTurnRadius_m << "m 左转 目标yaw=" << g_maze_turn_target
                     << endl;
                break;
            case 1:
                case1BeginUTurn(sc, false);
                started_maneuver = true;
                cout << "[case1] 第2次路口触发（前进判定与第一次相同）(front=" << front << " L=" << left
                     << " R=" << right << ") → 180° R=" << kCase1UTurnRadius_m << "m 右转 目标yaw="
                     << g_maze_turn_target << endl;
                break;
            default:
                break;
            }
        }

        /* 两次 180° 后：前方第 3 次满足路口判定（稳定帧）→ 左转 90° 弧线 */
        if (!started_maneuver && (g_case1_seg == kCase1NumUTurns) && !g_case1_left90_done && cd_ok)
        {
            if (isMazeJunctionCandidate(front, left, right))
                ++g_case1_post90_stable;
            else
                g_case1_post90_stable = 0;

            if (g_case1_post90_stable >= kMazeJunctionStableFrames)
            {
                sc.StopMove();
                g_case1_post90_stable = 0;
                case1BeginLeft90Arc(sc);
                started_maneuver = true;
                cout << "[case1] 第3次路口判定 (front=" << front << " L=" << left << " R=" << right
                     << ") → 左转90°弧线 R=" << kCase1UTurnRadius_m << "m 目标yaw=" << g_maze_turn_target
                     << endl;
            }
        }

        if (!started_maneuver)
            case1CorridorFollowFrame(sc, left, right, side_sum);
    }
    else if (g_maze_nav == 1)
    {
        const int ut = case1UTurnTickFrame(sc);
        if (ut == 1)
        {
            ++g_case1_seg;
            const double err = wrapAngle(g_maze_turn_target - yaw);
            cout << "[case1] 第" << g_case1_seg << "次180°掉头完成 err=" << err;
            if (g_case1_seg >= kCase1NumUTurns)
                cout << " → 巡航，等待第3次路口判定做90°左弧" << endl;
            else
                cout << " → 继续前进，等待下一次路口" << endl;
            case1CorridorFollowFrame(sc, left, right, side_sum);
        }
        else if (ut == 2)
            cout << "[case1] 第" << (g_case1_seg + 1) << "次掉头超时" << endl;
    }
    else if (g_maze_nav == 2)
    {
        const int ar = case1Arc90LeftTickFrame(sc);
        if (ar == 1)
        {
            const double err = wrapAngle(g_maze_turn_target - yaw);
            cout << "[case1] 左转90°弧线完成 err=" << err << " → Flag_Task=2（占位停车）" << endl;
            g_case2_post_maze_placeholder = true;
            sc.StaticWalk();
            sc.Euler(0, 0.25, 0);
            sc.Move(0.f, 0.f, 0.f);
            return true;
        }
        else if (ar == 2)
            cout << "[case1] 左转90°弧线超时" << endl;
    }
    return false;
}