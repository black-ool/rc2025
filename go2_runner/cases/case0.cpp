#include "case0.h"
#include "../params.h"
#include "../globals.h"
#include "../utils.h"

#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>

using namespace unitree::robot;
using namespace cv;
using namespace std;

bool case0_tick(go2::SportClient &sc,
                const Mat &undist,
                const unitree_go::msg::dds_::SportModeState_ &state,
                double lx)
{
    /** 0 前进；1 落地站直 + 稳定等待；2 巡线 */
    static int case0_phase = 0;
    static double lx_anchor_case0 = 0;
    static bool lx_anchor_case0_set = false;
    static int case0_jump_settle_frm = 0;
    static bool case0_settle_armed = false;
    static int case0_upright_stable = 0;
    static int case0_upright_wait_frm = 0;
    static double lx_anchor_post_jump = 0;
    static bool lx_anchor_post_jump_set = false;

    if (!lx_anchor_case0_set)
    {
        lx_anchor_case0 = lx;
        lx_anchor_case0_set = true;
    }
    const double forward_in_case0 = lx - lx_anchor_case0;

    if (case0_phase == 0)
    {
        sc.StaticWalk();
        if (forward_in_case0 >= kCase0PreJumpForward_m)
        {
            sc.StopMove();
            sc.FrontJump();
            case0_phase = 1;
            case0_settle_armed = false;
            case0_upright_stable = 0;
            case0_upright_wait_frm = 0;
            start_jump_times = 1;
            cout << "[case0] 已前进 " << forward_in_case0 << " m → 起跳" << endl;
        }
        else
        {
            float vx0 = 0.3f, vy0 = 0.f;
            applyRangeClearance(ob_x, ob_y, ob_z, vx0, vy0);
            sc.Move(vx0, vy0, 0.f);
        }
        return false;
    }

    if (case0_phase == 1)
    {
        sc.StaticWalk();
        sc.Move(0.f, 0.f, 0.f);
        ++case0_upright_wait_frm;

        if (!case0_settle_armed)
        {
            if (isGo2UprightAfterJump(state))
                ++case0_upright_stable;
            else
                case0_upright_stable = 0;

            if (case0_upright_stable >= kCase0UprightStableFrames)
            {
                case0_settle_armed = true;
                case0_jump_settle_frm = kCase0PostJumpSettleFrames;
                cout << "[case0] 已落地站直，开始稳定等待 " << kCase0PostJumpSettleFrames
                     << " 帧" << endl;
            }
            else if (case0_upright_wait_frm >= kCase0UprightTimeoutFrames)
            {
                case0_settle_armed = true;
                case0_jump_settle_frm = kCase0PostJumpSettleFrames;
                cout << "[case0] 站直检测超时，仍开始稳定等待 " << kCase0PostJumpSettleFrames
                     << " 帧" << endl;
            }
        }
        else if (--case0_jump_settle_frm <= 0)
        {
            case0_phase = 2;
            lx_anchor_post_jump = lx;
            lx_anchor_post_jump_set = true;
            cout << "[case0] 稳定等待完成，开始巡线" << endl;
        }
        return false;
    }

    /* case0_phase == 2：巡线 */
    if (!lx_anchor_post_jump_set)
    {
        lx_anchor_post_jump = lx;
        lx_anchor_post_jump_set = true;
    }
    const double forward_since_jump = lx - lx_anchor_post_jump;

    Mat gray, blur, bin;
    cvtColor(undist, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, blur, {15, 15}, 0);
    threshold(blur, bin, 50, 255, THRESH_BINARY_INV);

    double err = 0;
    int cnt = 0;
    for (int r = blur.rows - 1; r >= blur.rows - 120; --r)
    {
        const uchar *row = bin.ptr(r);
        for (int c = 0; c < bin.cols; ++c)
            if (row[c])
            {
                err += c - 640;
                cnt++;
            }
    }
    err = cnt ? err / cnt : 0;

    double steer = -0.001 * err;

    sc.StaticWalk();
    sc.Euler(0, 0.25, 0);

    {
        float vx = kCase0LineFollowVx, vy = 0.f;
        applyRangeClearance(ob_x, ob_y, ob_z, vx, vy);
        sc.Move(vx, vy, steer);
    }

    const float f0 = safeRange(ob_x_f);
    const float l0 = safeRange(ob_y_f);
    const float r0 = safeRange(ob_z_f);
    static int case0_enter_maze_cnt = 0;
    if (forward_since_jump >= kCase0PostJumpMinForward_m && isInNarrowCorridor(l0, r0))
        case0_enter_maze_cnt++;
    else
        case0_enter_maze_cnt = 0;

    if (case0_enter_maze_cnt >= kEnterMazeStableFrames)
    {
        case0_enter_maze_cnt = 0;
        resetMazeFsm();
        cout << "[case0] 进入窄过道/迷宫 区段 (lx_fwd=" << forward_since_jump << " m, f=" << f0
             << " L=" << l0 << " R=" << r0 << ") → Flag_Task=1" << endl;
        return true;
    }
    return false;
}