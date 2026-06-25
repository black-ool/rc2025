#include "case0.h"
#include "../params.h"
#include "../globals.h"
#include "../utils.h"
#include "../line_follow.h"

#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>

using namespace unitree::robot;
using namespace cv;
using namespace std;

// =============================================================================
// case0 主入口
// 返回: 0=继续, 1=去case1(避障), 2=去case2(ArUco)
// =============================================================================
int case0_tick(go2::SportClient &sc,
               const Mat &undist,
               const unitree_go::msg::dds_::SportModeState_ &state,
               int fcount)
{
    (void)state;

    double lx, ly, dyaw;
    transformLocal(px, py, yaw, lx, ly, dyaw);

    // init_stage: 0=直行0.2m, 1=跳跃, 2=稳定站立, 3=巡线
    static int init_stage = 0;

    // ★ --task 0：跳过启动阶段，直接巡线
    if (g_case0_skip_init && init_stage < 3)
    {
        init_stage = 3;
        cout << "[Start] ✅ --task 0: skip init, directly line follow." << endl;
    }

    if (init_stage == 0) {
        sc.StaticWalk(); sc.Euler(0, 0, 0); sc.Move(0.15, 0, 0);
        if (lx >= 0.2) { sc.StopMove(); sc.Move(0, 0, 0); init_stage = 1; }
        return 0;
    }
    if (init_stage == 1) {
        sc.FrontJump(); init_stage = 2;
        this_thread::sleep_for(chrono::milliseconds(300));
        px0 = px; py0 = py; yaw0 = yaw;
        return 0;
    }
    if (init_stage == 2) {
        sc.BalanceStand();
        this_thread::sleep_for(chrono::milliseconds(500));
        init_stage = 3;
        return 0;
    }

    // init_stage == 3: 纯巡线
    return pureLineFollow(sc, undist, lx, ly, dyaw, fcount, g_case0_second_pass);
}

void case0_reset_statics() {}