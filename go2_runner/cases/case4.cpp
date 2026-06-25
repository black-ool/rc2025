#include "case4.h"

#include <iostream>
#include <thread>
#include <chrono>

using namespace unitree::robot;
using namespace std;

// =============================================================================
// case4：任务完成 —— 停止 + 恢复遥控器 (与 rc2025.cpp Flag_Task=9 一致)
// =============================================================================
bool case4_tick(go2::SportClient &sc,
                go2::ObstaclesAvoidClient &avoid_client)
{
    static bool done = false;
    if (done)
        return true;

    sc.StopMove();

    // 恢复遥控器控制
    avoid_client.UseRemoteCommandFromApi(false);
    avoid_client.SwitchSet(false);
    avoid_client.Move(0, 0, 0);
    this_thread::sleep_for(chrono::milliseconds(200));
    sc.SwitchJoystick(true);   // 重新启用摇杆
    sc.RecoveryStand();         // 恢复站立，释放 API 控制
    this_thread::sleep_for(chrono::milliseconds(500));
    sc.BalanceStand();

    cout << "\033[32mMission complete! Remote control restored.\033[0m" << endl;

    done = true;
    return true;  // → 主循环退出
}