#include "case1.h"
#include "../params.h"
#include "../globals.h"

#include <cmath>
#include <iostream>
#include <algorithm>

using namespace unitree::robot;
using namespace std;

// =============================================================================
// case1：S型走廊避障 (与 rc2025.cpp Flag_Task=1 完全一致)
// =============================================================================
bool case1_tick(go2::SportClient &sc,
                int fcount,
                double lx,
                double ly,
                double yaw_now)
{
    // 平视，让激光雷达正对前方墙壁
    sc.Euler(0, 0, 0);
    sc.StaticWalk();

    static int phase = 0;
    static double phase_start_lx = lx;
    static double yaw_turn_start = yaw_now;

    // 雷达三通道 (EMA 滤波值)
    double front_dist = ob_x_f;
    double left_dist  = ob_y_f;
    double right_dist = ob_z_f;

    // 墙壁检测：原始值 < 0.6m → 触发转弯
    bool front_wall_raw = (ob_x > 0.01 && ob_x <= 0.6);
    bool front_too_close_raw = (ob_x <= 0.01 && front_dist < 0.6);
    bool wall_detected = front_wall_raw || front_too_close_raw;
    bool wall_detected_straight = wall_detected;

    // 三向安全保护
    float vx_safe = 0.15f, vy_safe = 0.f;

    // 左侧太近 → vy 减小（向右侧平移）
    if (left_dist > 999.0 || left_dist < 0.2) {
        float deficit = (left_dist > 999.0) ? 0.2f : (0.2f - left_dist);
        vy_safe -= min(0.38f, deficit * 3.0f);
    }
    // 右侧太近 → vy 增大（向左侧平移）
    if (right_dist > 999.0 || right_dist < 0.2) {
        float deficit = (right_dist > 999.0) ? 0.2f : (0.2f - right_dist);
        vy_safe += min(0.38f, deficit * 3.0f);
    }
    // 前方太近 → 减速/后退
    if (front_dist > 999.0 || front_dist < 0.4) {
        float deficit = (front_dist > 999.0) ? 0.4f : (0.4f - front_dist);
        vx_safe -= min(0.28f, deficit * 2.0f);
        vx_safe = max(-0.28f, vx_safe);
    }

    // 安全状态日志
    bool safety_active = (abs(vy_safe) > 0.02 || vx_safe < 0.12);
    if (safety_active) {
        cout << "[OB] 🛡️ SAFETY: vx=" << vx_safe << " vy=" << vy_safe
             << " (front=" << front_dist << " left=" << left_dist << " right=" << right_dist << ")" << endl;
    }

    // 走廊居中 (直行阶段)
    bool is_straight = (phase == 0 || phase == 2 || phase == 4 || phase == 6 || phase == 8 || phase == 10);
    float vy_center = 0.f;
    if (is_straight) {
        float side_sum = left_dist + right_dist;
        if (side_sum > 0.05f) {
            vy_center = max(-0.12f, min((float)((left_dist - right_dist) * 0.42f), 0.12f));
            if (abs(vy_center) > 0.01f)
                cout << "[OB] 🎯 Centering: L=" << left_dist << " R=" << right_dist << " vy=" << vy_center << endl;
        }
    }

    // 最终 vx/vy
    float vx_final = vx_safe;
    float vy_final = vy_safe + vy_center;
    vy_final = max(-0.38f, min(0.38f, vy_final));

    // 调试输出
    if (fcount % 10 == 0)
    {
        cout << "[OB] S-CORRIDOR phase=" << phase
             << " F=" << front_dist << " L=" << left_dist << " R=" << right_dist
             << " lx=" << lx << " vy=" << vy_final
             << " yaw=" << yaw_now*180/M_PI << "deg" << endl;
    }

    // ----- Phase 0: 直线走到墙 (带纠偏) -----
    if (phase == 0) {
        double steer_phase0 = -ly * 0.3;
        double yaw_drift = yaw_now - yaw0;
        if (yaw_drift > M_PI) yaw_drift -= 2*M_PI;
        if (yaw_drift < -M_PI) yaw_drift += 2*M_PI;
        steer_phase0 += -yaw_drift * 2.0;
        steer_phase0 = max(-0.3, min(0.3, steer_phase0));

        if (abs(steer_phase0) > 0.02) {
            cout << "[OB] Phase0 STRAIGHT: ly=" << ly << " yaw_drift="
                 << yaw_drift*180/M_PI << "deg steer=" << steer_phase0 << endl;
        }

        sc.Move(vx_final, vy_final, steer_phase0);

        // 紧急停止保护
        if (front_dist < 0.2) {
            sc.Move(0, 0, 0);
            cout << "[OB] ⚠️ EMERGENCY STOP: front_dist=" << front_dist << "m!" << endl;
        }

        // lx >= 1.4 后才允许触发
        if (wall_detected && lx >= 1.4) {
            phase = 1;
            yaw_turn_start = yaw_now;
            phase_start_lx = lx;
            cout << "[OB] ✅ Phase 0 DONE: Wall at " << front_dist
                 << "m (lx=" << lx << ") → START LEFT TURN 90°" << endl;
        }
    }
    // ----- Phase 1/3/9: 向左弧线转 90° -----
    else if (phase == 1 || phase == 3 || phase == 9) {
        double yaw_diff = yaw_now - yaw_turn_start;
        if (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
        if (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;

        if (yaw_diff >= M_PI / 2 * 0.9) {
            int prev = phase;
            phase++;
            phase_start_lx = lx;
            cout << "[OB] ✅ Phase " << prev << " DONE: Left turn 90° (yaw diff=" << yaw_diff*180/M_PI << "deg)" << endl;
        } else {
            float vx_turn = min(0.15f, vx_final);
            sc.Move(vx_turn, vy_final, 0.8f);  // 左转弧线
        }
    }
    // ----- Phase 5/7: 向右弧线转 90° -----
    else if (phase == 5 || phase == 7) {
        double yaw_diff = yaw_now - yaw_turn_start;
        if (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
        if (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;

        if (yaw_diff <= -M_PI / 2 * 0.9) {
            int prev = phase;
            phase++;
            phase_start_lx = lx;
            cout << "[OB] ✅ Phase " << prev << " DONE: Right turn 90° (yaw diff=" << yaw_diff*180/M_PI << "deg)" << endl;
        } else {
            float vx_turn = min(0.15f, vx_final);
            sc.Move(vx_turn, vy_final, -0.8f);  // 右转弧线
        }
    }
    // ----- Phase 2/4/6/8/10: 直行 -----
    else if (phase == 2 || phase == 4 || phase == 6 || phase == 8 || phase == 10) {
        if (wall_detected_straight) {
            if (phase == 10) {
                // Phase 10 → 完成 S 型序列，回到巡线
                phase = 0;
                cout << "[OB] 🎉 S-SHAPED CORRIDOR NAVIGATION COMPLETE! Resuming line follow." << endl;
                return true;  // → Flag_Task = 0
            } else {
                phase++;
                yaw_turn_start = yaw_now;
                phase_start_lx = lx;
                cout << "[OB] ✅ Phase " << phase-1 << " DONE: Forward until wall → Phase " << phase << endl;
            }
        } else {
            sc.Move(vx_final, vy_final, 0.f);
        }
    }

    return false;
}

void case1_reset_statics()
{
    // phase / phase_start_lx / yaw_turn_start 等 static 变量在 case1_tick 内部
    // 此函数为占位，留待后续需要时扩展
}
