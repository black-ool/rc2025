#include "utils.h"
#include "params.h"
#include "globals.h"

#include <cmath>
#include <algorithm>

// =============================================================================
// 坐标变换
// =============================================================================
void transformLocal(double x, double y, double yaw_now,
                    double &lx, double &ly, double &dyaw)
{
    double c = cos(yaw0), s = sin(yaw0);
    lx = (x - px0) * c + (y - py0) * s;
    ly = -(x - px0) * s + (y - py0) * c;
    dyaw = yaw_now - yaw0;
    if (dyaw > M_PI)
        dyaw -= 2 * M_PI;
    if (dyaw < -M_PI)
        dyaw += 2 * M_PI;
}

// =============================================================================
// PID
// =============================================================================
float PID_Yaw(float expect, float err)
{
    static float integral = 0, error_last = 0;
    float p = 5.0, i = 0, d = 0;
    float error_current = err - expect;
    integral += error_current;
    float output = -(p * error_current + i * integral + d * (error_current - error_last));
    error_last = error_current;
    return std::max(-2.0f, std::min(2.0f, output));
}

float PID_Yaw1(float expect, float err)
{
    static float integral = 0, error_last = 0;
    float p = 0.025, i = 0, d = 0;
    float error_current = err - expect;
    integral += error_current;
    float output = -(p * error_current + i * integral + d * (error_current - error_last));
    error_last = error_current;
    return std::max(-2.0f, std::min(2.0f, output));
}

// =============================================================================
// 角度工具
// =============================================================================
double wrapAngle(double a)
{
    while (a > M_PI)
        a -= 2 * M_PI;
    while (a <= -M_PI)
        a += 2 * M_PI;
    return a;
}

// =============================================================================
// 雷达安全距离
// =============================================================================
float safeRange(float v, float fallback)
{
    if (!std::isfinite(v))
        return fallback;
    return std::fabs(v);
}

void applyRangeClearance(float raw_front, float raw_left, float raw_right,
                         float &vx, float &vy)
{
    const float dL = rawRangeMag(raw_left);
    const float dR = rawRangeMag(raw_right);
    const float dF = rawRangeMag(raw_front);

    if (!std::isfinite(raw_left) || dL < kClearMinSide_m)
    {
        const float deficit =
            !std::isfinite(raw_left) ? kClearMinSide_m : (kClearMinSide_m - dL);
        vy -= std::min(kClearVyAbsMax, deficit * kClearSideGain);
    }
    if (!std::isfinite(raw_right) || dR < kClearMinSide_m)
    {
        const float deficit =
            !std::isfinite(raw_right) ? kClearMinSide_m : (kClearMinSide_m - dR);
        vy += std::min(kClearVyAbsMax, deficit * kClearSideGain);
    }
    vy = std::max(-kClearVyAbsMax, std::min(kClearVyAbsMax, vy));

    if (!std::isfinite(raw_front) || dF < kClearMinFront_m)
    {
        const float deficit =
            !std::isfinite(raw_front) ? kClearMinFront_m : (kClearMinFront_m - dF);
        vx -= std::min(kClearVxRetreatMax, deficit * kClearFrontGain);
        vx = std::max(-kClearVxRetreatMax, vx);
    }
}

void applyRangeClearanceFrontOnly(float raw_front, float &vx)
{
    const float dF = rawRangeMag(raw_front);
    if (!std::isfinite(raw_front) || dF < kClearMinFront_m)
    {
        const float deficit =
            !std::isfinite(raw_front) ? kClearMinFront_m : (kClearMinFront_m - dF);
        vx -= std::min(kClearVxRetreatMax, deficit * kClearFrontGain);
        vx = std::max(-kClearVxRetreatMax, vx);
    }
}

// =============================================================================
// 站直检测
// =============================================================================
bool isGo2UprightAfterJump(const unitree_go::msg::dds_::SportModeState_ &st)
{
    const auto &rpy = st.imu_state().rpy();
    if (std::fabs(rpy[0]) > kCase0UprightMaxRollPitchRad)
        return false;
    if (std::fabs(rpy[1]) > kCase0UprightMaxRollPitchRad)
        return false;

    const auto &vel = st.velocity();
    const float vxy = std::sqrt(vel[0] * vel[0] + vel[1] * vel[1]);
    if (vxy > kCase0UprightMaxHorizVel)
        return false;
    if (std::fabs(vel[2]) > kCase0UprightMaxVertVel)
        return false;
    if (std::fabs(st.yaw_speed()) > kCase0UprightMaxYawRate)
        return false;

    const auto &ff = st.foot_force();
    for (int i = 0; i < 4; ++i)
    {
        if (ff[static_cast<size_t>(i)] < kCase0UprightMinFootForce)
            return false;
    }
    return true;
}

// =============================================================================
// 转弯角速度指令
// =============================================================================
double yawTurnRateCmd(double err_rad)
{
    double c = err_rad * kYawTurnGain;
    if (std::fabs(c) < kYawTurnCmdFloor && std::fabs(err_rad) > 0.05)
        c = (err_rad > 0.0 ? kYawTurnCmdFloor : -kYawTurnCmdFloor);
    return std::max(-kYawTurnCmdClamp, std::min(kYawTurnCmdClamp, c));
}

// =============================================================================
// 窄过道 / 路口判定
// =============================================================================
bool isInNarrowCorridor(float left, float right)
{
    const bool band_lr = (left >= kEnterMazeSideLo_m && left <= kEnterMazeSideHi_m
                          && right >= kEnterMazeSideLo_m && right <= kEnterMazeSideHi_m);
    const float s = left + right;
    const bool sum_ok =
        (s >= kEnterMazeSideSumLo_m && s <= kEnterMazeSideSumHi_m && std::min(left, right) > 0.12f);
    return band_lr || sum_ok;
}

bool isMazeJunctionCandidate(float front, float left, float right)
{
    if (!(front < kMazeJunctionFront_m))
        return false;
    return (left > kMazeSideOpen_m) || (right > kMazeSideOpen_m);
}

// =============================================================================
// 迷宫子状态重置
// =============================================================================
void resetMazeFsm()
{
    g_maze_nav = 0;
    g_maze_turn_target = 0.;
    g_maze_junc_stable = 0;
    g_maze_turn_cd = 0;
    g_maze_turn_frm = 0;
    g_case1_seg = 0;
    g_case1_left90_done = false;
    g_case1_post90_stable = 0;
    g_case1_entry_delay_frm = kCase1PostEntryDelayFrames;
    g_case1_coord_w_sign = 0;
    g_case2_post_maze_placeholder = false;
}