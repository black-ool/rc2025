#pragma once

#include <cmath>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

// =============================================================================
// 坐标变换
// =============================================================================
void transformLocal(double x, double y, double yaw_now,
                    double &lx, double &ly, double &dyaw);

// =============================================================================
// PID
// =============================================================================
float PID_Yaw(float expect, float err);
float PID_Yaw1(float expect, float err);

// =============================================================================
// 角度工具
// =============================================================================
double wrapAngle(double a);

// =============================================================================
// 雷达安全距离
// =============================================================================
float safeRange(float v, float fallback = 5.0f);

inline float rawRangeMag(float r)
{
    return std::isfinite(r) ? std::fabs(r) : 0.f;
}

void applyRangeClearance(float raw_front, float raw_left, float raw_right,
                         float &vx, float &vy);

void applyRangeClearanceFrontOnly(float raw_front, float &vx);

// =============================================================================
// 站直检测
// =============================================================================
bool isGo2UprightAfterJump(const unitree_go::msg::dds_::SportModeState_ &st);

// =============================================================================
// 转弯角速度指令
// =============================================================================
double yawTurnRateCmd(double err_rad);

// =============================================================================
// 窄过道 / 路口判定
// =============================================================================
bool isInNarrowCorridor(float left, float right);
bool isMazeJunctionCandidate(float front, float left, float right);

// =============================================================================
// 迷宫子状态重置
// =============================================================================
void resetMazeFsm();