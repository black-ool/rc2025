#include "utils.h"
#include "params.h"
#include "globals.h"

#include <cmath>
#include <algorithm>

// =============================================================================
// 安全范围过滤 (与 rc2025.cpp safeRange 完全一致)
// =============================================================================
double safeRange(double raw)
{
    if (raw > 0.1 && raw < 5.0)
    {
        return raw;
    }
    return 999.0; // 无效值标记
}

// =============================================================================
// 坐标变换 (与 rc2025.cpp transformLocal 完全一致)
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