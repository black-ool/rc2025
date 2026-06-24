#include "callbacks.h"
#include "params.h"
#include "globals.h"

#include <unitree/idl/ros2/PointStamped_.hpp>
#include <cmath>
#include <iostream>

using namespace std;

// =============================================================================
// 雷达测距回调 (与 rc2025.cpp rangeCB 完全一致)
// =============================================================================
void rangeCB(const void *m)
{
    auto *msg = static_cast<const geometry_msgs::msg::dds_::PointStamped_ *>(m);
    double rx = msg->point().x();  // front
    double ry = msg->point().y();  // left
    double rz = msg->point().z();  // right

    static bool g_have_front = false, g_have_left = false, g_have_right = false;

    // Update raw values (valid range 0.01~10.0m)
    if (rx > 0.01 && rx < 10.0) {
        ob_x = rx;
        if (!g_have_front) { ob_x_f = rx; g_have_front = true; }
        else ob_x_f = ob_x_f * (1.0 - kRangeEmaAlpha) + rx * kRangeEmaAlpha;
    }
    if (ry > 0.01 && ry < 10.0) {
        ob_y = ry;
        if (!g_have_left) { ob_y_f = ry; g_have_left = true; }
        else ob_y_f = ob_y_f * (1.0 - kRangeEmaAlpha) + ry * kRangeEmaAlpha;
    }
    if (rz > 0.01 && rz < 10.0) {
        ob_z = rz;
        if (!g_have_right) { ob_z_f = rz; g_have_right = true; }
        else ob_z_f = ob_z_f * (1.0 - kRangeEmaAlpha) + rz * kRangeEmaAlpha;
    }

    static int count = 0;
    if (++count % 100 == 0) {
        std::cout << "[RANGE] front=" << ob_x_f << " left=" << ob_y_f
                  << " right=" << ob_z_f << " (raw: x=" << rx << " y=" << ry << " z=" << rz << ")" << std::endl;
    }
}

// =============================================================================
// 运动状态回调
// =============================================================================
void StateCB::operator()(const void *m)
{
    state = *(const unitree_go::msg::dds_::SportModeState_ *)m;
    px = state.position()[0];
    py = state.position()[1];
    yaw = state.imu_state().rpy()[2];
}