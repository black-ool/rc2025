#include "callbacks.h"
#include "params.h"
#include "globals.h"

#include <unitree/idl/ros2/PointStamped_.hpp>
#include <cmath>

// =============================================================================
// 雷达测距回调
// =============================================================================
void rangeCB(const void *m)
{
    auto *p = (const geometry_msgs::msg::dds_::PointStamped_ *)m;
    const float rx = p->point().x();
    const float ry = p->point().y();
    const float rz = p->point().z();
    ob_x = rx;
    ob_y = ry;
    ob_z = rz;

    static bool have_x = false, have_y = false, have_z = false;
    auto ema = [](float raw, float &out, bool &have) {
        if (!std::isfinite(raw))
            return;
        if (!have)
        {
            out = raw;
            have = true;
        }
        else
            out = out * (1.f - kRangeEmaAlpha) + raw * kRangeEmaAlpha;
    };
    ema(rx, ob_x_f, have_x);
    ema(ry, ob_y_f, have_y);
    ema(rz, ob_z_f, have_z);
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