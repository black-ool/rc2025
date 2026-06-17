#pragma once

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

// =============================================================================
// DDS 回调声明
// =============================================================================
void rangeCB(const void *m);

class StateCB
{
public:
    unitree_go::msg::dds_::SportModeState_ state;
    void operator()(const void *m);
};