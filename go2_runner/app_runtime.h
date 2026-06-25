#pragma once

#include "params.h"
#include "callbacks.h"

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/ros2/PointStamped_.hpp>

#include <opencv2/opencv.hpp>

// =============================================================================
// 运行时对象：DDS 订阅、Sport、Avoid、相机 —— 与 main 同生命周期
// =============================================================================
struct AppRuntime
{
    unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::PointStamped_> sub_range;
    StateCB stateCB;
    unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> sub_state;
    unitree::robot::go2::SportClient sc;
    unitree::robot::go2::ObstaclesAvoidClient avoid_client;
    cv::VideoCapture cap;

    AppRuntime()
        : sub_range(TOPIC_RANGE_INFO)
        , sub_state(TOPIC_HIGHSTATE)
    {}
};

/** Sport、订阅、Avoid、初始位姿、GStreamer 前视相机 */
bool initAppRuntime(AppRuntime &rt, const char *eth_if);