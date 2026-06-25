#pragma once

#include <opencv2/opencv.hpp>

#define TOPIC_RANGE_INFO "rt/utlidar/range_info"
#define TOPIC_HIGHSTATE "rt/sportmodestate"

// =============================================================================
// 相机内参与畸变（请按实机标定替换）
// =============================================================================
static const cv::Mat K = (cv::Mat_<double>(3, 3) << 929.7797, 0, 629.6662,
                          0, 926.7584, 335.6207,
                          0, 0, 1);
static const cv::Mat D = (cv::Mat_<double>(1, 4) << -0.4157, 0.1327, 0, 0);

// =============================================================================
// 可调参数
// =============================================================================

// --- 雷达滤波 ---
static constexpr float kRangeEmaAlpha = 0.35f;

// --- 雷达安全阈值 ---
static constexpr float kClearMinSide_m = 0.2f;
static constexpr float kClearMinFront_m = 0.4f;
static constexpr float kClearVyAbsMax = 0.38f;
static constexpr float kClearVxRetreatMax = 0.28f;

// --- 巡线 (case0) ---
static constexpr double kPreJumpForward_m = 0.2;       // 跳跃前直行距离
static constexpr double kLineJumpTrigger_m = 0.6;       // 线条跳变检测所需最小 lx
static constexpr double kLineJumpThreshold = 130.0;     // 线条跳变像素阈值
static constexpr double kLineObstacleTrigger_m = 0.75;  // 雷达触发避障所需 lx
static constexpr double kLineObstacleFront_m = 1.5;     // 前方障碍触发避障距离
static constexpr double kLinePID_Kp = 0.12;
static constexpr double kLinePID_Ki = 0.002;
static constexpr double kLinePID_Kd = 0.01;
static constexpr double kLinePID_IntegralMax = 50.0;
static constexpr double kLineSteerMax = 0.5;
static constexpr double kLineSoftSteerMax = 0.3;
static constexpr double kLineNoLineSteerMax = 0.3;
static constexpr double kLineYawKeepGain = 1.5;
static constexpr double kLineLyCorrThreshold = 0.35;
static constexpr double kLineLyCorrSteer = 0.3;

// --- S型走廊避障 (case1) ---
static constexpr float kObsSafeVx = 0.15f;
static constexpr float kObsCorridorVyGain = 0.42f;
static constexpr float kObsCorridorVyClamp = 0.12f;
static constexpr float kObsCorridorVyMax = 0.38f;
static constexpr float kObsTurnYawRate = 0.8f;
static constexpr double kObsPhase0MinLx = 1.4;
static constexpr float kObsPhase0LyGain = 0.3f;
static constexpr double kObsPhase0YawGain = 2.0;
static constexpr double kObsPhase0SteerMax = 0.3;
static constexpr float kObsPhase0SafeFront = 0.2f;

// --- case3 台阶距离 ---
static constexpr double kStairsDist_m = 0.8;