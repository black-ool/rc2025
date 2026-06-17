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

// --- case0 → case1：窄过道入门（约 0.6 m 宽 → 居中时左右约 0.3 m）---
static constexpr double kCase0PreJumpForward_m = 0.1;
static constexpr double kCase0PostJumpMinForward_m = 0.45;
static constexpr int kCase0PostJumpSettleFrames = 12;
static constexpr float kCase0LineFollowVx = 0.18f;
static constexpr double kCase0UprightMaxRollPitchRad = 0.18;
static constexpr float kCase0UprightMaxHorizVel = 0.25f;
static constexpr float kCase0UprightMaxVertVel = 0.20f;
static constexpr float kCase0UprightMaxYawRate = 0.45f;
static constexpr int16_t kCase0UprightMinFootForce = 80;
static constexpr int kCase0UprightStableFrames = 4;
static constexpr int kCase0UprightTimeoutFrames = 150;
static constexpr float kEnterMazeSideLo_m = 0.18f;
static constexpr float kEnterMazeSideHi_m = 0.48f;
static constexpr float kEnterMazeSideSumLo_m = 0.45f;
static constexpr float kEnterMazeSideSumHi_m = 0.88f;
static constexpr int kEnterMazeStableFrames = 4;

// --- case1：迷宫 —— 两次 180° 掉头 ---
static constexpr int kCase1NumUTurns = 2;
static constexpr float kMazeJunctionFront_m = 0.45f;
static constexpr float kMazeSideOpen_m = 0.52f;
static constexpr int kMazeJunctionStableFrames = 3;
static constexpr int kCase1PostEntryDelayFrames = 55;
static constexpr int kMazeTurnCooldownFrames = 20;
static constexpr int kMazeTurnTimeoutFrames = 700;
static constexpr int kMazeArc90TimeoutFrames = 500;
static constexpr float kMazeCorridorVyGain = 0.42f;
static constexpr float kMazeCorridorVyClamp = 0.12f;
static constexpr float kMazeForwardVx = 0.20f;
static constexpr double kCase1UTurnRadius_m = 0.6;
static constexpr float kCase1UTurnForwardVx = 0.15f;
static constexpr float kCase1FirstUTurnVxScale = 0.82f;
static constexpr double kCase1FirstUTurnRadiusScale = 1.12;
static constexpr float kCase1FirstUTurnVyBias = 0.04f;
static constexpr float kCase1SecondUTurnVxScale = 0.85f;
static constexpr double kCase1SecondUTurnRadiusScale = 1.10;
static constexpr float kCase1SecondUTurnVyBias = -0.045f;

// --- 原地转弯 ---
static constexpr double kYawTurnGain = 4.8;
static constexpr double kYawTurnCmdClamp = 1.25;
static constexpr double kYawTurnCmdFloor = 0.55;
static constexpr double kYawTurnDoneErrRad = 0.10;

// --- 雷达安全裕量 ---
static constexpr float kClearMinSide_m = 0.2f;
static constexpr float kClearMinFront_m = 0.4f;
static constexpr float kClearSideGain = 3.0f;
static constexpr float kClearFrontGain = 2.0f;
static constexpr float kClearVyAbsMax = 0.38f;
static constexpr float kClearVxRetreatMax = 0.28f;