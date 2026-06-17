#pragma once

#include <opencv2/opencv.hpp>
#include <deque>

// =============================================================================
// 实时雷达曲线可视化（OpenCV）
// =============================================================================

void pushRangeSampleRaw(std::deque<float> &q, float v);

/** 绘制 ob_x (raw) 和 ob_y+ob_z 共享 Y 轴曲线 */
cv::Mat makeRangePlotMat(const std::deque<float> &qx,
                         const std::deque<float> &qy,
                         const std::deque<float> &qz);