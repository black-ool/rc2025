#include "line_follow.h"
#include "params.h"
#include "globals.h"

#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include <vector>

using namespace unitree::robot;
using namespace cv;
using namespace std;

// =============================================================================
// detectLine: 峰值法检测黑线 (与 rc2025.cpp detectLine 完全一致)
// =============================================================================
static bool detectLine(const Mat &undist, double &err, int &cnt, Mat &debug_img)
{
    debug_img = undist.clone();

    Mat gray, blur, bin;
    cvtColor(undist, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, blur, {5, 5}, 0);

    static int debug_threshold = 50;
    threshold(blur, bin, debug_threshold, 255, THRESH_BINARY_INV);

    {
        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(bin, bin, MORPH_OPEN, kernel);
    }

    int roi_height = 40;
    int roi_y_start = blur.rows - roi_height;
    if (roi_y_start < 0) roi_y_start = 0;

    rectangle(debug_img, {0, roi_y_start}, {blur.cols, blur.rows}, {255, 0, 0}, 1);

    err = 0;
    cnt = 0;
    const int roi_width = bin.cols;
    vector<int> col_count(roi_width, 0);

    for (int r = roi_y_start; r < blur.rows; ++r)
    {
        const uchar *row = bin.ptr(r);
        for (int c = 0; c < roi_width; ++c)
            if (row[c] > 0) { col_count[c]++; cnt++; }
    }

    int roi_total_pixels = (blur.rows - roi_y_start) * roi_width;
    double roi_percentage = (double)cnt / roi_total_pixels * 100;

    int peak_col = -1, peak_count = 0;
    for (int c = 0; c < roi_width; ++c)
        if (col_count[c] > peak_count) { peak_count = col_count[c]; peak_col = c; }

    if (peak_count >= 5)
    {
        err = peak_col - 640;
        int cx = max(0, min(1279, peak_col));
        line(debug_img, {cx, blur.rows - roi_height/2}, {cx, blur.rows}, {0, 255, 0}, 2);
        circle(debug_img, {cx, blur.rows - roi_height/2}, 8, {0, 255, 0}, 2);
    }
    else err = 0;

    if (cnt >= 50 && cnt <= 50000 && peak_count >= 5)
    {
        putText(debug_img, format("Line: err=%.1f cnt=%d peak=%d %.1f%%",
                 err, cnt, peak_count, roi_percentage), {10, 30}, FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
        return true;
    }

    string reason;
    if (cnt < 50) reason = format("NO LINE: cnt=%d", cnt);
    else if (cnt > 50000) reason = format("NO LINE: cnt>50000");
    else if (peak_count < 5) reason = format("NO LINE: peak=%d", peak_count);
    else reason = "NO LINE";

    putText(debug_img, reason, {10, 60}, FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 255}, 2);
    return false;
}

// =============================================================================
// pureLineFollow: 纯巡线（无跳跃/启动）
// is_second_pass=true 时检测到 ArUco 返回 2（→ case2）
// =============================================================================
int pureLineFollow(go2::SportClient &sc,
                   const Mat &undist,
                   double lx, double ly, double dyaw,
                   int fcount,
                   bool is_second_pass)
{
    // ★ 第二段巡线：检测到 ArUco → 直接返回 2
    if (is_second_pass && g_last_aruco_id > 0)
    {
        cout << "\033[32m[Line] ArUco " << g_last_aruco_id.load() << " → case2\033[0m" << endl;
        sc.StopMove();
        return 2;
    }

    double line_err = 0;
    int line_cnt = 0;
    Mat line_debug;
    bool line_found = detectLine(undist, line_err, line_cnt, line_debug);

    // 线条跳变检测
    static double prev_line_err = 0;
    static bool had_line_before = false;
    if (line_found && had_line_before && lx >= kLineJumpTrigger_m)
    {
        if (abs(line_err - prev_line_err) > kLineJumpThreshold)
        {
            cout << "[Line] JUMP → exit" << endl;
            sc.StopMove();
            prev_line_err = 0; had_line_before = false;
            return 1;
        }
    }
    if (line_found) { prev_line_err = line_err; had_line_before = true; }

    // ly 纠偏
    double ly_correction = 0;
    if (ly > kLineLyCorrThreshold)       ly_correction = -kLineLyCorrSteer;
    else if (ly < -kLineLyCorrThreshold) ly_correction =  kLineLyCorrSteer;

    // PID 巡线
    if (line_found && abs(line_err) < 400 && line_cnt > 100 && line_cnt < 10000)
    {
        static double integral = 0, last_err = 0;
        integral += line_err;
        integral = max(-kLinePID_IntegralMax, min(kLinePID_IntegralMax, integral));
        double steer = -(kLinePID_Kp * line_err + kLinePID_Ki * integral + kLinePID_Kd * (line_err - last_err));
        last_err = line_err;
        steer = max(-kLineSteerMax, min(kLineSteerMax, steer));
        if (abs(ly_correction) > 0.01) steer = ly_correction;
        steer += -dyaw * kLineYawKeepGain;
        steer = max(-kLineSteerMax, min(kLineSteerMax, steer));

        sc.StaticWalk(); sc.Euler(0, 0.4, 0); sc.Move(0.25, 0, steer);
    }
    else if (line_found)
    {
        double soft_steer = abs(line_err) < 640 ? max(-kLineSoftSteerMax, min(kLineSoftSteerMax, -line_err * 0.003)) : 0;
        if (abs(ly_correction) > 0.01) soft_steer = ly_correction;
        soft_steer += -dyaw * kLineYawKeepGain;
        soft_steer = max(-kLineSteerMax, min(kLineSteerMax, soft_steer));
        sc.StaticWalk(); sc.Euler(0, 0.4, 0); sc.Move(0.2, 0, soft_steer);
    }
    else
    {
        double steer = max(-kLineNoLineSteerMax, min(kLineNoLineSteerMax, ly_correction + (-dyaw * kLineYawKeepGain)));
        sc.StaticWalk(); sc.Euler(0, 0.4, 0); sc.Move(0.15, 0, steer);
    }

    // 第一段：雷达触发避障
    if (!is_second_pass && lx > kLineObstacleTrigger_m && ob_x_f < kLineObstacleFront_m)
    {
        cout << "\033[32m[Line] Lidar obstacle → case1\033[0m" << endl;
        sc.StopMove();
        return 1;
    }

    return 0;
}