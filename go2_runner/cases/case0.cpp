#include "case0.h"
#include "../params.h"
#include "../globals.h"
#include "../utils.h"

#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>
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

    // 形态学开运算 (3x3): 去除文字笔画噪点
    {
        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(bin, bin, MORPH_OPEN, kernel);
    }

    // ROI: 底部 40 行
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
        {
            if (row[c] > 0)
            {
                col_count[c]++;
                cnt++;
            }
        }
    }

    int roi_total_pixels = (blur.rows - roi_y_start) * roi_width;
    double roi_percentage = (double)cnt / roi_total_pixels * 100;

    // 水平投影峰值法找线条中心
    int peak_col = -1;
    int peak_count = 0;
    for (int c = 0; c < roi_width; ++c) {
        if (col_count[c] > peak_count) {
            peak_count = col_count[c];
            peak_col = c;
        }
    }

    if (peak_count >= 5) {
        err = peak_col - 640;  // 偏差 = 峰值列 - 画面中心

        int center_x = peak_col;
        center_x = max(0, min(1279, center_x));

        line(debug_img, {center_x, blur.rows - roi_height/2},
             {center_x, blur.rows}, {0, 255, 0}, 2);
        circle(debug_img, {center_x, blur.rows - roi_height/2}, 8, {0, 255, 0}, 2);
    } else {
        err = 0;
    }

    // 有效性检查
    if (cnt >= 50 && cnt <= 50000 && peak_count >= 5)
    {
        putText(debug_img, format("Line: err=%.1f cnt=%d peak=%d thresh=%d %.1f%%",
                 err, cnt, peak_count, debug_threshold, roi_percentage),
                {10, 30}, FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);

        cout << "[LINE DETAIL] SUCCESS: err=" << err << " cnt=" << cnt << " peak=" << peak_count
             << " percentage=" << roi_percentage << "%" << endl;

        return true;
    }

    string reason;
    int status_color = 0;

    if (cnt < 50) {
        reason = format("NO LINE: cnt=%d (<50)", cnt);
        status_color = 0;
    } else if (cnt > 50000) {
        reason = format("NO LINE: cnt=%d (>50000)", cnt);
        status_color = 0;
    } else if (peak_count < 5) {
        reason = format("NO LINE: peak=%d (<5) text noise?", peak_count);
        status_color = 1;
    } else {
        reason = format("NO LINE: bad ROI or lighting");
        status_color = 1;
    }

    putText(debug_img, reason, {10, 60}, FONT_HERSHEY_SIMPLEX, 0.6,
            status_color == 0 ? Scalar(0, 0, 255) : Scalar(0, 255, 255), 2);

    static int print_counter = 0;
    print_counter++;
    if (print_counter % 10 == 0)
    {
        cout << "[LINE DEBUG] img_mean=" << mean(gray)[0]
             << " roi_total=" << roi_total_pixels
             << " cnt=" << cnt
             << " peak=" << peak_count
             << " percentage=" << roi_percentage << "%" << endl;
    }

    return false;
}

// =============================================================================
// case0 主入口
// =============================================================================
bool case0_tick(go2::SportClient &sc,
                const Mat &undist,
                const unitree_go::msg::dds_::SportModeState_ &state,
                int fcount)
{
    (void)state; // 当前未使用，保留接口兼容

    double lx, ly, dyaw;
    transformLocal(px, py, yaw, lx, ly, dyaw);

    // ---- init_stage: 0=先直走0.2m, 1=跳跃, 2=恢复站立, 3=巡线 ----
    static int init_stage = 0;

    if (init_stage == 0) {
        sc.StaticWalk();
        sc.Euler(0, 0, 0);
        sc.Move(0.15, 0, 0);
        if (lx >= 0.2) {
            sc.StopMove();
            sc.Move(0, 0, 0);
            init_stage = 1;
            cout << "[Start] ✅ Pre-move 0.2m done (lx=" << lx << ")" << endl;
        }
        return false;
    }

    if (init_stage == 1) {
        sc.FrontJump();
        init_stage = 2;
        this_thread::sleep_for(chrono::milliseconds(300));

        px0 = px;
        py0 = py;
        yaw0 = yaw;
        cout << "[Start] Jump done, reset origin. Now stabilizing..." << endl;
        return false;
    }

    if (init_stage == 2) {
        sc.BalanceStand();
        this_thread::sleep_for(chrono::milliseconds(500));
        init_stage = 3;
        cout << "[Start] ✅ Stabilized after jump. Starting line follow." << endl;
        return false;
    }

    // ---- init_stage == 3: 正常巡线 ----

    // Line detection
    double line_err = 0;
    int line_cnt = 0;
    Mat line_debug;
    bool line_found = detectLine(undist, line_err, line_cnt, line_debug);

    // 线条跳变检测 (终点的"入口"文字)
    static double prev_line_err = 0;
    static bool had_line_before = false;
    if (line_found && had_line_before && lx >= 0.6) {
        double jump = abs(line_err - prev_line_err);
        if (jump > 130.0) {
            cout << "[Line] ⚠️ JUMP detected: prev_err=" << prev_line_err
                 << " now=" << line_err << " jump=" << jump
                 << " → END OF LINE, entering obstacle avoidance" << endl;
            line_found = false;
            sc.StopMove();
            cout << "\033[32m[Transition] Line ended (JUMP), entering obstacle avoidance\033[0m" << endl;
            // 重置跳变检测
            prev_line_err = 0;
            had_line_before = false;
            return true;  // → Flag_Task = 1
        }
    }
    if (line_found) {
        prev_line_err = line_err;
        had_line_before = true;
    }

    // 每 10 帧输出 lx 值
    if (fcount % 10 == 0)
    {
        cout << "[Line] lx=" << lx << " ly=" << ly << " (trigger at 0.75m)" << endl;
    }

    // ly 漂移主动回正
    double ly_correction = 0;
    if (ly > 0.35) {
        ly_correction = -0.3;
        cout << "[LY] ly=" << ly << " > 0.35 → LEFT correction" << endl;
    } else if (ly < -0.35) {
        ly_correction = 0.3;
        cout << "[LY] ly=" << ly << " < -0.35 → RIGHT correction" << endl;
    }

    // 有效的线条 → PID 巡线
    if (line_found)
    {
        if (abs(line_err) < 400 && line_cnt > 100 && line_cnt < 10000)
        {
            cout << "[Line] lx=" << lx << " ly=" << ly << " err=" << line_err
                 << " cnt=" << line_cnt << " (OK)" << endl;

            // PID 控制: Kp=0.12, Ki=0.002, Kd=0.01
            static double integral = 0, last_err = 0;
            double Kp = 0.12;
            double Ki = 0.002;
            double Kd = 0.01;

            integral += line_err;
            integral = max(-50.0, min(50.0, integral));

            double derivative = line_err - last_err;
            last_err = line_err;

            double steer = -(Kp * line_err + Ki * integral + Kd * derivative);
            steer = max(-0.5, min(0.5, steer));

            // ly 纠偏覆盖
            if (abs(ly_correction) > 0.01) {
                steer = ly_correction;
                cout << "[Line] ly OVERRIDE steer=" << steer << endl;
            }

            // 航向保持
            steer += -dyaw * 1.5;
            steer = max(-0.5, min(0.5, steer));

            cout << "[Line] steer=" << steer << " (yaw_keep: dyaw=" << dyaw*180/M_PI << "deg)" << endl;

            sc.StaticWalk();
            sc.Euler(0, 0.4, 0);  // 低头姿态
            sc.Move(0.25, 0, steer);
        }
        else
        {
            // 误差超范围 → 直行 + 软纠偏
            double soft_steer = 0;
            if (abs(line_err) < 640) {
                soft_steer = -line_err * 0.003;
                soft_steer = max(-0.3, min(0.3, soft_steer));
            }
            if (abs(ly_correction) > 0.01) {
                soft_steer = ly_correction;
                cout << "[Line] INVALID ly OVERRIDE steer=" << soft_steer << endl;
            }
            soft_steer += -dyaw * 1.5;
            soft_steer = max(-0.5, min(0.5, soft_steer));

            cout << "[Line] INVALID: err=" << line_err << " cnt=" << line_cnt
                 << " -> going straight with soft steer=" << soft_steer
                 << " (yaw_keep: dyaw=" << dyaw*180/M_PI << "deg)" << endl;

            sc.StaticWalk();
            sc.Euler(0, 0.4, 0);
            sc.Move(0.2, 0, soft_steer);
        }
    }
    else
    {
        // 无线条 → 直行 + ly纠偏 + 航向保持
        double noline_steer = ly_correction + (-dyaw * 1.5);
        noline_steer = max(-0.3, min(0.3, noline_steer));

        cout << "[Line] NO LINE - going straight (yaw_keep: dyaw=" << dyaw*180/M_PI << "deg steer=" << noline_steer << ")" << endl;

        sc.StaticWalk();
        sc.Euler(0, 0.4, 0);
        sc.Move(0.15, 0, noline_steer);
    }

    // 雷达触发进入避障: lx > 0.75m 且前方障碍 < 1.5m
    if (lx > 0.75 && ob_x_f < 1.5)
    {
        cout << "\033[32m[Transition] Obstacle detected by lidar (dist=" << ob_x_f << "m), entering obstacle avoidance\033[0m" << endl;
        sc.StopMove();
        return true;  // → Flag_Task = 1
    }

    return false;
}

// 外部 reset 接口（占位，case0 内部 static 变量由函数自身管理）
void case0_reset_statics() {}