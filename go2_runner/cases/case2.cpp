#include "case2.h"
#include "../params.h"
#include "../globals.h"
#include "../utils.h"

#include <opencv2/opencv.hpp>

using namespace unitree::robot;
using namespace cv;
using namespace std;

void case2_tick(go2::SportClient &sc,
                const Mat &undist)
{
    if (g_case2_post_maze_placeholder)
    {
        sc.StaticWalk();
        sc.Euler(0, 0.25, 0);
        sc.Move(0.f, 0.f, 0.f);
        return;
    }

    sc.StaticWalk();
    sc.Euler(0, 0.25, 0);

    Mat gray7, blur7, bin7;
    cvtColor(undist, gray7, COLOR_BGR2GRAY);
    GaussianBlur(gray7, blur7, {15, 15}, 0);
    threshold(blur7, bin7, 50, 255, THRESH_BINARY_INV);

    double err7 = 0;
    int cnt7 = 0;
    for (int r = blur7.rows - 1; r >= blur7.rows - 120; --r)
    {
        const uchar *row = bin7.ptr(r);
        for (int c = 0; c < bin7.cols; ++c)
            if (row[c])
            {
                err7 += c - 640;
                cnt7++;
            }
    }
    err7 = cnt7 ? err7 / cnt7 : 0;
    double steer7 = -0.001 * err7;

    {
        float vx = 0.25f, vy = 0.f;
        applyRangeClearance(ob_x, ob_y, ob_z, vx, vy);
        sc.Move(vx, vy, steer7);
    }

    // 此处留给后续台阶检测触发条件（Flag_Task = 3）
}