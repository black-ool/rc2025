#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/common/time/time_tool.hpp>

#include <opencv2/opencv.hpp>

#include <chrono>
#include <iostream>
#include <thread>
#include <deque>

#include "params.h"
#include "globals.h"
#include "utils.h"
#include "visualizer.h"
#include "aruco_server.h"
#include "app_runtime.h"
#include "cases/case0.h"
#include "cases/case1.h"
#include "cases/case2.h"
#include "cases/case3.h"
#include "cases/case4.h"

using namespace unitree::robot;
using namespace cv;
using namespace std;

// =============================================================================
// 主循环
// =============================================================================
static int runMainLoop(AppRuntime &rt)
{
    go2::SportClient &sc = rt.sc;
    VideoCapture &cap = rt.cap;

    Mat frame, undist;
    int fcount = 0;
    auto t0 = chrono::steady_clock::now();

    while (true)
    {
        // ---------- 图像采集与去畸变 ----------
        if (!cap.read(frame) || frame.empty())
            break;
        fcount++;
        undistort(frame, undist, K, D);

        // ---------- 状态日志 ----------
        cout << " Flag_Task: " << Flag_Task
             << "|ob_x: " << ob_x << "|ob_y: " << ob_y << "|ob_z: " << ob_z
             << "|px: " << px << "|py: " << py << "|yaw: " << yaw << endl;

        if (g_enable_gui)
        {
            double fps = fcount / chrono::duration<double>(chrono::steady_clock::now() - t0).count();
            putText(undist, format("FPS %.1f", fps), {10, 30},
                    FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);
        }

        // ---------- 相对起点坐标 ----------
        double lx, ly, dyaw;
        transformLocal(px, py, yaw, lx, ly, dyaw);
        (void)ly;
        (void)dyaw;

        // ---------- 状态机调度 ----------
        switch (Flag_Task)
        {
        case 0:
            if (case0_tick(sc, undist, rt.stateCB.state, lx))
                Flag_Task = 1;
            break;
        case 1:
            if (case1_tick(sc))
                Flag_Task = 2;
            break;
        case 2:
            case2_tick(sc, undist);
            break;
        case 3:
            case3_tick(sc);
            break;
        case 4:
            if (case4_tick(sc))
                return 0;
            break;
        }

        // ---------- GUI ----------
        if (g_enable_gui)
        {
            static std::deque<float> range_hist_x, range_hist_y, range_hist_z;
            pushRangeSampleRaw(range_hist_x, ob_x);
            pushRangeSampleRaw(range_hist_y, ob_y);
            pushRangeSampleRaw(range_hist_z, ob_z);
            imshow("Range ob_x | ob_y+ob_z (raw)",
                   makeRangePlotMat(range_hist_x, range_hist_y, range_hist_z));
            imshow("Go2 Front Cam", undist);
            if (waitKey(1) == 27)
                break;
        }
    }
    sc.StopMove();
    return 0;
}

// =============================================================================
// 入口
// =============================================================================
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        cerr << "Usage: " << argv[0] << " <ethernet_if> [--gui]\n"
             << "  --gui   显示雷达曲线与前视窗口（需桌面或 X11）；默认关闭以便无 DISPLAY 运行\n";
        return -1;
    }

    const char *eth_if = argv[1];
    for (int i = 2; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--gui")
            g_enable_gui = true;
        else
        {
            cerr << "Unknown option: " << argv[i] << "\n";
            return -1;
        }
    }

    ChannelFactory::Instance()->Init(0, eth_if);
    AppRuntime rt;
    if (!initAppRuntime(rt, eth_if))
    {
        cerr << "Front camera stream not opened\n";
        return -1;
    }

    std::thread aruco_thread(aruco_socket_server, 5005);
    aruco_thread.detach();

    if (g_enable_gui)
        cout << "GUI enabled (ESC in video window to quit)\n";
    else
        cout << "GUI disabled (headless); use Ctrl+C to stop\n";

    return runMainLoop(rt);
}