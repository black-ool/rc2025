#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/common/time/time_tool.hpp>

#include <opencv2/opencv.hpp>

#include <chrono>
#include <iostream>
#include <thread>
#include <csignal>
#include <atomic>

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
// Ctrl+C 安全退出
// =============================================================================
static atomic<bool> g_exit_requested(false);

void signalHandler(int sig)
{
    if (sig == SIGINT)
    {
        cout << "\n[SIGINT] Ctrl+C caught, will exit after cleanup..." << endl;
        g_exit_requested = true;
    }
}

// =============================================================================
// 主循环
// =============================================================================
static int runMainLoop(AppRuntime &rt)
{
    go2::SportClient &sc = rt.sc;
    go2::ObstaclesAvoidClient &avoid_client = rt.avoid_client;
    VideoCapture &cap = rt.cap;

    Mat frame, undist;
    int fcount = 0;
    auto t0 = chrono::steady_clock::now();

    while (!g_exit_requested)
    {
        // ---------- 图像采集与去畸变 ----------
        if (!cap.read(frame) || frame.empty())
            break;
        fcount++;
        undistort(frame, undist, K, D);

        // ---------- 状态日志 (每30帧) ----------
        if (fcount % 30 == 0)
        {
            double lx, ly, dyaw;
            transformLocal(px, py, yaw, lx, ly, dyaw);
            cout << "[Status] Flag=" << Flag_Task << " lx=" << lx << " ly=" << ly
                 << " yaw=" << dyaw << " px=" << px << " py=" << py << endl;
        }

        // ---------- FPS 叠加 ----------
        double fps = fcount / chrono::duration<double>(chrono::steady_clock::now() - t0).count();
        putText(undist, format("FPS %.1f", fps), {10, 30},
                FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);

        // ---------- 相对起点坐标 ----------
        double lx, ly, dyaw;
        transformLocal(px, py, yaw, lx, ly, dyaw);

        // ---------- Debug 显示 ----------
        static Mat display_img = undist.clone();

        // ===================== 主状态机 FSM =====================
        if (g_force_task >= 0)
            Flag_Task = g_force_task;
        switch (Flag_Task)
        {
        // ---- case 0: 巡线 ----
        case 0:
        {
            int case0_ret = case0_tick(sc, undist, rt.stateCB.state, fcount);
            display_img = undist.clone();
            if (case0_ret == 1)
            {
                Flag_Task = 1;
                g_case0_second_pass = false;  // 第一段巡线结束
                case1_reset_statics();
            }
            else if (case0_ret == 2)
            {
                Flag_Task = 2;
                g_case0_second_pass = false;  // 重置标记
                case2_reset();
            }
            break;
        }

        // ---- case 1: S型走廊避障 ----
        case 1:
        {
            bool back_to_0 = case1_tick(sc, fcount, lx, ly, yaw);
            display_img = undist.clone();
            if (back_to_0)
            {
                Flag_Task = 0;
                g_case0_second_pass = true;  // 第二段巡线：case1 完成后返回
                case0_reset_statics();
            }
            break;
        }

        // ---- case 2: ArUco 检测 + 左转90° ----
        case 2:
        {
            bool to_case3 = case2_tick(sc);
            if (to_case3)
            {
                Flag_Task = 3;
            }
            break;
        }

        // ---- case 3: 前进找下一个 ArUco → Flag_Task=4~8 ----
        case 3:
        {
            bool to_case9 = case3_tick(sc, lx, ly, dyaw);
            if (to_case9)
            {
                Flag_Task = 9;
            }
            break;
        }

        // ---- case 4~8 合并在 case3 内部子状态处理 ----
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        {
            // 这些 case 在旧代码中是独立的 while(true) 阻塞式，但这里统一走 case3
            // 如果外部仍有 Flag_Task 设为这些值，也走 case3
            bool to_case9 = case3_tick(sc, lx, ly, dyaw);
            if (to_case9)
            {
                Flag_Task = 9;
            }
            break;
        }

        // ---- case 9: 完成 + 恢复遥控 ----
        case 9:
        {
            if (case4_tick(sc, avoid_client))
            {
                cout << "[Exit] Mission complete, exiting main loop." << endl;
                return 0;
            }
            break;
        }

        default:
            break;
        }

        // ---------- GUI ----------
        if (g_enable_gui)
        {
            imshow("Go2 Front Cam - Visual Nav", display_img);
            if (waitKey(1) == 27 || g_exit_requested)
                break;
        }
    }

    // 退出清理
    cout << "[Exit] Cleaning up and restoring remote control..." << endl;
    sc.StopMove();
    avoid_client.UseRemoteCommandFromApi(false);
    avoid_client.SwitchSet(false);
    avoid_client.Move(0, 0, 0);
    this_thread::sleep_for(chrono::milliseconds(200));
    sc.SwitchJoystick(true);
    sc.RecoveryStand();
    this_thread::sleep_for(chrono::milliseconds(500));
    sc.BalanceStand();
    cout << "[Exit] Remote control restored. Goodbye." << endl;

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
             << "  --gui   显示前视窗口（需桌面或 X11）；默认关闭以便无 DISPLAY 运行\n";
        return -1;
    }

    const char *eth_if = argv[1];
    for (int i = 2; i < argc; ++i)
    {
        string arg = argv[i];
        if (arg == "--gui")
            g_enable_gui = true;
        else if (arg == "--task" && i + 1 < argc)
        {
            g_force_task = atoi(argv[++i]);
            if (g_force_task == 0)
                g_case0_skip_init = true;  // --task 0: 跳过跳跃，直接纯巡线
            cout << "[Config] Force task mode: Flag_Task locked to " << g_force_task << endl;
        }
        else
        {
            cerr << "Unknown option: " << arg << "\n";
            return -1;
        }
    }

    // 注册 SIGINT 信号处理器
    signal(SIGINT, signalHandler);

    /* Init Unitree DDS */
    ChannelFactory::Instance()->Init(0, eth_if);

    AppRuntime rt;
    if (!initAppRuntime(rt, eth_if))
    {
        cerr << "Front camera stream not opened\n";
        return -1;
    }

    // 保存初始位姿 (跳跃前)
    px0 = px;
    py0 = py;
    yaw0 = yaw;

    // 启动 ArUco socket 服务线程
    thread aruco_thread(aruco_socket_server, 5005);
    aruco_thread.detach();

    if (g_enable_gui)
        cout << "GUI enabled (ESC in video window to quit)\n";
    else
        cout << "GUI disabled (headless); use Ctrl+C to stop\n";

    return runMainLoop(rt);
}