#include <opencv2/opencv.hpp>

// #include <opencv2/aruco.hpp>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/ros2/PointStamped_.hpp>

#include <chrono>
#include <cmath>
#include <algorithm>
#include <deque>
#include <iostream>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <atomic>

#define TOPIC_RANGE_INFO "rt/utlidar/range_info"
#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;
using namespace cv;
using namespace std;

// =============================================================================
// 标定与话题（相机内参、DDS 话题名）
// =============================================================================
/** 前视 RGB 相机内参与畸变（请按实机标定替换） */
static const Mat K = (Mat_<double>(3, 3) << 929.7797, 0, 629.6662,
                      0, 926.7584, 335.6207,
                      0, 0, 1);
static const Mat D = (Mat_<double>(1, 4) << -0.4157, 0.1327, 0, 0);

// =============================================================================
// 可调参数（仅 constexpr，便于集中标定；勿在此处声明运行时变量）
// =============================================================================
// --- 雷达滤波 ---
/** 测距 EMA 系数，越大越跟手、越小越平滑 */
static constexpr float kRangeEmaAlpha = 0.35f;

// --- case0 → case1：窄过道入门（约 0.6 m 宽 → 居中时左右约 0.3 m）---
/** 起跳后沿局部前向 lx 至少前进该距离后才允许雷达触发进入 case1，防止落点误触发 */
static constexpr double kCase0PostJumpMinForward_m = 0.45;
/** 单侧距离落在该区间 (m) 视为贴近过道一侧墙（居中） */
static constexpr float kEnterMazeSideLo_m = 0.18f;
static constexpr float kEnterMazeSideHi_m = 0.48f;
/** 左右之和在该区间 (m) 视为窄过道截面 */
static constexpr float kEnterMazeSideSumLo_m = 0.45f;
static constexpr float kEnterMazeSideSumHi_m = 0.88f;
static constexpr int kEnterMazeStableFrames = 4;

// --- case1：迷宫 —— 两次 180° 掉头；第 1 次路口判定同前+左转；第 2 次前进判定路口+右转（稳定帧同第一次）---
static constexpr int kCase1NumUTurns = 2;
/** 前方视为「贴墙/路口」的距离上限 (m)，略宽于半宽以抑制噪声 */
static constexpr float kMazeJunctionFront_m = 0.45f;
/** 该侧距离大于此值 (m) 视为路口一侧明显开阔，可转 90° */
static constexpr float kMazeSideOpen_m = 0.52f;
/** 路口条件连续满足的帧数 */
static constexpr int kMazeJunctionStableFrames = 3;
/** 刚进入 case1 后若干帧内禁止第 1 次路口判定，避免尚未完全进入窄道就掉头 */
static constexpr int kCase1PostEntryDelayFrames = 55;
/** 完成转弯后的冷却帧，避免同一路口重复触发 */
static constexpr int kMazeTurnCooldownFrames = 20;
/** 180° 弧线耗时更长 */
static constexpr int kMazeTurnTimeoutFrames = 700;
/** 两次 180° 后第 3 次路口判定时左转 90° 弧线的超时帧数 */
static constexpr int kMazeArc90TimeoutFrames = 500;
/** 过道内横偏速度上限与增益（左右差 → vy） */
static constexpr float kMazeCorridorVyGain = 0.42f;
static constexpr float kMazeCorridorVyClamp = 0.12f;
static constexpr float kMazeForwardVx = 0.20f;
/** 路口一次 180° 掉头：转弯半径 R (m)，协调关系 |ω|≈vx/R（Move 第三参按 rad/s 量级） */
static constexpr double kCase1UTurnRadius_m = 0.6;
static constexpr float kCase1UTurnForwardVx = 0.15f;
/** 第 1 次 180°（左转）略降前向、略增大等效转弯半径，并微左偏，减轻蹭外侧墙 */
static constexpr float kCase1FirstUTurnVxScale = 0.82f;
static constexpr double kCase1FirstUTurnRadiusScale = 1.12;
static constexpr float kCase1FirstUTurnVyBias = 0.04f;
/** 第 2 次 180°（右转）略降前向、略增大等效半径，并微右偏，减轻蹭后方/侧墙 */
static constexpr float kCase1SecondUTurnVxScale = 0.85f;
static constexpr double kCase1SecondUTurnRadiusScale = 1.10;
static constexpr float kCase1SecondUTurnVyBias = -0.045f;
// --- 原地转弯（Sport Move 第三分量） ---
static constexpr double kYawTurnGain = 4.8;
static constexpr double kYawTurnCmdClamp = 1.25;
/** 克服控制死区的最小角速度指令幅值 */
static constexpr double kYawTurnCmdFloor = 0.55;
/** 认为转弯到位的最大航向误差 (rad) */
static constexpr double kYawTurnDoneErrRad = 0.10;

// --- 雷达安全裕量：低于阈值或 inf 时向反方向平移/减速（使用原始 ob_* 判定 inf）---
static constexpr float kClearMinSide_m = 0.2f;
static constexpr float kClearMinFront_m = 0.4f;
static constexpr float kClearSideGain = 3.0f;
static constexpr float kClearFrontGain = 2.0f;
/** 侧向修正上限（与 Move 第二参数一致：左近减 vy、右近加 vy） */
static constexpr float kClearVyAbsMax = 0.38f;
/** 前向过近时允许的最大后退分量（vx 可减至 -该值） */
static constexpr float kClearVxRetreatMax = 0.28f;

// =============================================================================
// 全局状态（传感器、位姿、任务机）
// =============================================================================
float ob_x = 0, ob_y = 0, ob_z = 0;       // 雷达原始测距 (PointStamped x/y/z)
float ob_x_f = 0, ob_y_f = 0, ob_z_f = 0; // 滤波后测距，用于控制
double px = 0, py = 0, yaw = 0;           // 机体世界系位姿（来自 sportmodestate）
double px0 = 0, py0 = 0, yaw0 = 0;        // 程序启动时刻位姿，用于 transformLocal
int Flag_Task = 0;                        // 主状态机 case 编号

int start_jump_times = 0;   // 起点前跳次数
int end_jump_times = 0;     // 终点前跳次数
bool found_turn = false;

/** case1：0 巡航；1 执行 180° 掉头；2 执行左转 90° 弧线 */
int g_maze_nav = 0;
double g_maze_turn_target = 0.;
int g_maze_junc_stable = 0;
int g_maze_turn_cd = 0;
int g_maze_turn_frm = 0;
/**
 * case1 已完成 180° 掉头次数：0→未做；1→已完成第 1 次（左转 180°）；2→已完成第 2 次（右转 180°）。
 * 触发下一机动时严格按该计数绑定：仅 seg==0 启动左转 180°，仅 seg==1 启动右转 180°，仅 seg==2 且未做 90° 时启动左转 90°。
 */
int g_case1_seg = 0;
/** 两次 180° 后第 3 次路口判定触发的左转 90° 是否已执行 */
bool g_case1_left90_done = false;
/** 两次 180° 完成后，路口判定连续满足的帧计数（用于第 3 次触发 90°） */
int g_case1_post90_stable = 0;
/** 进入 case1 后倒计时：>0 时禁止「第 1 次」180° 的路口累加与触发（每帧减 1） */
int g_case1_entry_delay_frm = 0;
/**
 * 当前协调弧线的角速度方向（勿用 err 符号代替）：+1 左转/CCW，-1 右转/CW。
 * yaw±π 在角度上为同一点，wrapAngle 后最短路径 err 总同号，会导致两次 180° 实际同向旋转。
 */
int g_case1_coord_w_sign = 0;
/** 迷宫 case1 三段完成后进入 case2 时先仅停车占位（后续再接台阶/巡线逻辑） */
bool g_case2_post_maze_placeholder = false;

std::atomic<int> g_last_aruco_id(-1);
/** 为 true 时显示 OpenCV 窗口；默认 false，便于无 DISPLAY 的机载环境 */
static bool g_enable_gui = false;

// =============================================================================
// 辅助线程：ArUco ID 经 TCP 写入 g_last_aruco_id
// =============================================================================
void aruco_socket_server(int port = 5005)
{
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr("127.0.0.1");
    address.sin_port = htons(port);
    bind(server_fd, (struct sockaddr *)&address, sizeof(address));
    listen(server_fd, 1);
    std::cout << "Aruco socket server listening on 127.0.0.1:" << port << std::endl;
    while (true)
    {
        int client_fd = accept(server_fd, nullptr, nullptr);
        char buffer[32];
        while (true)
        {
            ssize_t len = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
            if (len > 0)
            {
                buffer[len] = '\0';
                int id = atoi(buffer);
                g_last_aruco_id = id;
                std::cout << "Received aruco id: " << id << std::endl;
            }
            else if (len == 0)
            {
                // client disconnected
                close(client_fd);
                break;
            }
        }
    }
    close(server_fd);
}

// =============================================================================
// DDS 回调（其它线程/通道触发；勿在此做耗时阻塞）
// =============================================================================
void rangeCB(const void *m)
{
    auto *p = (const geometry_msgs::msg::dds_::PointStamped_ *)m;
    const float rx = p->point().x();
    const float ry = p->point().y();
    const float rz = p->point().z();
    ob_x = rx;
    ob_y = ry;
    ob_z = rz;

    static bool have_x = false, have_y = false, have_z = false;
    auto ema = [](float raw, float &out, bool &have) {
        if (!std::isfinite(raw))
            return;
        if (!have)
        {
            out = raw;
            have = true;
        }
        else
            out = out * (1.f - kRangeEmaAlpha) + raw * kRangeEmaAlpha;
    };
    ema(rx, ob_x_f, have_x);
    ema(ry, ob_y_f, have_y);
    ema(rz, ob_z_f, have_z);
}

class StateCB
{
public:
    unitree_go::msg::dds_::SportModeState_ state;
    void operator()(const void *m)
    {
        state = *(const unitree_go::msg::dds_::SportModeState_ *)m;
        px = state.position()[0];
        py = state.position()[1];
        yaw = state.imu_state().rpy()[2];
    }
};

/** 将世界系位姿转换到以 (px0,py0,yaw0) 为原点的局部前向坐标系 */
static inline void transformLocal(double x, double y, double yaw_now,
                                  double &lx, double &ly, double &dyaw)
{
    double c = cos(yaw0), s = sin(yaw0);
    lx = (x - px0) * c + (y - py0) * s;
    ly = -(x - px0) * s + (y - py0) * c;
    dyaw = yaw_now - yaw0;
    if (dyaw > M_PI)
        dyaw -= 2 * M_PI;
    if (dyaw < -M_PI)
        dyaw += 2 * M_PI;
}

// =============================================================================
// 工具函数（纯计算 / 无 DDS；可配合单元测试）
// =============================================================================
// Add PID_Yaw and PID_Yaw1 functions from v1_code.cpp
float PID_Yaw(float expect, float err)
{
    static float integral = 0, error_last = 0;
    float p = 5.0, i = 0, d = 0;
    float error_current = err - expect;
    integral += error_current;
    float output = -(p * error_current + i * integral + d * (error_current - error_last));
    error_last = error_current;
    return std::max(-2.0f, std::min(2.0f, output));
}

float PID_Yaw1(float expect, float err)
{
    static float integral = 0, error_last = 0;
    float p = 0.025, i = 0, d = 0;
    float error_current = err - expect;
    integral += error_current;
    float output = -(p * error_current + i * integral + d * (error_current - error_last));
    error_last = error_current;
    return std::max(-2.0f, std::min(2.0f, output));
}

static inline float safeRange(float v, float fallback = 5.0f)
{
    if (!std::isfinite(v))
        return fallback;
    return std::fabs(v);
}

static inline double wrapAngle(double a)
{
    while (a > M_PI)
        a -= 2 * M_PI;
    while (a <= -M_PI)
        a += 2 * M_PI;
    return a;
}

/** Sport Move 第三参数：目标 yaw 误差(rad) -> 建议角速度，需配合 StaticWalk 使用 */
static inline double yawTurnRateCmd(double err_rad)
{
    double c = err_rad * kYawTurnGain;
    if (std::fabs(c) < kYawTurnCmdFloor && std::fabs(err_rad) > 0.05)
        c = (err_rad > 0.0 ? kYawTurnCmdFloor : -kYawTurnCmdFloor);
    return std::max(-kYawTurnCmdClamp, std::min(kYawTurnCmdClamp, c));
}

/** 进入 case1 时清空迷宫子状态 */
static inline void resetMazeFsm()
{
    g_maze_nav = 0;
    g_maze_turn_target = 0.;
    g_maze_junc_stable = 0;
    g_maze_turn_cd = 0;
    g_maze_turn_frm = 0;
    g_case1_seg = 0;
    g_case1_left90_done = false;
    g_case1_post90_stable = 0;
    g_case1_entry_delay_frm = kCase1PostEntryDelayFrames;
    g_case1_coord_w_sign = 0;
    g_case2_post_maze_placeholder = false;
}

/** 滤波距离均落在窄过道「居中」区间，或左右之和符合约 0.6 m 过道截面 */
static inline bool isInNarrowCorridor(float left, float right)
{
    const bool band_lr = (left >= kEnterMazeSideLo_m && left <= kEnterMazeSideHi_m && right >= kEnterMazeSideLo_m
                          && right <= kEnterMazeSideHi_m);
    const float s = left + right;
    const bool sum_ok =
        (s >= kEnterMazeSideSumLo_m && s <= kEnterMazeSideSumHi_m && std::min(left, right) > 0.12f);
    return band_lr || sum_ok;
}

/** 路口：前向较近，且至少一侧明显开阔（与地图说明一致） */
static inline bool isMazeJunctionCandidate(float front, float left, float right)
{
    if (!(front < kMazeJunctionFront_m))
        return false;
    return (left > kMazeSideOpen_m) || (right > kMazeSideOpen_m);
}

/**
 * 开始一次 180° 掉头：turn_left=true 为左转（+π），false 为右转（-π）。
 * 目标航向 yaw±π 在圆上为同一点，故角速度方向由 g_case1_coord_w_sign 记录，勿仅靠 err 符号。
 */
static inline void case1BeginUTurn(bool turn_left)
{
    g_maze_turn_target = wrapAngle(yaw + (turn_left ? M_PI : -M_PI));
    g_case1_coord_w_sign = turn_left ? 1 : -1;
    g_maze_nav = 1;
    g_maze_turn_frm = 0;
}

/** 两次 180° 后第 3 次路口判定时：左转 90° 协调弧线，R 同 kCase1UTurnRadius_m */
static inline void case1BeginLeft90Arc()
{
    g_maze_turn_target = wrapAngle(yaw + M_PI / 2.0);
    g_case1_coord_w_sign = 1;
    g_maze_nav = 2;
    g_maze_turn_frm = 0;
}

/** 第二次掉头前的路口触发：与第一次相同的判定（isMazeJunctionCandidate + 同稳定帧）；可改为纯前向 */
static inline bool isCase1SecondJunctionTrigger(float front, float left, float right)
{
    return isMazeJunctionCandidate(front, left, right);
}

static inline float rawRangeMag(float r)
{
    return std::isfinite(r) ? std::fabs(r) : 0.f;
}

/** 直行阶段仅用前向安全距离修正 vx（不参与左右侧距） */
static inline void applyRangeClearanceFrontOnly(float raw_front, float &vx)
{
    const float dF = rawRangeMag(raw_front);
    if (!std::isfinite(raw_front) || dF < kClearMinFront_m)
    {
        const float deficit =
            !std::isfinite(raw_front) ? kClearMinFront_m : (kClearMinFront_m - dF);
        vx -= std::min(kClearVxRetreatMax, deficit * kClearFrontGain);
        vx = std::max(-kClearVxRetreatMax, vx);
    }
}

/**
 * 在 Sport Move(vx, vy, w) 下叠加安全修正：左 ob_y 过小或无效 → vy 减小（向机体右侧平移）；
 * 右 ob_z 过小或无效 → vy 增大；前 ob_x 过小或无效 → 减小 vx（可后退）直至回到阈值以上。
 * 应在写出每一帧期望 vx、vy 之后调用。
 */
static inline void applyRangeClearance(float raw_front, float raw_left, float raw_right, float &vx,
                                       float &vy)
{
    const float dL = rawRangeMag(raw_left);
    const float dR = rawRangeMag(raw_right);
    const float dF = rawRangeMag(raw_front);

    if (!std::isfinite(raw_left) || dL < kClearMinSide_m)
    {
        const float deficit =
            !std::isfinite(raw_left) ? kClearMinSide_m : (kClearMinSide_m - dL);
        vy -= std::min(kClearVyAbsMax, deficit * kClearSideGain);
    }
    if (!std::isfinite(raw_right) || dR < kClearMinSide_m)
    {
        const float deficit =
            !std::isfinite(raw_right) ? kClearMinSide_m : (kClearMinSide_m - dR);
        vy += std::min(kClearVyAbsMax, deficit * kClearSideGain);
    }
    vy = std::max(-kClearVyAbsMax, std::min(kClearVyAbsMax, vy));

    if (!std::isfinite(raw_front) || dF < kClearMinFront_m)
    {
        const float deficit =
            !std::isfinite(raw_front) ? kClearMinFront_m : (kClearMinFront_m - dF);
        vx -= std::min(kClearVxRetreatMax, deficit * kClearFrontGain);
        vx = std::max(-kClearVxRetreatMax, vx);
    }
}

/** case1 巡航一帧：需 StaticWalk 与 Move 配合，否则 StopMove 后可能不再前进 */
static inline void case1CorridorFollowFrame(go2::SportClient &sc, float left, float right, float side_sum)
{
    sc.StaticWalk();
    float vx = kMazeForwardVx;
    float vy = 0.f;
    if (side_sum > 0.05f)
        vy = std::max(-kMazeCorridorVyClamp,
                      std::min(kMazeCorridorVyClamp, (left - right) * kMazeCorridorVyGain));
    applyRangeClearance(ob_x, ob_y, ob_z, vx, vy);
    sc.Move(vx, vy, 0.f);
}

/**
 * 180° 协调掉头单帧（|ω|≈vx/R）；依赖 g_maze_turn_target、g_maze_turn_frm。
 * @return 0 进行中；1 到位；2 超时
 */
static inline int case1UTurnTickFrame(go2::SportClient &sc)
{
    sc.StaticWalk();
    const double err = wrapAngle(g_maze_turn_target - yaw);
    ++g_maze_turn_frm;

    if (std::fabs(err) <= kYawTurnDoneErrRad)
    {
        sc.StopMove();
        g_maze_turn_frm = 0;
        g_maze_turn_cd = kMazeTurnCooldownFrames;
        g_maze_nav = 0;
        g_maze_junc_stable = 0;
        return 1;
    }
    if (g_maze_turn_frm > kMazeTurnTimeoutFrames)
    {
        sc.StopMove();
        g_maze_nav = 0;
        g_maze_turn_frm = 0;
        g_maze_turn_cd = kMazeTurnCooldownFrames * 2;
        return 2;
    }

    float vx = kCase1UTurnForwardVx;
    if (g_case1_seg == 0)
        vx *= kCase1FirstUTurnVxScale;
    else if (g_case1_seg == 1)
        vx *= kCase1SecondUTurnVxScale;
    applyRangeClearanceFrontOnly(ob_x, vx);
    const double vx_use = std::max((double)vx, 1e-4);
    double R_eff = kCase1UTurnRadius_m;
    if (g_case1_seg == 0)
        R_eff *= kCase1FirstUTurnRadiusScale;
    else if (g_case1_seg == 1)
        R_eff *= kCase1SecondUTurnRadiusScale;
    double w_mag = vx_use / R_eff;
    if (w_mag < kYawTurnCmdFloor)
        w_mag = kYawTurnCmdFloor;
    w_mag = std::min(w_mag, kYawTurnCmdClamp);
    const int w_dir = (g_case1_coord_w_sign >= 0 ? 1 : -1);
    float vy = 0.f;
    if (g_case1_seg == 0 && g_case1_coord_w_sign > 0)
        vy = kCase1FirstUTurnVyBias;
    else if (g_case1_seg == 1 && g_case1_coord_w_sign < 0)
        vy = kCase1SecondUTurnVyBias;
    const double w_cmd = (double)w_dir * w_mag;
    sc.Move(vx, vy, w_cmd);
    return 0;
}

/**
 * 左转 90° 协调弧线单帧：|ω|≈vx/R，R=kCase1UTurnRadius_m。
 * @return 0 进行中；1 到位；2 超时
 */
static inline int case1Arc90LeftTickFrame(go2::SportClient &sc)
{
    sc.StaticWalk();
    const double err = wrapAngle(g_maze_turn_target - yaw);
    ++g_maze_turn_frm;

    if (std::fabs(err) <= kYawTurnDoneErrRad)
    {
        sc.StopMove();
        g_maze_turn_frm = 0;
        g_maze_turn_cd = kMazeTurnCooldownFrames;
        g_maze_nav = 0;
        g_maze_junc_stable = 0;
        g_case1_post90_stable = 0;
        g_case1_left90_done = true;
        return 1;
    }
    if (g_maze_turn_frm > kMazeArc90TimeoutFrames)
    {
        sc.StopMove();
        g_maze_nav = 0;
        g_maze_turn_frm = 0;
        g_maze_turn_cd = kMazeTurnCooldownFrames * 2;
        g_case1_left90_done = true;
        return 2;
    }

    float vx = kCase1UTurnForwardVx;
    applyRangeClearanceFrontOnly(ob_x, vx);
    const double vx_use = std::max((double)vx, 1e-4);
    double w_mag = vx_use / kCase1UTurnRadius_m;
    if (w_mag < kYawTurnCmdFloor)
        w_mag = kYawTurnCmdFloor;
    w_mag = std::min(w_mag, kYawTurnCmdClamp);
    const int w_dir = (g_case1_coord_w_sign >= 0 ? 1 : -1);
    const double w_cmd = (double)w_dir * w_mag;
    sc.Move(vx, 0.f, w_cmd);
    return 0;
}

// =============================================================================
// 可视化（仅 OpenCV 实时曲线，不参与控制决策）
// =============================================================================
namespace {

constexpr size_t kRangePlotSamples = 600;
constexpr int kPlotW = 960;
constexpr int kPlotH = 420;

/** Append raw sensor value (no clamp / no sentinel for inf). */
static inline void pushRangeSampleRaw(std::deque<float> &q, float v)
{
    q.push_back(v);
    while (q.size() > kRangePlotSamples)
        q.pop_front();
}

static void autoYAxisFinite(const std::deque<float> &data, float &ymin, float &ymax)
{
    bool any = false;
    float lo = 0.f, hi = 0.f;
    for (float v : data)
    {
        if (!std::isfinite(v))
            continue;
        if (!any)
        {
            lo = hi = v;
            any = true;
        }
        else
        {
            lo = std::min(lo, v);
            hi = std::max(hi, v);
        }
    }
    if (!any)
    {
        ymin = 0.f;
        ymax = 5.f;
        return;
    }
    ymin = lo;
    ymax = hi;
    float span = ymax - ymin;
    float pad = std::max(0.08f, span * 0.12f);
    ymin -= pad;
    ymax += pad;
    if (ymax - ymin < 0.2f)
    {
        ymin -= 0.1f;
        ymax += 0.1f;
    }
}

static void autoYAxisFiniteShared(const std::deque<float> &a, const std::deque<float> &b,
                                  float &ymin, float &ymax)
{
    bool any = false;
    float lo = 0.f, hi = 0.f;
    auto consider = [&](float v) {
        if (!std::isfinite(v))
            return;
        if (!any)
        {
            lo = hi = v;
            any = true;
        }
        else
        {
            lo = std::min(lo, v);
            hi = std::max(hi, v);
        }
    };
    for (float v : a)
        consider(v);
    for (float v : b)
        consider(v);
    if (!any)
    {
        ymin = 0.f;
        ymax = 5.f;
        return;
    }
    ymin = lo;
    ymax = hi;
    float span = ymax - ymin;
    float pad = std::max(0.08f, span * 0.12f);
    ymin -= pad;
    ymax += pad;
    if (ymax - ymin < 0.2f)
    {
        ymin -= 0.1f;
        ymax += 0.1f;
    }
}

/** Polyline skipping non-finite samples (break segments, no invented values). */
static void drawFinitePolyline(Mat &canvas, int ox, int oy_bot, int w, int h, float ymin, float ymax,
                               const std::deque<float> &data, const Scalar &line_color,
                               int n_plot = -1)
{
    std::vector<Point> seg;
    const int n = n_plot > 0 ? std::min(n_plot, static_cast<int>(data.size()))
                             : static_cast<int>(data.size());
    for (int i = 0; i < n; ++i)
    {
        float v = data[static_cast<size_t>(i)];
        if (!std::isfinite(v))
        {
            if (seg.size() >= 2)
                polylines(canvas, seg, false, line_color, 2, LINE_AA);
            seg.clear();
            continue;
        }
        int px = ox + i * w / std::max(1, n - 1);
        int py = oy_bot - cvRound((v - ymin) / (ymax - ymin + 1e-6f) * h);
        seg.push_back(Point(px, py));
    }
    if (seg.size() >= 2)
        polylines(canvas, seg, false, line_color, 2, LINE_AA);
}

static void drawRangeStrip(Mat &canvas, const Rect &band, const std::deque<float> &data,
                           const char *title, const Scalar &line_color)
{
    rectangle(canvas, band, Scalar(32, 32, 32), FILLED);
    putText(canvas, title, Point(band.x + 8, band.y + 22),
            FONT_HERSHEY_SIMPLEX, 0.55, Scalar(220, 220, 220), 1, LINE_AA);

    float ymin = 0, ymax = 5;
    autoYAxisFinite(data, ymin, ymax);

    const int margin_l = 52;
    const int margin_r = 10;
    const int margin_t = 30;
    const int margin_b = 12;
    int ox = band.x + margin_l;
    int oy_top = band.y + margin_t;
    int w = band.width - margin_l - margin_r;
    int h = band.height - margin_t - margin_b;
    int oy_bot = oy_top + h;

    if (w < 4 || h < 4 || data.size() < 2)
    {
        putText(canvas, "(waiting samples...)", Point(ox, oy_top + h / 2),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(140, 140, 140), 1, LINE_AA);
        return;
    }

    for (int gi = 0; gi <= 4; ++gi)
    {
        float g = ymin + (ymax - ymin) * (gi / 4.f);
        int gy = oy_bot - cvRound((g - ymin) / (ymax - ymin + 1e-6f) * h);
        line(canvas, Point(ox, gy), Point(ox + w, gy), Scalar(55, 55, 55), 1, LINE_AA);
        putText(canvas, format("%.2f", g), Point(band.x + 4, gy + 4),
                FONT_HERSHEY_SIMPLEX, 0.35, Scalar(110, 110, 110), 1, LINE_AA);
    }

    drawFinitePolyline(canvas, ox, oy_bot, w, h, ymin, ymax, data, line_color, -1);

    std::string cur_txt = "now: ";
    if (!data.empty() && std::isfinite(data.back()))
        cur_txt += format("%.4f", data.back());
    else if (!data.empty())
        cur_txt += "non-finite";
    else
        cur_txt += "---";
    putText(canvas, cur_txt, Point(ox + w - 240, band.y + 22),
            FONT_HERSHEY_SIMPLEX, 0.5, line_color, 1, LINE_AA);
}

/** ob_y and ob_z in one plot: shared vertical axis from raw finite samples only. */
static void drawRangeStripYzShared(Mat &canvas, const Rect &band, const std::deque<float> &qy,
                                   const std::deque<float> &qz)
{
    rectangle(canvas, band, Scalar(32, 32, 32), FILLED);
    putText(canvas, "ob_y + ob_z (shared Y, raw)", Point(band.x + 8, band.y + 22),
            FONT_HERSHEY_SIMPLEX, 0.55, Scalar(220, 220, 220), 1, LINE_AA);

    float ymin = 0, ymax = 5;
    autoYAxisFiniteShared(qy, qz, ymin, ymax);

    const int margin_l = 52;
    const int margin_r = 10;
    const int margin_t = 30;
    const int margin_b = 12;
    int ox = band.x + margin_l;
    int oy_top = band.y + margin_t;
    int w = band.width - margin_l - margin_r;
    int h = band.height - margin_t - margin_b;
    int oy_bot = oy_top + h;

    const int n = static_cast<int>(std::min(qy.size(), qz.size()));
    if (w < 4 || h < 4 || n < 2)
    {
        putText(canvas, "(waiting samples...)", Point(ox, oy_top + h / 2),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(140, 140, 140), 1, LINE_AA);
        return;
    }

    for (int gi = 0; gi <= 4; ++gi)
    {
        float g = ymin + (ymax - ymin) * (gi / 4.f);
        int gy = oy_bot - cvRound((g - ymin) / (ymax - ymin + 1e-6f) * h);
        line(canvas, Point(ox, gy), Point(ox + w, gy), Scalar(55, 55, 55), 1, LINE_AA);
        putText(canvas, format("%.2f", g), Point(band.x + 4, gy + 4),
                FONT_HERSHEY_SIMPLEX, 0.35, Scalar(110, 110, 110), 1, LINE_AA);
    }

    drawFinitePolyline(canvas, ox, oy_bot, w, h, ymin, ymax, qy, Scalar(80, 255, 140), n);
    drawFinitePolyline(canvas, ox, oy_bot, w, h, ymin, ymax, qz, Scalar(200, 120, 255), n);

    putText(canvas, "ob_y", Point(ox + w - 220, band.y + 22),
            FONT_HERSHEY_SIMPLEX, 0.45, Scalar(80, 255, 140), 1, LINE_AA);
    putText(canvas, "ob_z", Point(ox + w - 120, band.y + 22),
            FONT_HERSHEY_SIMPLEX, 0.45, Scalar(200, 120, 255), 1, LINE_AA);
}

static Mat makeRangePlotMat(const std::deque<float> &qx, const std::deque<float> &qy,
                            const std::deque<float> &qz)
{
    Mat canvas(kPlotH, kPlotW, CV_8UC3, Scalar(20, 20, 20));
    int bh0 = kPlotH / 2;
    int bh1 = kPlotH - bh0;
    drawRangeStrip(canvas, Rect(0, 0, kPlotW, bh0), qx, "ob_x (raw)", Scalar(80, 180, 255));
    drawRangeStripYzShared(canvas, Rect(0, bh0, kPlotW, bh1), qy, qz);
    return canvas;
}

} // namespace

// =============================================================================
// 运行时对象：DDS 订阅、Sport、相机与 main 同生命周期
// =============================================================================
struct AppRuntime {
    ChannelSubscriber<geometry_msgs::msg::dds_::PointStamped_> sub_range;
    StateCB stateCB;
    ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> sub_state;
    go2::SportClient sc;
    VideoCapture cap;

    AppRuntime()
        : sub_range(TOPIC_RANGE_INFO)
        , sub_state(TOPIC_HIGHSTATE)
    {}
};

/** Sport、订阅、初始位姿、GStreamer 前视相机（须先由 main 完成 ChannelFactory::Init） */
static bool initAppRuntime(AppRuntime &rt, const char *eth_if)
{
    // SportClient::Init 必须在任何 ChannelSubscriber::InitChannel 之前完成，
    // 否则 Cyclone DDS 在创建 DataReader 时可能崩溃（与官方 go2_sport_client 示例顺序一致）。
    rt.sc.SetTimeout(10.0f);
    rt.sc.Init();
    rt.sub_range.InitChannel(rangeCB);
    rt.sub_state.InitChannel(rt.stateCB);
    rt.sc.BalanceStand();
    px0 = px;
    py0 = py;
    yaw0 = yaw;
    const std::string gst_front =
        std::string("udpsrc address=230.1.1.1 port=1720 multicast-iface=") + eth_if +
        " ! application/x-rtp, media=video, encoding-name=H264 "
        "! rtph264depay ! h264parse ! avdec_h264 ! videoconvert "
        "! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1";
    return rt.cap.open(gst_front, CAP_GSTREAMER);
}

/** 读帧、去畸变、主 FSM、雷达/相机窗口 */
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

        // ---------- 状态日志（每帧；可后续改为降频或宏开关）----------
        cout << " Flag_Task: " << Flag_Task << "|ob_x: " << ob_x << "|ob_y: " << ob_y << "|ob_z: " << ob_z << "|px: " << px << "|py: " << py << "|yaw: " << yaw << endl;

        if (g_enable_gui)
        {
            double fps = fcount / chrono::duration<double>(chrono::steady_clock::now() - t0).count();
            putText(undist, format("FPS %.1f", fps), {10, 30},
                    FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);
        }

        // ---------- FSM 公共量：相对起点的局部坐标 ----------
        double lx, ly, dyaw;
        transformLocal(px, py, yaw, lx, ly, dyaw);
        (void)ly;
        (void)dyaw;
        //Flag_Task = -1;
        switch (Flag_Task)
        {
        // ----- case0：起点前跳 + 巡线 → 雷达判定进入窄过道（case1）-----
        case 0: /* 起点区 —— jump once, then line following until case1 */
        {
            // Walk forward briefly, then jump once to cross the starting line
            if (start_jump_times == 0)
            {
                sc.StaticWalk();
                {
                    float vx0 = 0.3f, vy0 = 0.f;
                    applyRangeClearance(ob_x, ob_y, ob_z, vx0, vy0);
                    sc.Move(vx0, vy0, 0);
                }
                this_thread::sleep_for(chrono::milliseconds(2500));
                sc.StopMove();
                sc.FrontJump();
                start_jump_times++;
                cout << "跳跃完成" << endl;
                // Wait for the jump to complete before resuming walk
                this_thread::sleep_for(chrono::milliseconds(1500));
                break; // Skip line-following this iteration
            }

            /* 起跳后沿 lx 累计前进距离，与雷达联判进入迷宫 */
            static double lx_anchor_post_jump = 0;
            static bool lx_anchor_post_jump_set = false;
            if (!lx_anchor_post_jump_set)
            {
                lx_anchor_post_jump = lx;
                lx_anchor_post_jump_set = true;
            }
            const double forward_since_jump = lx - lx_anchor_post_jump;

            /* 巡线 */
            /* Basic binary mask & centre-line deviation */
            Mat gray, blur, bin;
            cvtColor(undist, gray, COLOR_BGR2GRAY);
            GaussianBlur(gray, blur, {15, 15}, 0);
            threshold(blur, bin, 50, 255, THRESH_BINARY_INV);

            // compute mean X of white pixels on bottom rows
            double err = 0;
            int cnt = 0;
            for (int r = blur.rows - 1; r >= blur.rows - 120; --r)
            {
                const uchar *row = bin.ptr(r);
                for (int c = 0; c < bin.cols; ++c)
                    if (row[c])
                    {
                        err += c - 640;
                        cnt++;
                    }
            }
            err = cnt ? err / cnt : 0;

            double steer = -0.001 * err; // pixel→rad gain

            // Enter static walk mode
            sc.StaticWalk();
            sc.Euler(0, 0.25, 0);

            {
                float vx = 0.25f, vy = 0.f;
                applyRangeClearance(ob_x, ob_y, ob_z, vx, vy);
                sc.Move(vx, vy, steer);
            }

            const float f0 = safeRange(ob_x_f);
            const float l0 = safeRange(ob_y_f);
            const float r0 = safeRange(ob_z_f);
            static int case0_enter_maze_cnt = 0;
            if (forward_since_jump >= kCase0PostJumpMinForward_m && isInNarrowCorridor(l0, r0))
                case0_enter_maze_cnt++;
            else
                case0_enter_maze_cnt = 0;

            if (case0_enter_maze_cnt >= kEnterMazeStableFrames)
            {
                case0_enter_maze_cnt = 0;
                resetMazeFsm();
                cout << "[case0] 进入窄过道/迷宫 区段 (lx_fwd=" << forward_since_jump << " m, f=" << f0
                     << " L=" << l0 << " R=" << r0 << ") → Flag_Task=1" << endl;
                Flag_Task = 1;
            }
        }
        break;

        // ----- case1：两次 180°；两次后第 3 次路口判定时左转 90° 弧线(R=0.6m) -----
        case 1:
        {
            const float front = safeRange(ob_x_f);
            const float left = safeRange(ob_y_f);
            const float right = safeRange(ob_z_f);
            const float side_sum = left + right;

            sc.Euler(0, 0, 0);

            if (g_maze_nav == 0)
            {
                if (g_maze_turn_cd > 0)
                    --g_maze_turn_cd;

                if (g_case1_entry_delay_frm > 0)
                {
                    --g_case1_entry_delay_frm;
                    g_maze_junc_stable = 0;
                }

                const bool cd_ok = (g_maze_turn_cd == 0);
                bool started_maneuver = false;

                /* 仅第 1 次 180° 受入门延时；seg>=1 后不再等待 */
                const bool first_uturn_armed =
                    (g_case1_seg > 0) || (g_case1_entry_delay_frm == 0);
                const bool detect_junction =
                    (g_case1_seg < kCase1NumUTurns) && cd_ok && first_uturn_armed;

                const bool cand_first =
                    detect_junction && (g_case1_seg == 0) && isMazeJunctionCandidate(front, left, right);
                const bool cand_second =
                    detect_junction && (g_case1_seg == 1) && isCase1SecondJunctionTrigger(front, left, right);

                if (cand_first || cand_second)
                    ++g_maze_junc_stable;
                else
                    g_maze_junc_stable = 0;

                if (detect_junction && g_maze_junc_stable >= kMazeJunctionStableFrames)
                {
                    sc.StopMove();
                    g_maze_junc_stable = 0;
                    /* 顺序写死：仅 case 0→左转 180°；仅 case 1→右转 180°；无 if-else 链歧义 */
                    switch (g_case1_seg)
                    {
                    case 0:
                        case1BeginUTurn(true);
                        started_maneuver = true;
                        cout << "[case1] 第1次路口触发 (front=" << front << " L=" << left << " R=" << right
                             << ") → 180° R=" << kCase1UTurnRadius_m << "m 左转 目标yaw=" << g_maze_turn_target
                             << endl;
                        break;
                    case 1:
                        case1BeginUTurn(false);
                        started_maneuver = true;
                        cout << "[case1] 第2次路口触发（前进判定与第一次相同）(front=" << front << " L=" << left
                             << " R=" << right << ") → 180° R=" << kCase1UTurnRadius_m << "m 右转 目标yaw="
                             << g_maze_turn_target << endl;
                        break;
                    default:
                        break;
                    }
                }

                /* 两次 180° 后：前方第 3 次满足路口判定（稳定帧）→ 左转 90° 弧线（仅 seg==2） */
                if (!started_maneuver && (g_case1_seg == kCase1NumUTurns) && !g_case1_left90_done && cd_ok)
                {
                    if (isMazeJunctionCandidate(front, left, right))
                        ++g_case1_post90_stable;
                    else
                        g_case1_post90_stable = 0;

                    if (g_case1_post90_stable >= kMazeJunctionStableFrames)
                    {
                        sc.StopMove();
                        g_case1_post90_stable = 0;
                        case1BeginLeft90Arc();
                        started_maneuver = true;
                        cout << "[case1] 第3次路口判定 (front=" << front << " L=" << left << " R=" << right
                             << ") → 左转90°弧线 R=" << kCase1UTurnRadius_m << "m 目标yaw=" << g_maze_turn_target
                             << endl;
                    }
                }

                if (!started_maneuver)
                    case1CorridorFollowFrame(sc, left, right, side_sum);
            }
            else if (g_maze_nav == 1)
            {
                const int ut = case1UTurnTickFrame(sc);
                if (ut == 1)
                {
                    ++g_case1_seg;
                    const double err = wrapAngle(g_maze_turn_target - yaw);
                    cout << "[case1] 第" << g_case1_seg << "次180°掉头完成 err=" << err;
                    if (g_case1_seg >= kCase1NumUTurns)
                        cout << " → 巡航，等待第3次路口判定做90°左弧" << endl;
                    else
                        cout << " → 继续前进，等待下一次路口" << endl;
                    /* 掉头结束已 StopMove；同一帧续走，避免整帧无 Move 体感停车 */
                    case1CorridorFollowFrame(sc, left, right, side_sum);
                }
                else if (ut == 2)
                    cout << "[case1] 第" << (g_case1_seg + 1) << "次掉头超时" << endl;
            }
            else if (g_maze_nav == 2)
            {
                const int ar = case1Arc90LeftTickFrame(sc);
                if (ar == 1)
                {
                    const double err = wrapAngle(g_maze_turn_target - yaw);
                    cout << "[case1] 左转90°弧线完成 err=" << err << " → Flag_Task=2（占位停车）" << endl;
                    g_case2_post_maze_placeholder = true;
                    Flag_Task = 2;
                    /* 弧结束帧内 arc 已 StopMove；立刻发零速 Move，避免整帧无 Move（同掉头后续走） */
                    sc.StaticWalk();
                    sc.Euler(0, 0.25, 0);
                    sc.Move(0.f, 0.f, 0.f);
                }
                else if (ar == 2)
                    cout << "[case1] 左转90°弧线超时" << endl;
            }
        }
        break;

        // ----- case2：巡线接近台阶（原 case7）；迷宫结束后可先仅占位停车 -----
        case 2: /* 巡线准备过台阶 —— 逻辑同 case 0 巡线部分 */
        {
            if (g_case2_post_maze_placeholder)
            {
                /* 勿仅用 StopMove：StaticWalk 下需每帧 Move 才稳，否则易失速后倾 */
                sc.StaticWalk();
                sc.Euler(0, 0.25, 0);
                sc.Move(0.f, 0.f, 0.f);
                break;
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
        break;

        // ----- case3：跳进终点区（原 case8）-----
        case 3: /* 跳进终点区域 */
        {
            if (end_jump_times == 0)
            {
                sc.FrontJump();
                end_jump_times++;
                this_thread::sleep_for(chrono::milliseconds(2500));
            }
            Flag_Task = 4;
        }
        break;

        // ----- case4：任务结束（原 case9）-----
        case 4: /* 终点区 —— stop and celebrate */
            sc.StopMove();
            cout << "Mission complete\n";
            return 0;
        }

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
                break; // ESC to quit
        }
    }
    sc.StopMove();
    return 0;
}

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

    // SportClient 构造即会建立 DDS 请求通道，必须先初始化 ChannelFactory（见 AppRuntime 成员顺序）。
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