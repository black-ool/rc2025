#include <opencv2/opencv.hpp>

// #include <opencv2/aruco.hpp>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/ros2/PointStamped_.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <csignal>

double safeRange(double raw) 
{
    if (raw > 0.1 && raw < 5.0) 
    {
        return raw;
    }
    return 999.0; // 无效值标记
}

#define TOPIC_RANGE_INFO "rt/utlidar/range_info"
#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;
using namespace cv;
using namespace std;

// ---- 全局标志：Ctrl+C 时安全退出 ----
static std::atomic<bool> g_exit_requested(false);

void signalHandler(int sig)
{
    if (sig == SIGINT)
    {
        std::cout << "\n[SIGINT] Ctrl+C caught, will exit after cleanup..." << std::endl;
        g_exit_requested = true;
    }
}

/* Camera intrinsics for the **front** RGB camera (replace with yours) */
static const Mat K = (Mat_<double>(3, 3) << 929.7797, 0, 629.6662,
                      0, 926.7584, 335.6207,
                      0, 0, 1);
static const Mat D = (Mat_<double>(1, 4) << -0.4157, 0.1327, 0, 0);

/* ------------------------------------------------------------------ */
/* -------------------  Global navigation state  -------------------- */
double px = 0, py = 0, yaw = 0;     // body pose
double px0 = 0, py0 = 0, yaw0 = 0;  // pose at start
int Flag_Task = 0;                  // main FSM flag

/* -------------------  Safety Zone  -------------------- */
int start_jump_times = 0;
int end_jump_times = 0;
bool found_turn = false;
int obstacle_avoidance_state = 0;

/* Global variable to store last detected marker id */
std::atomic<int> g_last_aruco_id(-1);

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

/* Utility: convert pose to local frame aligned with (px0,py0,yaw0) */
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

/* ------------------------------------------------------------------ */
/* ------------------  ROS2 / RTDDS Call-backs  --------------------- */
/* Global variables for lidar distances (x=front, y=left, z=right) */
double g_lidar_front_dist = 999.0;  // Front distance (x)
double g_lidar_left_dist  = 999.0;  // Left distance (y)
double g_lidar_right_dist = 999.0;  // Right distance (z)
// EMA-filtered versions for smoother control (alpha=0.35)
double g_lidar_front_f = 999.0;
double g_lidar_left_f  = 999.0;
double g_lidar_right_f = 999.0;
static bool g_have_front = false, g_have_left = false, g_have_right = false;
static constexpr float kRangeEmaAlpha = 0.35f;

void rangeCB(const void *m)
{
    auto *msg = static_cast<const geometry_msgs::msg::dds_::PointStamped_ *>(m);
    double rx = msg->point().x();  // front
    double ry = msg->point().y();  // left
    double rz = msg->point().z();  // right

    // Update raw values (valid range 0.01~10.0m)
    if (rx > 0.01 && rx < 10.0) {
        g_lidar_front_dist = rx;
        if (!g_have_front) { g_lidar_front_f = rx; g_have_front = true; }
        else g_lidar_front_f = g_lidar_front_f * (1.0 - kRangeEmaAlpha) + rx * kRangeEmaAlpha;
    }
    if (ry > 0.01 && ry < 10.0) {
        g_lidar_left_dist = ry;
        if (!g_have_left) { g_lidar_left_f = ry; g_have_left = true; }
        else g_lidar_left_f = g_lidar_left_f * (1.0 - kRangeEmaAlpha) + ry * kRangeEmaAlpha;
    }
    if (rz > 0.01 && rz < 10.0) {
        g_lidar_right_dist = rz;
        if (!g_have_right) { g_lidar_right_f = rz; g_have_right = true; }
        else g_lidar_right_f = g_lidar_right_f * (1.0 - kRangeEmaAlpha) + rz * kRangeEmaAlpha;
    }

    static int count = 0;
    if (++count % 100 == 0) {
        std::cout << "[RANGE] front=" << g_lidar_front_f << " left=" << g_lidar_left_f
                  << " right=" << g_lidar_right_f << " (raw: x=" << rx << " y=" << ry << " z=" << rz << ")" << std::endl;
    }
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

/* ------------------------------------------------------------------ */
/* ---------------  Visual Obstacle Detection  ---------------------- */
/*
 * Detect obstacles in the upper-middle region of the image.
 * Strategy:
 *   - Floor/ground is usually bright and uniform in the lower part
 *   - Obstacles appear as darker, textured regions in the upper-middle
 *   - Use edge density or color variance to detect non-floor regions
 *
 * Returns true if obstacle detected within threshold distance (estimated)
 */
bool detectObstacleVisual(const Mat &frame, double &obstacle_score, Mat &debug_img)
{
    Mat roi = frame.clone();
    debug_img = roi;

    // ROI: upper-middle region (where obstacles would appear)
    // Skip top 1/3 (horizon/sky) and bottom 1/3 (floor close to robot)
    int h = roi.rows;
    int w = roi.cols;
    int y_start = h / 4;       // 25% from top
    int y_end = h * 2 / 3;     // 66% from top
    int x_start = w / 4;       // 25% from left
    int x_end = w * 3 / 4;     // 75% from left (center region)

    Mat roi_region = roi(Rect(x_start, y_start, x_end - x_start, y_end - y_start));

    // Convert to HSV for better color-based detection
    Mat hsv, mask;
    cvtColor(roi_region, hsv, COLOR_BGR2HSV);

    // Detect non-floor regions:
    // Floor is usually bright (high V) and low saturation (low S)
    // Obstacles tend to have more color variation and lower brightness

    // Method 1: Edge density - obstacles have more edges than floor
    Mat gray, edges;
    cvtColor(roi_region, gray, COLOR_BGR2GRAY);
    Canny(gray, edges, 50, 150);

    int edge_pixels = countNonZero(edges);
    int total_pixels = edges.rows * edges.cols;
    double edge_ratio = (double)edge_pixels / total_pixels;

    // Method 2: Brightness variance - obstacles create more variance
    Scalar mean, stddev;
    meanStdDev(gray, mean, stddev);
    double brightness_std = stddev[0];

    // Combine metrics
    obstacle_score = edge_ratio * 100 + brightness_std * 0.5;

    // Draw debug info
    rectangle(debug_img, {x_start, y_start}, {x_end, y_end}, {255, 0, 0}, 2);
    putText(debug_img, format("Edge:%.2f%% Var:%.1f Score:%.1f",
              edge_ratio * 100, brightness_std, obstacle_score),
            {10, h - 20}, FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 255}, 1);

    // Threshold: tune based on testing
    // Higher score = more likely obstacle
    bool obstacle_detected = (edge_ratio > 0.05 || brightness_std > 40);

    return obstacle_detected;
}

/* Simple line-following detection */
bool detectLine(const Mat &undist, double &err, int &cnt, Mat &debug_img)
{
    // 先初始化 debug_img 为 undist 的副本
    debug_img = undist.clone();
    
    Mat gray, blur, bin;
    cvtColor(undist, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, blur, {5, 5}, 0);

    // Detect dark lines: use THRESH_BINARY_INV to highlight DARK regions
    // pixels < threshold become WHITE (255), others become BLACK (0)
    // 降低阈值到更低的值，只检测真正的黑线
    static int debug_threshold = 50;  // 降低到 50，如果还是检测到太多浅色区域
    threshold(blur, bin, debug_threshold, 255, THRESH_BINARY_INV);

    // 形态学开运算（先腐蚀后膨胀）：去除"入口"等文字笔画的孤立噪点
    // 3x3 核保留细线条，配合下游 spread 检查排除文字
    {
        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(bin, bin, MORPH_OPEN, kernel);
    }
    
    // ROI: bottom narrow strip (closer to robot)
    int roi_height = 40;
    int roi_y_start = blur.rows - roi_height;
    if (roi_y_start < 0) roi_y_start = 0;
    
    // Draw ROI rectangle on image for debugging
    rectangle(debug_img, {0, roi_y_start}, {blur.cols, blur.rows}, {255, 0, 0}, 1);

    err = 0;
    cnt = 0;
    const int roi_width = bin.cols;
    vector<int> col_count(roi_width, 0);  // 每列白色像素计数

    // Only scan the ROI region
    for (int r = roi_y_start; r < blur.rows; ++r)
    {
        const uchar *row = bin.ptr(r);
        for (int c = 0; c < roi_width; ++c)
        {
            if (row[c] > 0)  // White pixel detected
            {
                col_count[c]++;
                cnt++;
            }
        }
    }
    
    // Debug info
    int roi_total_pixels = (blur.rows - roi_y_start) * roi_width;
    double roi_percentage = (double)cnt / roi_total_pixels * 100;

    // —————— 水平投影峰值法找线条中心 ——————
    // 用每列白色像素计数代替平均值，避免"入口"文字拉偏质心
    int peak_col = -1;
    int peak_count = 0;
    for (int c = 0; c < roi_width; ++c) {
        if (col_count[c] > peak_count) {
            peak_count = col_count[c];
            peak_col = c;
        }
    }

    // 只有当峰值列至少有 5 个像素（线条是连续的，文字笔画分散不会形成单列高峰）
    if (peak_count >= 5) {
        err = peak_col - 640;  // 偏差 = 峰值列 - 画面中心

        int center_x = peak_col;
        // Clamp to image bounds
        center_x = max(0, min(1279, center_x));
        
        line(debug_img, {center_x, blur.rows - roi_height/2}, 
             {center_x, blur.rows}, {0, 255, 0}, 2);
        circle(debug_img, {center_x, blur.rows - roi_height/2}, 8, {0, 255, 0}, 2);
    } else {
        err = 0;  // 无有效峰值，fallback
    }

    // 检查是否为有效线条
    // 总像素 cnt 不能太少（无线条）也不能太多（全白/噪音）
    // 峰值 peak_count 至少 5 列以上（线条是连续的垂直带）
    if (cnt >= 50 && cnt <= 50000 && peak_count >= 5)
    {
        putText(debug_img, format("Line: err=%.1f cnt=%d peak=%d thresh=%d %.1f%%", 
                 err, cnt, peak_count, debug_threshold, roi_percentage),
                {10, 30}, FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
        
        // Print success info to console
        cout << "[LINE DETAIL] SUCCESS: err=" << err << " cnt=" << cnt << " peak=" << peak_count
             << " percentage=" << roi_percentage << "%" << endl;
        
        return true;
    }

    // Debug: show why line not detected
    string reason;
    int status_color = 0;  // 0=red for error, 1=yellow for warning
    
    if (cnt < 50) {
        reason = format("NO LINE: cnt=%d (<50)", cnt);
        status_color = 0;
    } else if (cnt > 50000) {
        reason = format("NO LINE: cnt=%d (>50000)", cnt);
        status_color = 0;
    } else if (peak_count < 5) {
        reason = format("NO LINE: peak=%d (<5) text noise?", peak_count);
        status_color = 1;  // Yellow: 可能是文字干扰
    } else {
        reason = format("NO LINE: bad ROI or lighting");
        status_color = 1;
    }
    
    putText(debug_img, reason, {10, 60}, FONT_HERSHEY_SIMPLEX, 0.6, 
            status_color == 0 ? Scalar(0, 0, 255) : Scalar(0, 255, 255), 2);

    // Print detailed debug info to console periodically
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

/* ------------------------------------------------------------------ */
/* --------------------------  Main program  ------------------------ */
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        cerr << "Usage: " << argv[0] << " <ethernet_if>\n";
        return -1;
    }

    /* Init Unitree DDS */
    ChannelFactory::Instance()->Init(0, argv[1]);


    /* Subscribers */
    ChannelSubscriber<geometry_msgs::msg::dds_::PointStamped_> sub_range(TOPIC_RANGE_INFO);
    sub_range.InitChannel(rangeCB);

    StateCB stateCB;
    ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> sub_state(TOPIC_HIGHSTATE);
    sub_state.InitChannel(stateCB);

    /* Sport client */
    go2::SportClient sc;
    sc.SetTimeout(10.0f);
    sc.Init();
    sc.BalanceStand();

    /* Obstacles avoid client - for Move/MoveToIncrementPosition */
    go2::ObstaclesAvoidClient avoid_client;
    avoid_client.Init();
    avoid_client.UseRemoteCommandFromApi(true);  // Enable remote command mode
    avoid_client.SwitchSet(false);  // 强制关闭内置避障，由程序自主控制

    // 注册 SIGINT 信号处理器（仅设置标志，不阻塞）
    signal(SIGINT, signalHandler);

    /* Save initial pose */
    px0 = px;
    py0 = py;
    yaw0 = yaw;

    /* Front-RGB stream */
    VideoCapture cap(
        "udpsrc address=230.1.1.1 port=1720 multicast-iface=ens37 "
        "! application/x-rtp, media=video, encoding-name=H264 "
        "! rtph264depay ! avdec_h264 ! videoconvert "
        "! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1",
        CAP_GSTREAMER);
    if (!cap.isOpened())
    {
        cerr << "Front camera stream not opened\n";
        return -1;
    }

    Mat frame, undist;
    int fcount = 0;
    auto t0 = chrono::steady_clock::now();

    // Start the aruco socket server in a background thread
    std::thread aruco_thread(aruco_socket_server, 5005);
    aruco_thread.detach();

    /* Distance-based obstacle trigger (using position instead of lidar) */
    double obstacle_trigger_px = 0.8;  // Advance 0.8 meters before entering obstacle avoidance
    bool passed_obstacle_trigger = false;

    /* ---------------------------  LOOP  --------------------------- */
    while (!g_exit_requested)
    {
        if (!cap.read(frame) || frame.empty())
            break;
        fcount++;
        undistort(frame, undist, K, D);

        // Status output every 30 frames
        if (fcount % 30 == 0)
        {
            double lx, ly, dyaw;
            transformLocal(px, py, yaw, lx, ly, dyaw);
            cout << "[Status] Flag=" << Flag_Task << " lx=" << lx << " ly=" << ly
                 << " yaw=" << dyaw << " px=" << px << " py=" << py << endl;
        }

        /* FPS overlay */
        double fps = fcount / chrono::duration<double>(chrono::steady_clock::now() - t0).count();
        putText(undist, format("FPS %.1f", fps), {10, 30},
                FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);

        /*****************   MAIN FSM  *****************/
        double lx, ly, dyaw;
        transformLocal(px, py, yaw, lx, ly, dyaw);
        double yaw_pid = dyaw * -5.0; // crude P controller (rad->cmd)

        //Flag_Task = 3
        switch (Flag_Task)
        {
        case 0: /* Line following - advance until obstacle detected visually */
        {

            // --- 跳跃前先向前直行 0.2m ---
            // ★ 用 sc.Move 而非 avoid_client.Move，避免命令冲突导致后面转不了弯
            static int init_stage = 0;  // 0=先直走, 1=再跳跃, 2=恢复站立, 3=开始巡线
            if (init_stage == 0) {
                sc.StaticWalk();
                sc.Euler(0, 0, 0);   // 平视向前走
                sc.Move(0.15, 0, 0);
                if (lx >= 0.2) {
                    sc.StopMove();
                    sc.Move(0, 0, 0);  // 彻底清除 sc 的运动缓存
                    init_stage = 1;
                    cout << "[Start] ✅ Pre-move 0.2m done (lx=" << lx << ")" << endl;
                }
                break;  // 跳过巡线逻辑，下一帧继续前进
            }
            if (init_stage == 1) {
                sc.FrontJump();
                init_stage = 2;
                this_thread::sleep_for(chrono::milliseconds(300));

                // 跳跃后先保存起点，但还不开始巡线
                px0 = px;
                py0 = py;
                yaw0 = yaw;
                cout << "[Start] Jump done, reset origin. Now stabilizing..." << endl;
                break;
            }
            if (init_stage == 2) {
                // 跳跃后做 BalanceStand 恢复稳定，避免僵直倒地
                sc.BalanceStand();
                this_thread::sleep_for(chrono::milliseconds(500));
                init_stage = 3;
                cout << "[Start] ✅ Stabilized after jump. Starting line follow." << endl;
                break;
            }
            // init_stage == 3: 正常巡线（继续执行下面的巡线代码）


            // Line detection
            double line_err = 0;
            int line_cnt = 0;
            Mat line_debug;
            bool line_found = detectLine(undist, line_err, line_cnt, line_debug);

            // —————— 线条连续性检查 ——————
            // 如果线条中心相比上一帧跳变超过 130 像素，且 lx >= 0.6（已走够距离），
            // 说明巡线到终点了（"入口"文字出现），立即结束巡线进入避障
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
                    Flag_Task = 1;
                    cout << "\033[32m[Transition] Line ended (JUMP), entering obstacle avoidance\033[0m" << endl;
                }
            }
            if (line_found) {
                prev_line_err = line_err;
                had_line_before = true;
            }

            // 每帧输出 lx 值，方便调试
            if (fcount % 10 == 0)
            {
                cout << "[Line] lx=" << lx << " ly=" << ly << " (trigger at " << obstacle_trigger_px << "m)" << endl;
            }

            // —————— [新增] ly 漂移主动回正 ——————
            // 当 ly > 0.35 或 ly < -0.35 时，说明机器狗已经横向偏离了轨迹
            // 叠加一个侧向纠偏转向，优先级高于巡线PID
            double ly_correction = 0;
            if (ly > 0.35) {
                ly_correction = -0.3;  // 偏右太多 → 左转回正
                cout << "[LY] ly=" << ly << " > 0.35 → LEFT correction" << endl;
            } else if (ly < -0.35) {
                ly_correction = 0.3;   // 偏左太多 → 右转回正
                cout << "[LY] ly=" << ly << " < -0.35 → RIGHT correction" << endl;
            }

            if (line_found)
            {
                // Validate: check if error is within reasonable range
                if (abs(line_err) < 400 && line_cnt > 100 && line_cnt < 10000)
                {
                    cout << "[Line] lx=" << lx << " ly=" << ly << " err=" << line_err 
                         << " cnt=" << line_cnt << " (OK)" << endl;
                    
                    // PID 巡线控制
                    static double integral = 0, last_err = 0;
                    double Kp = 0.12;   // 大幅提高比例增益：err=-200 → steer≈+0.5（满幅左转）
                    double Ki = 0.002;  // 提高积分增益，消除稳态误差
                    double Kd = 0.01;   // 微分增益，抑制过冲
                    
                    integral += line_err;
                    // 限制积分防止饱和
                    integral = std::max(-50.0, std::min(50.0, integral));
    
                    double derivative = line_err - last_err;
                    last_err = line_err;
    
                    // PID 输出
                    // err 负值（线偏左）→ steer 负值 → 左转 → 纠偏回正
                    // 乘以 -1: err=-228 (偏左) → -(-228 * 0.12) = +27 → 限幅 +0.5 → 左转
                    double steer = -(Kp * line_err + Ki * integral + Kd * derivative);
                    steer = std::max(-0.5, std::min(0.5, steer));
                    
                    // ly 纠偏覆盖：如果 ly 偏差过大，以 ly 纠偏为主
                    if (abs(ly_correction) > 0.01) {
                        steer = ly_correction;
                        cout << "[Line] ly OVERRIDE steer=" << steer << endl;
                    }
    
                    // 叠加航向保持：始终锁定向 yaw0 方向，dyaw 偏差越大纠偏越强
                    steer += -dyaw * 1.5;
                    steer = std::max(-0.5, std::min(0.5, steer));

                    cout << "[Line] steer=" << steer << " (yaw_keep: dyaw=" << dyaw*180/M_PI << "deg)" << endl;
    
                    sc.StaticWalk();
                    // 低头姿态：pitch = 0.4（加大前倾角度，让摄像头更容易看到地面线条）
                    sc.Euler(0, 0.4, 0);
                    sc.Move(0.25, 0, steer);
    
                    // Update display image - use local variable for debug
                    cv::Mat display_img = line_debug.clone();
                    putText(display_img, format("FPS %.1f", fps), {10, 30},
                            FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);
                    undist = display_img;
                }
                else
                {
                    // Error out of range → 直行基础上按偏差方向软纠偏
                    double soft_steer = 0;
                    if (abs(line_err) < 640) {  // 640 = 半幅画面宽，仍在有效范围内
                        soft_steer = -line_err * 0.003;  // 软纠偏 Kp=0.003: err=-200 → steer=+0.6 → 左转
                        soft_steer = std::max(-0.3, std::min(0.3, soft_steer));
                    }
                    // ly 纠偏覆盖
                    if (abs(ly_correction) > 0.01) {
                        soft_steer = ly_correction;
                        cout << "[Line] INVALID ly OVERRIDE steer=" << soft_steer << endl;
                    }
                    // 航向保持：锁定向 yaw0 方向
                    soft_steer += -dyaw * 1.5;
                    soft_steer = std::max(-0.5, std::min(0.5, soft_steer));
                    cout << "[Line] INVALID: err=" << line_err << " cnt=" << line_cnt 
                         << " -> going straight with soft steer=" << soft_steer << " (yaw_keep: dyaw=" << dyaw*180/M_PI << "deg)" << endl;
                    sc.StaticWalk();
                    // 低头姿态：pitch = 0.4（加大前倾角度）
                    sc.Euler(0, 0.4, 0);
                    sc.Move(0.2, 0, soft_steer);  // 直行 + 软纠偏

                    cv::Mat display_img = line_debug.clone();
                    putText(display_img, format("FPS %.1f", fps), {10, 30},
                            FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);
                    undist = display_img;
                }
            }
            else
            {
                cout << "[Line] NO LINE - going straight";
                // ly 纠偏覆盖
                if (abs(ly_correction) > 0.01) {
                    cout << " with ly steer=" << ly_correction;
                }
                // 航向保持：锁定向 yaw0 方向
                double noline_steer = ly_correction + (-dyaw * 1.5);
                noline_steer = std::max(-0.3, std::min(0.3, noline_steer));
                cout << " (yaw_keep: dyaw=" << dyaw*180/M_PI << "deg steer=" << noline_steer << ")" << endl;
                sc.StaticWalk();
                // 低头姿态：pitch = 0.4（加大前倾角度）
                sc.Euler(0, 0.4, 0);
                sc.Move(0.15, 0, noline_steer);
            }
            
            // Update undist for display in NO LINE case
            cv::Mat display_img = line_debug.clone();
            putText(display_img, format("FPS %.1f", fps), {10, 30},
                    FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);
            undist = display_img;

            // 使用雷达检测障碍物，而不是固定位置触发
            // 当 lx > 0.75m 且雷达检测到前方障碍 < 1.5m 时进入避障
            if (lx > 0.75 && g_lidar_front_dist < 1.5)
            {
                cout << "\033[32m[Transition] Obstacle detected by lidar (dist=" << g_lidar_front_dist << "m), entering obstacle avoidance\033[0m" << endl;
                sc.StopMove();
                Flag_Task = 1;
            }
        }
        break;

        case 1: /* Obstacle avoidance - S-shaped corridor navigation */
        {
            // ✅ 避障阶段平视（pitch=0），让激光雷达正对前方墙壁
            sc.Euler(0, 0, 0);
            sc.StaticWalk();

            static int phase = 0;
            static double phase_start_lx = lx;
            static double yaw_turn_start = yaw;

            // —————— 雷达三通道：front=x, left=y, right=z ——————
            // 使用 EMA 滤波值做控制，平滑抗噪
            double front_dist = g_lidar_front_f;
            double left_dist  = g_lidar_left_f;
            double right_dist = g_lidar_right_f;

            // —————— 墙壁检测：用原始值（反应灵敏，不受 EMA 延迟影响） ——————
            // 原始值在 0.01~0.6 之间 → 墙在附近，提前触发转弯（避免撞墙）
            // 原始值 < 0.01 → 传感器已超出量程（贴脸了），也视为撞墙（需滤波确认）
            bool front_wall_raw = (g_lidar_front_dist > 0.01 && g_lidar_front_dist <= 0.6);
            bool front_too_close_raw = (g_lidar_front_dist <= 0.01 && front_dist < 0.6);  // 贴脸+滤波确认
            bool wall_detected = front_wall_raw || front_too_close_raw;
            // —————— 直行阶段触发条件 ——————
            // 所有直行阶段直接用 wall_detected（前墙≤0.6m）
            bool wall_detected_straight = wall_detected;

            // —————— 三向安全保护（参考代码 applyRangeClearance） ——————
            // 侧向 < 0.2m 或无效时推反方向 vy
            // 前向 < 0.4m 或无效时减速后退
            float vx_safe = 0.15f, vy_safe = 0.f;
            
            // 左侧太近 → vy 减小（向右侧平移）
            if (left_dist > 999.0 || left_dist < 0.2) {
                float deficit = (left_dist > 999.0) ? 0.2f : (0.2f - left_dist);
                vy_safe -= std::min(0.38f, deficit * 3.0f);
            }
            // 右侧太近 → vy 增大（向左侧平移）
            if (right_dist > 999.0 || right_dist < 0.2) {
                float deficit = (right_dist > 999.0) ? 0.2f : (0.2f - right_dist);
                vy_safe += std::min(0.38f, deficit * 3.0f);
            }
            // 前方太近 → 减速/后退
            if (front_dist > 999.0 || front_dist < 0.4) {
                float deficit = (front_dist > 999.0) ? 0.4f : (0.4f - front_dist);
                vx_safe -= std::min(0.28f, deficit * 2.0f);
                vx_safe = std::max(-0.28f, vx_safe);
            }
            
            // 安全状态日志
            bool safety_active = (abs(vy_safe) > 0.02 || vx_safe < 0.12);
            if (safety_active) {
                cout << "[OB] 🛡️ SAFETY: vx=" << vx_safe << " vy=" << vy_safe
                     << " (front=" << front_dist << " left=" << left_dist << " right=" << right_dist << ")" << endl;
            }

            // —————— 走廊居中（参考代码：vy = (left-right) * gain） ——————
            bool is_straight = (phase == 0 || phase == 2 || phase == 4 || phase == 6 || phase == 8 || phase == 10);
            float vy_center = 0.f;
            if (is_straight) {
                float side_sum = left_dist + right_dist;
                if (side_sum > 0.05f) {
                    vy_center = std::max(-0.12f, std::min((float)((left_dist - right_dist) * 0.42f), 0.12f));
                    if (abs(vy_center) > 0.01f)
                        cout << "[OB] 🎯 Centering: L=" << left_dist << " R=" << right_dist << " vy=" << vy_center << endl;
                }
            }

            // —————— 最终 vx/vy ——————
            // 安全保护优先，centering 叠加（同号相加）
            float vx_final = vx_safe;
            float vy_final = vy_safe + vy_center;
            vy_final = std::max(-0.38f, std::min(0.38f, vy_final));

            // 🔍 打印调试信息（每10帧一次）
            if (fcount % 10 == 0) 
            {
                cout << "[OB] S-CORRIDOR phase=" << phase
                     << " F=" << front_dist << " L=" << left_dist << " R=" << right_dist
                     << " lx=" << lx << " vy=" << vy_final
                     << " yaw=" << yaw*180/M_PI << "deg" << endl;
            }

            // ----- Phase 0: 走到墙，同时保持直线 -----
            if (phase == 0) {
                // Phase 0 走直线纠偏：ly 漂移回正 + 航向保持
                // ly 横向偏差 → 转向纠偏（ly>0 偏右 → 左转 steer 负值）
                double steer_phase0 = -ly * 0.3;
                // 航向保持：锁定跳跃完成时的 yaw0 方向
                double yaw_drift = yaw - yaw0;
                if (yaw_drift > M_PI) yaw_drift -= 2*M_PI;
                if (yaw_drift < -M_PI) yaw_drift += 2*M_PI;
                steer_phase0 += -yaw_drift * 2.0;
                steer_phase0 = std::max(-0.3, std::min(0.3, steer_phase0));

                // 仅在偏差明显时打印
                if (abs(steer_phase0) > 0.02) {
                    cout << "[OB] Phase0 STRAIGHT: ly=" << ly << " yaw_drift="
                         << yaw_drift*180/M_PI << "deg steer=" << steer_phase0 << endl;
                }

                sc.Move(vx_final, vy_final, steer_phase0);

                // 紧急停止保护
                if (front_dist < 0.2) {
                    sc.Move(0, 0, 0);
                    cout << "[OB] ⚠️ EMERGENCY STOP: front_dist=" << front_dist << "m!" << endl;
                }

                // Phase 0: lx >= 1.4 后才允许触发（避免刚进入避障就立刻转）
                if (wall_detected && lx >= 1.4) {
                    // ❌ 不用 StopMove，避免踉跄撞墙，直接切 phase
                    phase = 1;
                    yaw_turn_start = yaw;
                    phase_start_lx = lx;
                    cout << "[OB] ✅ Phase 0 DONE: Wall at " << front_dist
                         << "m (lx=" << lx << ") → START LEFT TURN 90°" << endl;
                }
            }
            // ----- Phase 1/3/9: 向左弧线转 90° -----
            else if (phase == 1 || phase == 3 || phase == 9) {
                double yaw_diff = yaw - yaw_turn_start;
                if (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
                if (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;

                if (yaw_diff >= M_PI / 2 * 0.9) {
                    phase++;
                    phase_start_lx = lx;
                    cout << "[OB] ✅ Phase " << phase-1 << " DONE: Left turn 90° (yaw diff=" << yaw_diff*180/M_PI << "deg)" << endl;
                } else {
                    // 转弯时最慢前进 vx=0.15，加快旋转速度 yaw_rate=0.8
                    float vx_turn = std::min(0.15f, vx_final);
                    sc.Move(vx_turn, vy_final, 0.8f);  // 左转弧线（旋转更快）
                }
            }
            // ----- Phase 5/7: 向右弧线转 90° -----
            else if (phase == 5 || phase == 7) {
                double yaw_diff = yaw - yaw_turn_start;
                if (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
                if (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;

                if (yaw_diff <= -M_PI / 2 * 0.9) {
                    phase++;
                    phase_start_lx = lx;
                    cout << "[OB] ✅ Phase " << phase-1 << " DONE: Right turn 90° (yaw diff=" << yaw_diff*180/M_PI << "deg)" << endl;
                } else {
                    // 右转同样最慢前进 vx=0.15，加快旋转速度 yaw_rate=-0.8
                    float vx_turn = std::min(0.15f, vx_final);
                    sc.Move(vx_turn, vy_final, -0.8f);  // 右转弧线（旋转更快）
                }
            }
            // ----- Phase 2/4/6/8/10: 直行 -----
            else if (phase == 2 || phase == 4 || phase == 6 || phase == 8 || phase == 10) {
                // ❌ 不再加延迟，检测到墙立刻触发（避免撞墙）
                if (wall_detected_straight) {
                    if (phase == 10) {
                        // Phase 10 → 完成 S 型序列
                        phase = 0;
                        Flag_Task = 0;
                        cout << "[OB] 🎉 S-SHAPED CORRIDOR NAVIGATION COMPLETE! Resuming line follow." << endl;
                    } else {
                        phase++;
                        yaw_turn_start = yaw;
                        phase_start_lx = lx;
                        cout << "[OB] ✅ Phase " << phase-1 << " DONE: Forward until wall → Phase " << phase << endl;
                    }
                } else {
                    sc.Move(vx_final, vy_final, 0.f);
                }
            }
        }
        break;

        case 2: /* Aruco detection - turn left 90 degrees */
        {
            static bool turned = false;
            if (!turned)
            {
                sc.StopMove();
                // Turn left until yaw changes by ~90 degrees
                double yaw_at_turn_start = yaw;
                while (true)
                {
                    double yaw_diff = yaw - yaw_at_turn_start;
                    if (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
                    if (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;

                    if (yaw_diff < -M_PI / 2 * 0.9)  // 90% of 90 degrees
                        break;
                    sc.Move(0, 0, 0.15);
                }
                sc.StopMove();
                turned = true;
                Flag_Task = 3;
            }
            else
            {
                // Check for aruco marker
                if (g_last_aruco_id == 0)
                {
                    cout << "[Aruco] Marker 0 detected!" << endl;
                    Flag_Task = 3;
                }
                else
                {
                    sc.Move(0.15, 0, 0);
                }
            }
        }
        break;

        case 3: /* Move forward to next aruco */
        {
            sc.StaticWalk();
            sc.Euler(0, 0, 0);
            sc.Move(0.2, 0, 0);

            if (g_last_aruco_id > 0)  // Next marker detected
            {
                cout << "[Aruco] Next marker detected: " << g_last_aruco_id.load() << endl;
                Flag_Task = 4;
            }
        }
        break;

        case 4: /* Right turn and continue */
        {
            sc.StopMove();
            double yaw_at_start = yaw;
            while (true)
            {
                double yaw_diff = yaw - yaw_at_start;
                if (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
                if (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;

                if (yaw_diff > M_PI / 2 * 0.9)  // Right turn 90 degrees
                    break;
                sc.Move(0, 0, -0.15);
            }
            sc.StopMove();
            Flag_Task = 5;
        }
        break;

        case 5: /* Stairs area - use FreeWalk mode */
        {
            sc.FreeWalk();

            // Simple stair traversal: move forward
            sc.Move(0.15, 0, 0);

            // After some distance, move to next phase
            double lx_stairs, ly_stairs, dyaw_stairs;
            transformLocal(px, py, yaw, lx_stairs, ly_stairs, dyaw_stairs);
            if (lx_stairs > obstacle_trigger_px + 2.0)
            {
                Flag_Task = 6;
            }
        }
        break;

        case 6: /* Back to normal gait */
        {
            sc.StaticWalk();
            sc.Move(0.2, 0, 0);

            if (g_last_aruco_id > 0)
            {
                Flag_Task = 7;
            }
        }
        break;

        case 7: /* Turn left towards finish */
        {
            sc.StopMove();
            double yaw_at_start = yaw;
            while (true)
            {
                double yaw_diff = yaw - yaw_at_start;
                if (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
                if (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;

                if (yaw_diff < -M_PI / 2 * 0.9)
                    break;
                sc.Move(0, 0, 0.15);
            }
            sc.StopMove();
            Flag_Task = 8;
        }
        break;

        case 8: /* Jump into finish area */
        {
            if (end_jump_times == 0)
            {
                sc.FrontJump();
                end_jump_times++;
            }
            sc.Move(0.2, 0, 0);
            Flag_Task = 9;
        }
        break;

        case 9: /* Finish */
            sc.StopMove();
            // Restore remote control fully
            avoid_client.UseRemoteCommandFromApi(false);
            avoid_client.SwitchSet(false);
            avoid_client.Move(0, 0, 0);
            this_thread::sleep_for(chrono::milliseconds(200));
            sc.SwitchJoystick(true);  // Re-enable joystick control
            sc.RecoveryStand();       // Recover to standing, release API control
            this_thread::sleep_for(chrono::milliseconds(500));
            sc.BalanceStand();
            cout << "\033[32mMission complete! Remote control restored.\033[0m" << endl;
            return 0;
        }

        imshow("Go2 Front Cam - Visual Nav", undist);
        int key = waitKey(1);
        if (key == 27 || g_exit_requested)
            break; // ESC or Ctrl+C to quit
    }
    cout << "[Exit] Cleaning up and restoring remote control..." << endl;
    sc.StopMove();
    // Restore remote control before exit
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
