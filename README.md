# RC2025 — 睿抗机器人开发者大赛 · 多模态巡检

本项目为 **2025 睿抗机器人开发者大赛（RC2025）** 参赛代码，基于宇树（Unitree）Go2 四足机器人与 D1 七轴机械臂，实现**多模态自主巡检与抓取**任务。

> 远程仓库: [https://github.com/Prcjimmy1015/rc2025](https://github.com/Prcjimmy1015/rc2025)

---

## 目录

- [项目结构](#项目结构)
- [模块一：Go2 机器狗运动控制（go2\_runner）](#模块一go2-机器狗运动控制go2_runner)
- [模块二：D1 机械臂视觉抓取（d1\_arm）](#模块二d1-机械臂视觉抓取d1_arm)
- [模块三：ArUco 感知模块（perception）](#模块三aruco-感知模块perception)
- [环境依赖](#环境依赖)
- [构建与运行](#构建与运行)
- [文档资料（docs）](#文档资料docs)

---

## 项目结构

```
rc2025_demo/
├── .gitignore                     # Git 忽略规则（构建产物、IDE 配置等）
├── README.md                      # 本文件
│
├── docs/                          # 竞赛文档与技术资料
│   ├── 1-2026睿抗机器人开发者大赛-多模态巡检.pdf
│   └── 02_D435深度相机取流及深度信息获取接口说明.pdf
│
├── go2_runner/                    # 【模块一】Go2 机器狗导航与控制 (C++)
│   ├── CMakeLists.txt             # CMake 构建配置
│   ├── run.sh                     # 一键编译+运行脚本
│   ├── main.cpp                   # 入口：信道初始化 → 主循环 → 状态机调度
│   ├── params.h                   # 相机内参 & 全局可调参数常量（阈值、速度、PID 等）
│   ├── globals.h / globals.cpp    # 全局状态变量声明/定义（雷达数据、位姿、任务标志）
│   ├── app_runtime.h / .cpp       # 运行时对象定义与初始化（DDS 订阅/Sport/相机）
│   ├── callbacks.h / callbacks.cpp# DDS 回调：雷达测距 rangeCB、运动状态 StateCB
│   ├── utils.h / utils.cpp        # 工具函数：坐标变换、PID、安全距离、站直检测、路口判定
│   ├── visualizer.h / .cpp        # 实时雷达距离曲线可视化（OpenCV）
│   ├── aruco_server.h / .cpp      # ArUco TCP 服务端（接收感知模块传来的标记 ID）
│   └── cases/                     # 任务状态机各阶段实现
│       ├── case0.h / case0.cpp    # 阶段0：前进→起跳→巡线→检测窄过道入口
│       ├── case1.h / case1.cpp    # 阶段1：迷宫——两次180°掉头+一次左转90°弧线
│       ├── case2.h / case2.cpp    # 阶段2：巡线准备过台阶（占位停车）
│       ├── case3.h / case3.cpp    # 阶段3：跳跃进入终点区域
│       └── case4.h / case4.cpp    # 阶段4：任务完成，停止并输出完成信息
│
├── d1_arm/                        # 【模块二】D1 机械臂视觉抓取 (Python)
│   ├── description/               # D1 机械臂模型描述
│   │   ├── d1_description.urdf    # URDF 机器人描述文件（7 轴运动链）
│   │   └── d1_description.csv     # 坐标系/关节辅助数据
│   ├── models/
│   │   └── best.onnx              # YOLOv8 ONNX 权重（water / assam / orange 三类）
│   └── scripts/                   # 控制与感知脚本
│       ├── d1_arm.py              # D1 机械臂 Python 封装（UnitreeD1Arm + D1RobotArmController）
│       ├── d1_pick.py             # 视觉抓取主流程（相机标定→YOLO检测→坐标转换→抓取）
│       ├── yolov8_onnx.py         # YOLOv8 ONNX 推理器（预处理/后处理/绘图）
│       ├── camera_d435.py         # Intel RealSense D435 深度相机封装（对齐/取流/测距）
│       └── check_urdf.py          # URDF 运动链解析与验证
│
├── perception/                    # 【模块三】ArUco 标记检测 (Python)
│   ├── aruco_detector.py          # 基于 Go2 前视压缩图的 ArUco 标记检测 + TCP 回传
│   └── d435_camera/
│       └── camera_d435.py         # D435 相机封装（与 d1_arm 中功能相同，独立副本）
│
└── go2_modular/                   # （预留）Go2 模块化组件
```

---

## 模块一：Go2 机器狗运动控制（go2_runner）

基于 **宇树 Unitree SDK2** 的 C++ 应用，控制 Go2 四足机器人在巡检场地中自主完成前进、跳跃、迷宫导航等任务。

### 文件功能说明

| 文件 | 功能 |
|------|------|
| `main.cpp` | 程序入口。解析命令行参数（网卡接口 `eth_if`、`--gui` 开关），初始化 DDS 通道与 Sport 客户端，启动 ArUco TCP 服务端线程，进入主循环（`runMainLoop`）。主循环每帧：采集前视图像 → 去畸变 → 日志输出 → 坐标转换 → 状态机调度 → GUI 渲染。 |
| `CMakeLists.txt` | CMake 构建配置。自动检测 CPU 架构（x86_64 / aarch64），定位 Unitree SDK2 路径，链接 OpenCV、Cyclone DDS、pthread 等依赖，构建 `rc2025_run` 可执行文件。 |
| `run.sh` | 一键构建+运行脚本。进入 build 目录执行 cmake/make，然后启动 `rc2025_run`。 |
| `params.h` | **全局参数中心**。定义相机内参矩阵 K 与畸变系数 D（需按实机标定替换），以及所有可调常量：雷达 EMA 系数、各 case 阶段的速度/距离阈值、PID 参数、路口判定区间等。 |
| `globals.h` / `globals.cpp` | 全局变量声明与定义。包括：雷达原始/滤波测距 (`ob_x/_y/_z`)、机体位姿 (`px/py/yaw`)、任务状态机标志 (`Flag_Task`)、迷宫子状态变量、ArUco ID 原子变量、GUI 开关。 |
| `app_runtime.h` / `app_runtime.cpp` | `AppRuntime` 结构体：聚合 DDS 订阅器（雷达 `sub_range`、运动状态 `sub_state`）、Sport 客户端、VideoCapture 相机。`initAppRuntime()` 函数按正确顺序初始化所有组件（Sport → 订阅 → GStreamer 视频流）。 |
| `callbacks.h` / `callbacks.cpp` | DDS 回调函数。`rangeCB()` 处理雷达测距（提取 x/y/z + EMA 滤波）；`StateCB::operator()` 处理四足状态（位姿、IMU、足端力），同时更新全局 `px/py/yaw`。 |
| `utils.h` / `utils.cpp` | **工具函数库**。关键功能：<br>• `transformLocal()` — 世界→局部坐标系变换<br>• `PID_Yaw/PID_Yaw1` — 两种 PID 航向控制器<br>• `applyRangeClearance()` — 基于雷达安全裕量的速度修正（前/左/右三向）<br>• `isGo2UprightAfterJump()` — 起跳后站直判定（姿态+速度+足力）<br>• `isInNarrowCorridor()` / `isMazeJunctionCandidate()` — 窄过道/路口判定<br>• `yawTurnRateCmd()` — 转弯角速度指令生成<br>• `resetMazeFsm()` — 迷宫子状态机重置 |
| `visualizer.h` / `visualizer.cpp` | 实时雷达距离曲线可视化（OpenCV）。`makeRangePlotMat()` 绘制 ob_x（前）/ ob_y（左）/ ob_z（右）的时序曲线，支持 `--gui` 开关。 |
| `aruco_server.h` / `aruco_server.cpp` | TCP 服务端（监听 `127.0.0.1:5005`）。接收 `perception/aruco_detector.py` 发来的 ArUco 标记 ID，写入全局原子变量 `g_last_aruco_id`。被 detach 到独立线程运行。 |

### 状态机（cases/）

状态机通过 `Flag_Task`（0~4）切换任务阶段：

| Case | 文件 | 行为描述 |
|------|------|----------|
| **Case 0** | `case0.h/cpp` | ① 前进 0.1m → ② 执行 `FrontJump`（前跳）→ ③ 检测落地站直（IMU 姿态+足端力）→ ④ 稳定等待 12 帧 → ⑤ 基于图像二值化巡线前进 → ⑥ 雷达检测窄过道入口（左右侧距在 0.18~0.48m 区间）→ 转入 Case 1 |
| **Case 1** | `case1.h/cpp` | 迷宫导航：① 沿窄过道壁面居中巡航 → ② 第 1 次路口：左转 180° 协调弧线 → ③ 继续巡航 → ④ 第 2 次路口：右转 180° 弧线 → ⑤ 第 3 次路口：左转 90° 弧线 → 转入 Case 2 |
| **Case 2** | `case2.h/cpp` | 巡线前进（图像二值化+雷达安全），为过台阶区域做准备。当前为占位停车逻辑。 |
| **Case 3** | `case3.h/cpp` | 执行终点跳跃 `FrontJump` + 等待 2.5s → 立即转入 Case 4 |
| **Case 4** | `case4.h/cpp` | 停止运动，输出 `"Mission complete"`，通知主循环退出。 |

### Go2 架构数据流

```
DDS 网络 ──────┐
               ├── rangeCB ──→ ob_x/y/z ──(EMA)──→ ob_x_f/y_f/z_f
               └── StateCB ──→ px/py/yaw
                                    │
前端摄像头 ──(GStreamer)──→ VideoCapture → 去畸变 → 巡线（二值化阈值）
                                    │
TCP 5005 ──── aruco_socket_server ──→ g_last_aruco_id (atomic)
                                    │
                                    ▼
                        runMainLoop() 主循环
                            │
                    全局变量 + FSM 调度
                            │
                    SportClient (Move/Jump/Stop)
                            │
                            ▼
                       Go2 机身执行
```

---

## 模块二：D1 机械臂视觉抓取（d1_arm）

基于 Python 的机械臂控制与 YOLOv8 视觉抓取系统，使用 Intel RealSense D435 深度相机和 ONNX 推理。

### 文件功能说明

| 文件 | 功能 |
|------|------|
| `scripts/d1_arm.py` | **D1 机械臂 Python 控制封装**。定义两个类：<br>• `UnitreeD1Arm` — 基础控制接口（enable/disable/home/safe_fold/move_single_joint）<br>• `D1RobotArmController` — 高级接口，兼容原有代码调用风格（blinx_movej/blinx_movel 关节/笛卡尔运动、导航/拍照/预抓取/抓取/放置姿态）<br>底层通过 `subprocess` 调用 C++ 编译的 `d1_*` 可执行文件实现。 |
| `scripts/d1_pick.py` | **视觉抓取主流程**（`Robot_pick` 类）。核心流程：<br>① 初始化相机 + YOLO 模型 + 机械臂<br>② 相机-机械臂标定（三点仿射变换矩阵）<br>③ 拍照 + YOLO 目标检测<br>④ 坐标转换（像素→世界坐标）<br>⑤ 多段轨迹规划：导航姿态 → 拍照姿态 → 预抓取姿态 → 笛卡尔逼近 → 抓取闭合 → 抬升 → 放置 |
| `scripts/yolov8_onnx.py` | **YOLOv8 ONNX 推理器**。支持 CUDA/CPU 推理，识别 `water` / `assam` / `orange` 三类目标。包含图像预处理（resize+归一化+通道转置）、NMS 后处理、检测框绘制。每个类别仅保留最高置信度目标。 |
| `scripts/camera_d435.py` | **Intel RealSense D435 相机封装**。支持彩色/深度流对齐取帧、指定像素点深度测距（mm）、彩色深度并排可视化。 |
| `scripts/check_urdf.py` | URDF 运动链解析脚本，使用 `ikpy` 库验证 D1 机械臂的关节结构与活动范围。 |
| `description/d1_description.urdf` | D1 七轴机械臂的 URDF 模型，由 SolidWorks 导出。定义 base_link→Link1~6→7_1/7_2 的完整运动链（6 个旋转关节 + 2 个棱柱关节），含惯性参数、STL 网格与关节限位。 |
| `description/d1_description.csv` | 关节/坐标系的辅助描述数据。 |
| `models/best.onnx` | YOLOv8 训练好的 ONNX 模型文件，用于目标检测。 |

### D1 视觉抓取流程

```
初始化
  ├── Camera (D435)
  ├── YOLOv8 (ONNX)
  ├── D1RobotArmController
  └── 标定矩阵 (三点仿射)
          │
拍照姿态 → 取帧 → YOLO 推理 → 检测到目标？
          │                        │
          ├─ No ─────────────────→ 导航姿态
          │
          └─ Yes → 坐标转换 (像素→世界)
                   → 预抓取姿态
                   → 笛卡尔逼近
                   → 抓手闭合
                   → 抬升
                   → 放置姿态
                   → 抓手张开
                   → 导航姿态
```

---

## 模块三：ArUco 感知模块（perception）

### 文件功能说明

| 文件 | 功能 |
|------|------|
| `aruco_detector.py` | **ArUco 标记检测与通信桥梁**。通过 Unitree SDK2 的 VideoClient 获取 Go2 前视摄像头的 H.264 压缩图像，解码后使用 OpenCV ArUco 检测器识别 DICT_4X4_50 字典中 ID 0~5 的标记，将识别到的 ID 通过 TCP Socket 发送到 `go2_runner` 的 `aruco_server`（端口 5005）。未检测到时发送 `-1`。 |
| `d435_camera/camera_d435.py` | D435 相机封装（与 d1_arm 中功能相同的独立副本）。 |

---

## 环境依赖

### 硬件

- **宇树 Go2 四足机器人**（含前视摄像头、UTLIDAR 雷达）
- **宇树 D1 七轴机械臂**（含灵巧手）
- **Intel RealSense D435 深度相机**
- **NVIDIA Jetson / x86_64 工控机**（运行 ONNX 推理）

### 软件（Go2 Runner）

| 依赖 | 用途 |
|------|------|
| Unitree SDK2 | Go2 DDS 通信、Sport 运动控制 |
| Cyclone DDS (ddsc/ddscxx) | DDS 中间件 |
| OpenCV 4.x | 图像处理（去畸变、巡线、GUI 可视化） |
| CMake ≥ 3.16, GCC ≥ 9 (C++17) | 编译构建 |
| ROS2 (std_msgs) | DDS 消息类型定义 |

### 软件（D1 Arm / Perception）

| 依赖 | 用途 |
|------|------|
| Python ≥ 3.8 | 运行环境 |
| pyrealsense2 | Intel RealSense D435 驱动 |
| onnxruntime (GPU/CPU) | YOLOv8 推理 |
| opencv-python | ArUco 检测、图像处理 |
| numpy | 数值计算 |
| unitree_sdk2py | Go2 视频流获取（ArUco 检测用） |
| ikpy | URDF 运动链解析（check_urdf.py） |

---

## 构建与运行

### Go2 Runner（C++）

```bash
# 进入 go2_runner 目录
cd go2_runner

# 方法一：使用脚本一键构建+运行
./run.sh eth0               # 无 GUI
./run.sh eth0 --gui         # 带 GUI 可视化窗口

# 方法二：手动构建
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# 运行（需先确保 Go2 处于就绪状态）
./rc2025_run eth0           # 无 GUI（headless 模式）
./rc2025_run eth0 --gui     # 带 GUI（需 X11/桌面环境）
```

### D1 Arm（Python）

```bash
# 确保已安装依赖
pip install pyrealsense2 onnxruntime opencv-python numpy ikpy

# 注意：需要先编译 D1 的 C++ 可执行文件
# (d1_enable, d1_disable, d1_home, d1_safe_fold, d1_move_single 等)
# 并放置在 d1_arm/scripts/ 目录下

# 测试 D1 机械臂控制
cd d1_arm/scripts
python d1_arm.py

# 执行视觉抓取
python d1_pick.py
```

### ArUco 感知模块（Python）

```bash
cd perception
python aruco_detector.py eth0   # 指定网卡接口
```

---

## 文档资料（docs）

| 文件 | 内容 |
|------|------|
| `1-2026睿抗机器人开发者大赛-多模态巡检.pdf` | 竞赛任务说明与规则 |
| `02_D435深度相机取流及深度信息获取接口说明.pdf` | RealSense D435 使用指南 |

---

## 任务流程总览

```
START
  │
  ▼
[CASE 0]  前进 0.1m → 前跳 → 站直检测 → 巡线 → 进入迷宫入口
  │
  ▼
[CASE 1]  迷宫导航：两次 180° 掉头 + 一次 90° 左转弧线
  │
  ▼
[CASE 2]  巡线 / 过台阶准备（平台停车占位）
  │
  ▼
[CASE 3]  终点前跳（FrontJump + 2.5s 等待）
  │
  ▼
[CASE 4]  停止 → "Mission complete"
  │
  ▼
FINISH
```

---

## 注意事项

1. **相机内参**（`params.h` 中的 `K`、`D` 矩阵）需按实机标定结果替换。
2. **网卡接口**：`eth_if` 参数为 Go2 与主机通信的有线网卡名（如 `eth0`、`enp3s0`），需根据实际环境指定。
3. **Unitree SDK2 路径**：CMakeLists.txt 优先使用 `-DUNITREE_SDK2_DIR` 或环境变量，默认查找 `$HOME/unitree_sdk2`。
4. **D1 可执行文件**：`d1_arm.py` 依赖编译好的 C++ 控制程序（`d1_enable` 等），需自行从宇树官方 SDK 编译。
5. **模型文件**：`d1_arm/models/best.onnx` 为自训练的 YOLOv8 模型，识别 `water`/`assam`/`orange` 三类，可按需替换。
6. **GUI 模式**：`--gui` 仅在有 X11 桌面环境的机器上可用，机载无头模式（headless）请省略该参数。