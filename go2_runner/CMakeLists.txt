cmake_minimum_required(VERSION 3.16)
project(rc2025 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

# ─────────── OpenCV (required) ───────────
find_package(OpenCV REQUIRED)

# ─────────── ROS2 dependencies ───────────
find_package(std_msgs REQUIRED)

# ─────────── Locate Unitree SDK2 ─────────
# Priority:  (1) -DUNITREE_SDK2_DIR=…  (2) $ENV{UNITREE_SDK2_DIR}  (3) $HOME/unitree_sdk2
if(NOT UNITREE_SDK2_DIR)
    if(DEFINED ENV{UNITREE_SDK2_DIR})
        set(UNITREE_SDK2_DIR $ENV{UNITREE_SDK2_DIR})
    else()
        set(UNITREE_SDK2_DIR "$ENV{HOME}/unitree_sdk2")
    endif()
endif()
message(STATUS "Using UNITREE_SDK2_DIR = ${UNITREE_SDK2_DIR}")

# Headers
include_directories(
    ${UNITREE_SDK2_DIR}/include
    ${UNITREE_SDK2_DIR}/thirdparty/include       # DDS core
    ${UNITREE_SDK2_DIR}/thirdparty/include/ddscxx
)

# ── Auto-detect CPU architecture ──────────────────────────────────────
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(UNITREE_ARCH "aarch64")
else()
    set(UNITREE_ARCH "x86_64")
endif()
message(STATUS "Unitree SDK architecture = ${UNITREE_ARCH}")

# Libraries
link_directories(
    ${UNITREE_SDK2_DIR}/lib/${UNITREE_ARCH}
    ${UNITREE_SDK2_DIR}/thirdparty/lib/${UNITREE_ARCH}
)

# ─────────── Our executable ──────────────
add_executable(rc2025_run
    main.cpp
    globals.cpp
    callbacks.cpp
    utils.cpp
    visualizer.cpp
    aruco_server.cpp
    app_runtime.cpp
    cases/case0.cpp
    cases/case1.cpp
    cases/case2.cpp
    cases/case3.cpp
    cases/case4.cpp
)

# Ensure we load the SAME DDS libraries at run-time that we linked against.
# Use RPATH (not RUNPATH) so it takes precedence over LD_LIBRARY_PATH.
set_target_properties(rc2025_run PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${UNITREE_SDK2_DIR}/thirdparty/lib/${UNITREE_ARCH};${UNITREE_SDK2_DIR}/lib/${UNITREE_ARCH}"
    LINK_FLAGS "-Wl,--disable-new-dtags"
)

# Link OpenCV, Unitree, DDS, pthread, and ROS2
# Use target_link_libraries for ROS2 as well
# (rclcpp_LIBRARIES and std_msgs_LIBRARIES are usually empty, so link by name)
target_link_libraries(rc2025_run
    PRIVATE
        ${OpenCV_LIBS}
        unitree_sdk2        # main SDK static lib
        ddscxx              # Cyclone DDS C++ wrapper
        ddsc                # Cyclone DDS C core
        pthread             # Posix thread
)
