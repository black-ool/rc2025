#include "case3.h"
#include "../globals.h"

#include <thread>
#include <chrono>

using namespace unitree::robot;

void case3_tick(go2::SportClient &sc)
{
    if (end_jump_times == 0)
    {
        sc.FrontJump();
        end_jump_times++;
        std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    }
    Flag_Task = 4;
}