#include "case4.h"

#include <iostream>

using namespace unitree::robot;

bool case4_tick(go2::SportClient &sc)
{
    sc.StopMove();
    std::cout << "Mission complete\n";
    return true;
}