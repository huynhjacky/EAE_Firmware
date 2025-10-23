#include "canbus.hpp"
#include <iostream>

//--------------------------------------------
// See header file canbus.hpp for description
//--------------------------------------------
CanBus::CanBus()
{
}

//--------------------------------------------
// See header file canbus.hpp for description
//--------------------------------------------
void CanBus::send(const CanFrame& frame)
{
    std::lock_guard<std::mutex> lock(busLock);
    bus.push(frame);
}

//--------------------------------------------
// See header file canbus.hpp for description
//--------------------------------------------
void CanBus::receive(CanFrame& frame)
{
    std::lock_guard<std::mutex> lock(busLock);
    if (!bus.empty())
    {
        frame = bus.front();
        bus.pop();
    }
}
