#include "sensornode.hpp"
#include <thread>
#include <cstring>

const uint32_t FANSPEED_FRAME_ID = 0x200;
const uint32_t TEMPERATURE_FRAME_ID = 0x100;

//--------------------------------------------------------------------------------------
// See header file sensornode.hpp for description
//--------------------------------------------------------------------------------------
Sensor::Sensor(CanBus& b)
        : bus(b) 
{
}

//--------------------------------------------------------------------------------------
// See header file sensornode.hppfor description
//--------------------------------------------------------------------------------------
void Sensor::run()
{
    while (true)
    {
        // exit signal detected
        if (exit)
        {
            return;
        }

        // read for Fan Speed CAN frame from BUS
        CanFrame frame;
        bus.receive(frame);
        if (frame.id == FANSPEED_FRAME_ID && frame.dlc > 0)
        {
            double fanSpeed;
            std::memcpy(&fanSpeed, &frame.data, sizeof(double));

            // calculate temperature with Fan Speed
            currentTemperature = calculateTemperature(fanSpeed, currentTemperature);
        }
        else // if no Fan Speed CAN Frame was read assume fan speed is 0
        {
            // calculate temperature with Fan Speed of 0
            currentTemperature = calculateTemperature(0, currentTemperature);
        }

        // send Coolant Temperature to CANBUS
        uint64_t raw;
        std::memcpy(&raw, &currentTemperature, sizeof(double));
        CanFrame tempFrame(TEMPERATURE_FRAME_ID, sizeof(double), raw);
        bus.send(tempFrame);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}
