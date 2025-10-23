#include "controllernode.hpp"
#include <chrono>
#include <thread>
#include <iostream>
#include <cstring>

const uint32_t FANSPEED_FRAME_ID = 0x200;
const uint32_t TEMPERATURE_FRAME_ID = 0x100;

//--------------------------------------------------------------------------------------
// See header file controllernode.hpp for description
//--------------------------------------------------------------------------------------
Controller::Controller(CanBus& b)
    : bus(b)
{
}

//--------------------------------------------------------------------------------------
// See header file controllernode.hpp for description
// NOTE: For this function I referenced - https://codepal.ai/code-generator/query/HV9eDXO9/pid-algorithm-cpp-function
//--------------------------------------------------------------------------------------
double Controller::PIDController::calculate(double measured)
{
    std::cout << "[Controller] set point is " << setPoint << "C" << std::endl;

    // Compute error
    double error = measured - setPoint;

    // Proportional
    double P = Kp * error;

    // Integral
    integral += error * dt;
    double I = Ki * integral;

    // Derivative
    double D = Kd * (error - preverror) / dt;

    double output = P + I + D;

    preverror = error;

    return output;
}

//--------------------------------------------------------------------------------------
// See header file controllernode.hpp for description
//--------------------------------------------------------------------------------------
double Controller::processFrame(const CanFrame& frame)
{
    // read coolant temperature from CAN frame
    double temperature;
    std::memcpy(&temperature, &frame.data, sizeof(double));
    std::cout << "[Controller] Coolant Temperature is " << temperature << "C" << std::endl;

    // pass coolant temperature to PIDController to calculate fan speed
    double fanSpeed = 0;
    fanSpeed = pidController.calculate(temperature);
    return fanSpeed;
}

//--------------------------------------------------------------------------------------
// See header file controllernode.hpp for description
//--------------------------------------------------------------------------------------
void Controller::run()
{
    while (true)
    {
        // exit signal detected
        if (exit)
        {
            return;
        }

        // read for Coolant Temperature CAN frame, and process frame to get Fan speed
        double fanSpeed = 0;
        CanFrame frame;
        bus.receive(frame);
        if (frame.id == TEMPERATURE_FRAME_ID && frame.dlc > 0)
        {
            fanSpeed = processFrame(frame);
        }

        // States of Fan Controller depending on Fan Speed
        switch (state)
        {
        // initialize Fan controller by setting all defaults
        case ControllerState::INIT: 
            std::cout << "[Controller] State: INIT" << std::endl;
            pidController.setPoint = 10;
            pidController.preverror = 0;
            pidController.integral = 0;
            state = ControllerState::IDLE;
            break;

        // When Fan Speed is 0 Fan Controller is Idle
        case ControllerState::IDLE:
            std::cout << "[Controller] State: IDLE" << std::endl;

            // if Fan turns on switch to COOLING state
            if (fanSpeed > 0)
            {
                state = ControllerState::COOLING;
            }
            else
            {
                fanSpeed = 0;
                // send Fan Speed to CANBUS
                uint64_t raw;
                std::memcpy(&raw, &fanSpeed, sizeof(double));
                CanFrame speedFrame(FANSPEED_FRAME_ID, sizeof(double), raw);
                bus.send(speedFrame);
            }
            break;

        // When Fan speed is greater than 0 Fan Controller is Cooling
        case ControllerState::COOLING:
            std::cout << "[Controller] State: COOLING" << std::endl;

            // if Fan is off switch to IDLE state
            if (fanSpeed <= 0)
            {
                state = ControllerState::IDLE;
                fanSpeed = 0;
            }
            else
            {
                // send Fan Speed to CANBUS
                uint64_t raw;
                std::memcpy(&raw, &fanSpeed, sizeof(double));
                CanFrame speedFrame(FANSPEED_FRAME_ID, sizeof(double), raw);
                bus.send(speedFrame);
            }
            break;

        default:
            break;
        }

        std::cout << "[Controller] Fan speed is " << fanSpeed << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}
