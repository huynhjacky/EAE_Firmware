#ifndef __CONTROLLERNODE_HPP__
#define __CONTROLLERNODE_HPP__

#include "canbus.hpp"
#include "canframe.hpp"

#include <atomic>

/**
 * @brief Fan Controller Class - Object to simulate a Fan Controller connected to CAN
 * 
 * References Used - https://codepal.ai/code-generator/query/HV9eDXO9/pid-algorithm-cpp-function
 */
class Controller
{
public:

    /**
     * @brief Constructor
     * @param bus CANBUS to send and receive frames
     */
    Controller(CanBus& bus);

    /**
     * @brief function to set setpoint
     * @param setPointVal value to set setpoint to
     */
    void setSetPoint(double setPointVal) { pidController.setPoint = setPointVal; };

    /**
     * @brief stops Fan Controller
     */
    void stop() { exit = true; }

    /**
     * @brief runs the Fan Controller
     */
    void run();

private:

    std::atomic<bool> exit{false};
 
    // Fan Controller states
    enum class ControllerState {
        INIT,
        IDLE,
        COOLING
    };
    // initialize Fan Controller state to INIT
    ControllerState state = ControllerState::INIT;

    // PID Controller object
    struct PIDController
    {
        // PID Controller tuning parameters
        // NOTE: setpoint is defaulted to 10C degrees
        const double dt = 1.0;
        const double Kp = 100;
        const double Ki = 4.0;
        const double Kd = 0.5;
        double setPoint = 10;
        double preverror = 0;
        double integral = 0;
        
        /**
         * @brief function to calculate output of Fan Speed
         * @param measured measured Coolant Temperature
         */
        double calculate(double measured);
    } pidController;

    /**
     * @brief function to read and process CAN frame containing Coolant Temperature
     * @param frame Frame containing Coolant Temperature
    */
    double processFrame(const CanFrame& frame);

    // CANBUS Object to send/receive frames
    CanBus& bus;
};

#endif /* __CONTROLLERNODE_HPP__ */