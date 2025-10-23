#ifndef __SENSORNODE_HPP__
#define __SENSORNODE_HPP__

#include "canbus.hpp"
#include <iostream>
#include <chrono>

#include <atomic>

/**
 * @brief Temperature Sensor Class - Object to simulate a Temperature Sensor reading temperature of coolant
 * 
 * References Used - 
 */
class Sensor
{
public:

    /**
     * @brief Constructor
     * @param bus CANBUS to send and receive frames
     */
    Sensor(CanBus& b);

    /**
     * @brief stops Temperature Sensor
     */
    void stop() { exit = true; }

    /**
     * @brief runs the Temperature Sensor
     */
    void run();

private:

    std::atomic<bool> exit{false};

    // CANBUS Object to send/receive frames
    CanBus& bus;
    
    // the current temperature of Coolant
    // NOTE: initialized to 25C to match ambient temperature of 25C
    double currentTemperature = 25;

    /**
     * @brief Calculates temperature of coolant based on the Fan Speed
     * @param fanSpeed Speed of Fan
     * @param currentTemperature Current Temperature of Coolant
     */
    static double calculateTemperature(double fanSpeed, double currentTemperature)
    {
        const double ambient = 30;
        const double heatingRate = 0.05;
        const double coolingRate = 0.25;

        // Add Heat
        currentTemperature += heatingRate;

        // Subtract to cool based on fanSpeed
        currentTemperature -= (fanSpeed / 255.0) * coolingRate;
        
        // coolant cannot be hotter than ambient temperature
        if (currentTemperature >= ambient)
            currentTemperature = ambient;

        return currentTemperature;
    }

};

#endif /* __SENSORNODE_HPP__ */