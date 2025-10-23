#include "canbus.hpp"
#include "sensornode.hpp"
#include "controllernode.hpp"

#include <thread>
#include <iostream>
#include <string>
#include <memory>

int main()
{
    // init CANBUS
    CanBus bus;

    // Create objects
    std::unique_ptr<Sensor> sensor = std::make_unique<Sensor>(bus);
    std::unique_ptr<Controller> controller = std::make_unique<Controller>(bus);

    // Run Temperature Sensor and Fan Controller loops in separate threads
    std::thread sensorThread(&Sensor::run, sensor.get());
    std::thread controllerThread(&Controller::run, controller.get());

    // Main thread: read user input for set point
    std::string input;
    while (true)
    {
        std::getline(std::cin, input);
        try {
            if (input == "exit")
            {
                sensor->stop();
                controller->stop();
            }
            int setPoint = std::stoi(input);
            controller->setSetPoint(setPoint);
        } catch (...) {
            std::cout << "Invalid input, enter a number" << std::endl;
        }
    }
}
