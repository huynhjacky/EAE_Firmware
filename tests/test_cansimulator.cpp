#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <cstring>

#include "canbus.hpp"
#include "controllernode.hpp"
#include "sensornode.hpp"
#include "canframe.hpp"

const uint32_t FANSPEED_FRAME_ID = 0x200;
const uint32_t TEMPERATURE_FRAME_ID = 0x100;

// Test Send and Receive calls on CANBUS
TEST(CanBusSimulatorTest, SendReceiveTest)
{
    CanBus bus;

    // send CAN Frame to CANBUS
    uint32_t id = 0x110;
    uint64_t raw = 1;
    CanFrame sendFrame(id, sizeof(uint64_t), raw);
    bus.send(sendFrame);

    // read CAN Frame
    CanFrame receiveFrame;
    bus.receive(receiveFrame);

    // check that received frame is correct
    ASSERT_GT(receiveFrame.dlc, 0);
    ASSERT_EQ(receiveFrame.id, id);
    ASSERT_EQ(receiveFrame.data, 1);
}

// Test Receiving empty CAN frame
TEST(CanBusSimulatorTest, ReceiveEmptyBusTest)
{
    CanBus bus;
    
    // read CAN Frame
    CanFrame receiveFrame;
    bus.receive(receiveFrame);

    // Check that it's empty
    ASSERT_EQ(receiveFrame.dlc, 0);
    ASSERT_EQ(receiveFrame.id, 0);
    ASSERT_EQ(receiveFrame.data, 0);
}

// Test Turning On Fan Controller
TEST(CanBusSimulatorTest, FanControllerOnTest)
{
    // initialize bus and controller threads
    CanBus bus;
    std::unique_ptr<Controller> fanController = std::make_unique<Controller>(bus);
    std::thread controllerThread(&Controller::run, fanController.get());

    // attempt 10 times to start Fan Controller by setting the set point to 0C
    int attempts = 0;
    bool fanOn = false;
    while (attempts < 10)
    {
        // set setpoint to 0C
        fanController->setSetPoint(0);

        // send CAN frame to CANBUS, so that Fan Controller will think that the current temperature is 25C
        uint64_t raw;
        double setTemperature = 25;
        std::memcpy(&raw, &setTemperature, sizeof(double));
        CanFrame tempFrame(TEMPERATURE_FRAME_ID, sizeof(double), raw);
        bus.send(tempFrame);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Read CANBUS to see if there are frames from the Fan Controller, and check that the Fan is spinning
        CanFrame receiveFrame;
        bus.receive(receiveFrame);
        if (receiveFrame.id == FANSPEED_FRAME_ID)
        {
            ASSERT_GT(receiveFrame.dlc, 0);
            double fanSpeed;
            std::memcpy(&fanSpeed, &receiveFrame.data, sizeof(double));
            ASSERT_GT(fanSpeed, 0);
            fanOn = true;
            break;
        }
        attempts++;
    }
    // did fan come on?
    ASSERT_TRUE(fanOn);

    // cleanup
    fanController->stop();
    if (controllerThread.joinable())
    {
        controllerThread.join();
    }
}

// Test Turning Off Fan Controller
TEST(CanBusSimulatorTest, FanControllerFanOffTest)
{
    // initialize bus and controller threads
    CanBus bus;
    std::unique_ptr<Controller> fanController = std::make_unique<Controller>(bus);
    std::thread controllerThread(&Controller::run, fanController.get());

    // attempt 10 times to stop Fan Controller by setting the set point to 100C
    int attempts = 0;
    bool fanOff = false;
    while (attempts < 10)
    {
        // set setpoint to 100C
        fanController->setSetPoint(100);

        // send CAN frame to CANBUS, so that Fan Controller will think that the current temperature is 0C
        uint64_t raw;
        double setTemperature = 0;
        std::memcpy(&raw, &setTemperature, sizeof(double));
        CanFrame tempFrame(TEMPERATURE_FRAME_ID, sizeof(double), raw);
        bus.send(tempFrame);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Read CANBUS to see if there are frames from the Fan Controller, and check that the Fan is not spinning
        CanFrame receiveFrame;
        bus.receive(receiveFrame);
        if (receiveFrame.id == FANSPEED_FRAME_ID)
        {
            ASSERT_GT(receiveFrame.dlc, 0);
            double fanSpeed;
            std::memcpy(&fanSpeed, &receiveFrame.data, sizeof(double));
            ASSERT_EQ(fanSpeed, 0);
            fanOff = true;
            break;
        }
        attempts++;
    }
    // did fan turn off?
    ASSERT_TRUE(fanOff);

    // cleanup
    fanController->stop();
    if (controllerThread.joinable())
    {
        controllerThread.join();
    }
}

// Test Turning Temperature sensors detects cooling
TEST(CanBusSimulatorTest, TemperatureSensorCoolingTest)
{
    // initialize bus and sensor thread
    CanBus bus;
    std::unique_ptr<Sensor> temperatureSensor = std::make_unique<Sensor>(bus);
    std::thread sensorThread(&Sensor::run, temperatureSensor.get());

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // get the current temperature of coolant from CANBUS
    CanFrame receiveFrame;
    bus.receive(receiveFrame);
    double currentTemperature;
    std::memcpy(&currentTemperature, &receiveFrame.data, sizeof(double));

    // attempt 10 times to see that temperature sensor is detecting that coolant temperature is going down by setting the fan speed to 10000
    int attempts = 0;
    bool cooling = false;
    while (attempts < 10)
    {
        // send CAN frame, so that temperature sensor thinks that the fan speed is 10000
        uint64_t raw;
        double setFanSpeed = 10000;
        std::memcpy(&raw, &setFanSpeed, sizeof(double));
        CanFrame fanspeedFrame(FANSPEED_FRAME_ID, sizeof(double), raw);
        bus.send(fanspeedFrame);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Read CANBUS to see if there are frames from the temperature sensor, and check that coolant temperature has gone down
        bus.receive(receiveFrame);
        if (receiveFrame.id == TEMPERATURE_FRAME_ID)
        {
            ASSERT_GT(receiveFrame.dlc, 0);
            double coolantTemperature;
            std::memcpy(&coolantTemperature, &receiveFrame.data, sizeof(double));
            ASSERT_LT(coolantTemperature, currentTemperature);
            cooling = true;
            break;
        }
    }

    // did coolant cool?
    ASSERT_TRUE(cooling);

    // cleanup
    temperatureSensor->stop();
    if (sensorThread.joinable())
    {
        sensorThread.join();
    }
}

// Test Turning Temperature sensors detects heating
TEST(CanBusSimulatorTest, TemperatureSensorHeatingTest)
{
    // initialize bus and sensor thread
    CanBus bus;
    std::unique_ptr<Sensor> temperatureSensor = std::make_unique<Sensor>(bus);
    std::thread sensorThread(&Sensor::run, temperatureSensor.get());

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // get the current temperature of coolant from CANBUS
    CanFrame receiveFrame;
    bus.receive(receiveFrame);
    double currentTemperature;
    std::memcpy(&currentTemperature, &receiveFrame.data, sizeof(double));

    // attempt 10 times to see that temperature sensor is detecting that coolant temperature is going up by setting the fan speed to 0
    int attempts = 0;
    bool heating = false;
    while (attempts < 10)
    {
        // send CAN frame, so that temperature sensor thinks that the fan speed is 0
        uint64_t raw;
        double setFanSpeed = 0;
        std::memcpy(&raw, &setFanSpeed, sizeof(double));
        CanFrame fanspeedFrame(FANSPEED_FRAME_ID, sizeof(double), raw);
        bus.send(fanspeedFrame);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Read CANBUS to see if there are frames from the temperature sensor, and check that coolant temperature has gone up
        bus.receive(receiveFrame);
        if (receiveFrame.id == TEMPERATURE_FRAME_ID)
        {
            ASSERT_GT(receiveFrame.dlc, 0);
            double coolantTemperature;
            std::memcpy(&coolantTemperature, &receiveFrame.data, sizeof(double));
            ASSERT_GT(coolantTemperature, currentTemperature);
            heating = true;
            break;
        }
    }

    // did coolant heat?
    ASSERT_TRUE(heating);

    // cleanup
    temperatureSensor->stop();
    if (sensorThread.joinable())
    {
        sensorThread.join();
    }
}
