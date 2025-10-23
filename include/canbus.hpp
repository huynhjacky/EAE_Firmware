#ifndef __CANBUS_HPP__
#define __CANBUS_HPP__

#include "canframe.hpp"
#include <queue>
#include <mutex>

/**
 * @brief CANBUS Class - Object to simulate CANBUS
 */
class CanBus
{
public:
    /**
     * @brief Constructor
     */
    CanBus ();

    /**
     * @brief sends data to bus
     * @param frame CAN frame to send to
     */
    void send(const CanFrame& frame);

    /**
     * @brief receive frames from bus
     * @param frame read frame
     */
    void receive(CanFrame& frame);

private:
    // queue to simulate bus
    std::queue<CanFrame> bus;

    // lock to add concurrency between send and read methods
    std::mutex busLock;
};

#endif /* __CANBUS_HPP__ */
