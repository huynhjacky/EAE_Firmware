#ifndef __CANFRAME_HPP__
#define __CANFRAME_HPP__

#include <stdint.h>

/**
 * @brief CAN Frame Object - Object to simulate a CAN Frame
 * 
 * References Used - https://medium.com/@sjindhirapooja/can-standard-data-frame-format-846b8f9fc749
 */
class CanFrame
{

public:
    // CAN Frame Fields

    //uint8_t sof;
    uint32_t id;
    //uint8_t rtr;
    //uint8_t ide;
    //uint8_t r0;
    //uint8_t r1;
    uint8_t dlc;
    uint64_t data;
    //uint16_t crc;
    //uint8_t ack;
    //uint8_t eof;
    //uint8_t ifs;

    CanFrame(//uint8_t sof = 0,
             uint32_t id = 0,
             //uint8_t rtr = 0,
             //uint8_t ide = 0,
             //uint8_t r0 = 0,
             //uint8_t r1 = 0,
             uint8_t dlc = 0,
             uint64_t data = 0
             //uint16_t crc = 0,
             //uint8_t ack = 0,
             //uint8_t eof = 0,
             //uint8_t ifs = 0
            ) :
            //sof(sof),
            id(id),
            //rtr(rtr),
            //ide(ide),
            //r0(r0),
            //r1(r1),
            dlc(dlc),
            data(data)
            //crc(crc),
            //ack(ack),
            //eof(eof),
            //ifs(ifs)
    {};

};

#endif /* __CANFRAME_HPP__ */
