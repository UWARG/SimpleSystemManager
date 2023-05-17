#ifndef COMMON_DATA_TYPES_H
#define COMMON_DATA_TYPES_H

#include <cstdint>

#define SBUS_INPUT_CHANNELS	16

struct SBus{
    uint16_t ch[SBUS_INPUT_CHANNELS]; //value from 192 - 1792, can be littel off
    bool lost_frame;
    bool failsafe;
    bool ch17, ch18;
    bool new_data;
};

/*  a struct for control signal channel mapping and actual attributes*/
struct RCControl{
    uint16_t ControlSignals[16];
    float Ali = ControlSignals[0];
    // ...
    

};

#endif