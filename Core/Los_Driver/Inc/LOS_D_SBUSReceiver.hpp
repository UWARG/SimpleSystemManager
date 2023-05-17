#ifndef SBUS_RECEIVER_HPP
#define SBUS_RECEIVER_HPP

#include "main.h"
#include "usart.h"
#include "CommonDataTypes.h"
#include "SBUSCommenDefines.h"

class SBUSReceiver{
    public:

    /*
        object constructor
        @param uart instance ie. &huart1
    */
        SBUSReceiver(UART_HandleTypeDef* uart);
    /*
        get the sbus data
        @return SBus struct
    */
        SBus GetResult();


    private:
    // member variables
        uint8_t raw_sbus_[SBUS_FRAME_SIZE];
        UART_HandleTypeDef* uart_;
        SBus received_sbus_;

    /* Parsing state tracking */
        int8_t state_ = 0;
        uint8_t prev_byte_ = FOOTER_;
        uint8_t cur_byte_;

    //private functions
        void read();
        bool parse();
};


#endif
