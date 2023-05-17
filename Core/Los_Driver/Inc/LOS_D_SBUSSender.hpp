#ifndef SBUS_SENDER_HPP
#define SBUS_SENDER_HPP

#include "main.h"
#include "usart.h"
#include "CommonDataTypes.h"
#include "SBUSCommenDefines.h"

class SBUSSender{
    public:
        SBUSSender(UART_HandleTypeDef* uart);

        /* 
            select the channel from 1 - 16
            set the value from 192 - 1792
        */
        void SetChannelValue(uint8_t channel, uint16_t value);

        /*
            directly setup a whole sbus message
        */
        void SetSBusValue(SBus values);

        /*
            send out the configured data as an UART/SBUS message
        */
        void SendData();

    private:
        // member variables
        UART_HandleTypeDef* uart_;
        SBus send_sbus_;
        uint8_t send_buf_[SBUS_FRAME_SIZE];

        //helping functions
        void assemble_packet();

};


#endif 