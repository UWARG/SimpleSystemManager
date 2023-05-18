#ifndef SBUS_RECEIVER_HPP
#define SBUS_RECEIVER_HPP

#include "main.h"
#include "usart.h"
#include "CommonDataTypes.h"
#include "SBUSCommenDefines.h"

class SBUSReceiver{
    public:
    /*
        serve for singleton structure application
        the code doesn't allow the class to make copy of its instance
    */
        SBUSReceiver (const SBUSReceiver*) = delete;
        SBUSReceiver &operator= (const SBUSReceiver&) = delete;

        static SBUSReceiver* getInstance(UART_HandleTypeDef* uart);

    /*
        get the sbus data
        @return SBus struct
    */
        SBus GetSBUS();

    /*
        get the RCControl data that is parsed from sbus
        more readable data that varies from 0 to 100
        @return RCControl struct
    */
        RCControl GetRCControl();


    private:

    /*
        object constructor
        @param uart instance ie. &huart1
    */
        SBUSReceiver(UART_HandleTypeDef* uart);
    
    // member variables
        static SBUSReceiver* singleton_;
        uint8_t raw_sbus_[SBUS_FRAME_SIZE];
        UART_HandleTypeDef* uart_;
        SBus received_sbus_;
        RCControl received_rccontrol_;

    /* Parsing state tracking */
        int8_t state_ = 0;
        uint8_t prev_byte_ = FOOTER_;
        uint8_t cur_byte_;

    //private functions
        void read();
        bool parse();
        void cast_rccontrol();
        float sbus_to_rccontrol(uint16_t channel_value);

};


#endif
