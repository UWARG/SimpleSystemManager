#ifndef SBUS_SENDER_HPP
#define SBUS_SENDER_HPP

#include "main.h"
#include "usart.h"
#include "CommonDataTypes.h"
#include "SBUSCommenDefines.h"

class SBUSSender{
    public:
	/*
	 	public variable
	 */
		uint8_t send_buf_[SBUS_FRAME_SIZE];

    /*
        serve for singleton structure application
        the code doesn't allow the class to make copy of its instance
    */
        SBUSSender (const SBUSSender*) = delete;
        SBUSSender &operator= (const SBUSSender&) = delete;

        static SBUSSender* getInstance(UART_HandleTypeDef* uart);


    /* 
        select the channel from 1 - 16
        set the value from 0 to 100 float as percentage
    */
        void SetChannelValue(uint8_t channel, float value);

    /*
        directly setup a whole sbus message
    */
        void SetSBusValue(SBus values);


    /*
        setup channel percentage values and parse it to sbus message
    */
        void SetRCControlValue(RCControl values);


        void assemble_packet();

    /*
        send out the configured data as an UART/SBUS message
    */
        //void SendData(); //deleted because now it uses dma to send automatically

    private:
    /* constructor*/
        SBUSSender(UART_HandleTypeDef* uart);
    // member variables
        static SBUSSender* singleton_;
        UART_HandleTypeDef* uart_;
        SBus send_sbus_;

    //helping functions
        uint16_t rccontrol_to_sbus(float rccontrol);
};


#endif 
