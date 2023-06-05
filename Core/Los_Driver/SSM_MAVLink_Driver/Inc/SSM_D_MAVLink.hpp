/*
 * SSM_D_MAVLink.hpp
 *
 *  Created on: May 26, 2023
 */

/*
    Be Careful when configure the uart for this driver
    The baudrate of the uart has to be corresponds to the ardupilot/ground side setting
*/

#ifndef LOS_DRIVER_SSM_MAVLINK_DRIVER_INC_SSM_D_MAVLINK_HPP_
#define LOS_DRIVER_SSM_MAVLINK_DRIVER_INC_SSM_D_MAVLINK_HPP_

#include "MAVLink/ardupilotmega/mavlink.h"
#include "main.h"
#include <string.h>
#include "../../Common/Inc/CircularBuffer.hpp"
//#include "../../Common/Inc/CommonDataTypes.h"

class MAVLink {
	public:

        uint8_t rx_circular_buffer_ptr_[1000];
        CircularBuffer* rx_circular_buffer_;
        uint8_t raw_rx_msg_[MAVLINK_MAX_PACKET_LEN];
        bool is_new_;
        
        /* Constructor */
		MAVLink(UART_HandleTypeDef* uart_handle);
        ~MAVLink();

        /* 
            a helper function on receiveing and parsing data
            need to be constantly called in a loop to constantly parsing the incoming message
        */
        bool readMessage(mavlink_message_t& message);

        void writeMessage(const mavlink_message_t output_message);

	private:
        UART_HandleTypeDef* uart_;
};

/*
    TODO:
    1. receive message through DMA
    2. Identify the completeness of the message
    3. Store the raw data in a struct

    Go to the ssm

    0. The messages are stored in queue
    1. Identify the message source
    2. Forward the data to the correct terminal 
*/

#endif /* LOS_DRIVER_SSM_MAVLINK_DRIVER_INC_SSM_D_MAVLINK_HPP_ */
