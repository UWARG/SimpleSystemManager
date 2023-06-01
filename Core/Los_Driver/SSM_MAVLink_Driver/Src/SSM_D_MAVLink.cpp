/*
 * SSM_D_MAVLink.cpp
 *
 *  Created on: May 26, 2023
 *
 */

#include "../Inc/SSM_D_MAVLink.hpp"
/*
    many struct types used here are referenced from mavlink_types.h
*/

MAVLink::MAVLink(UART_HandleTypeDef* uart_handle) : uart_(uart_handle)
{
    rx_circular_buffer_ = new CircularBuffer(rx_circular_buffer_ptr_, 1000);

    for(int i = 0; i < MAVLINK_MAX_PACKET_LEN; i++){
        raw_rx_msg_[MAVLINK_MAX_PACKET_LEN] = 0;
    }
}

MAVLink::~MAVLink(){
    delete rx_circular_buffer_;
}

mavlink_message_t* MAVLink::get_mavlink_msg(){
    //if Data Reception process is not ongoing, then activate it
    if(uart_->RxState != HAL_UART_STATE_BUSY_RX){
        HAL_UARTEx_ReceiveToIdle_DMA(uart_, raw_rx_msg_, MAVLINK_MAX_PACKET_LEN);
    }
    return &rx_msg_;
}

bool MAVLink::readMessage()
{
    const uint8_t STX = 0xFD;
    uint8_t payload_length = 0;
    uint8_t byte = 0;
	mavlink_status_t status = {};
    mavlink_message_t rx_msg;

    //part 1 - a quick check to  see if the data is okay
    if(raw_rx_msg_[0] == STX){

        //part 2 - parse the message
        payload_length = raw_rx_msg_[1];

        uint8_t parse_ret;
        for(uint16_t i = 0; i < (payload_length  + MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES); i++)
        {
            parse_ret = mavlink_parse_char(MAVLINK_COMM_1, byte, &(rx_msg), &status);
        }

        //part3 - check validity again
        if(parse_ret){
            rx_msg_ = rx_msg;
            return true;
        }
        else {
            return false;
        }
            
    } else {
        return false;
    }
}





