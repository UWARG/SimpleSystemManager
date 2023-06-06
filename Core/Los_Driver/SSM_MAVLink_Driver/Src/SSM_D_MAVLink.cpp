/*
 * SSM_D_MAVLink.cpp
 *
 *  Created on: May 26, 2023
 *
 */

#include "../Inc/SSM_D_MAVLink.hpp"

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

bool MAVLink::readMessage(mavlink_message_t& message)
{
    //if Data Reception process is not ongoing, then activate it
    if(uart_->RxState != HAL_UART_STATE_BUSY_RX){
        HAL_UARTEx_ReceiveToIdle_DMA(uart_, raw_rx_msg_, sizeof(raw_rx_msg_));
    }

    uint8_t byte = 0;
	mavlink_status_t status = {};
    mavlink_message_t rx_msg;

    while(rx_circular_buffer_->read(&byte, 1)){
        if( mavlink_parse_char(MAVLINK_COMM_1, byte, &(rx_msg), &status)){
            mavlink_reset_channel_status(MAVLINK_COMM_1);
            message = rx_msg;

            return true;
        }
    }
    return false;
}

void MAVLink::writeMessage(const mavlink_message_t output_message){
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &output_message);
	HAL_StatusTypeDef ret = HAL_UART_Transmit(uart_, buf, len, 1000);
}





