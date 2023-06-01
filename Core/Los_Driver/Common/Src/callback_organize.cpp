
#include "../../SSM_MAVLink_Driver/Inc/SSM_D_MAVLink.hpp"
#include "../../SBUS/Inc/LOS_D_SBUSReceiver.hpp"
#include "../../SBUS/Inc/LOS_D_SBUSSender.hpp"
#include "../Inc/driver_config.hpp"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size){
    /*TODO:
        1. separate differet uart cases in the callback function
        2. organize the mapping of uart to different devices in some common file
        3.figure out why uart_cpcallback is called
        4.change the class such that this is not singleton


    */ 

    
    if(huart == pixhawk_mavlink_uart){
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET); //turn on green light

        // //parse the data from buffer
    	/* the parsing function is not working, but this needs to be moved to else place anyway*/
        pixhawk_mavlink->readMessage();

        // //reset the status of parsing state machine
        mavlink_reset_channel_status(MAVLINK_COMM_1);


        // for (uint16_t i = 0; i < size; i++)
        // {
        //     MAVLink::getInstance(huart)->rx_circular_buffer_->write(MAVLink::getInstance(huart)->raw_rx_msg_[i]);
        // }
        //listen to more data
        HAL_UARTEx_ReceiveToIdle_DMA(huart, pixhawk_mavlink->raw_rx_msg_, MAVLINK_MAX_PACKET_LEN);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET); //turn off green light
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_RESET); //turn off red light
    }


}

 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
    if(huart == sbus_uart){
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_RESET); //turn off red light
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_SET); //turn on blue light
        SBUSReceiver::getInstance(huart)->parse();
        HAL_UART_Receive_DMA (huart, SBUSReceiver::getInstance(huart)->raw_sbus_, SBUS_FRAME_SIZE);
        SBUSSender::getInstance(huart)->assemble_packet();
        HAL_UART_Transmit(huart, SBUSSender::getInstance(huart)->send_buf_, SBUS_FRAME_SIZE, 10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_RESET); //turn off blue light
    }

    if(huart == pixhawk_mavlink_uart){
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_SET); //turn on blue light
        /*
            should never enter this callback
        */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_RESET); //turn off blue light
    }

 }

 void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
 {
    if(huart == sbus_uart){
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_SET); //turn on red light
        HAL_UART_DMAStop(huart);
        HAL_UART_Receive_DMA(huart, SBUSReceiver::getInstance(huart)->raw_sbus_, SBUS_FRAME_SIZE);
    }
    if(huart == pixhawk_mavlink_uart){
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_SET); //turn on red light
        HAL_UART_DMAStop(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, pixhawk_mavlink->raw_rx_msg_, MAVLINK_MAX_PACKET_LEN);
    }


 }
