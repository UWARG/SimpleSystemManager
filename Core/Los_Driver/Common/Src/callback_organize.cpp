
#include "../../SSM_MAVLink_Driver/Inc/SSM_D_MAVLink.hpp"
#include "../../SBUS/Inc/LOS_D_SBUSReceiver.hpp"
#include "../../SBUS/Inc/LOS_D_SBUSSender.hpp"
#include "../Inc/driver_config.hpp"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size){
    
    if(huart == pixhawk_mavlink_uart){
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET); //turn on green light

        for (uint16_t i = 0; i < size; i++)
        {
            pixhawk_mavlink->rx_circular_buffer_->write(pixhawk_mavlink->raw_rx_msg_[i]);
        }
        //listen to more data
        HAL_UARTEx_ReceiveToIdle_DMA(huart, pixhawk_mavlink->raw_rx_msg_, sizeof(pixhawk_mavlink->raw_rx_msg_));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET); //turn off green light
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_RESET); //turn off red light
    }

    if(huart == ground_mavlink_uart){
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET); //turn on green light

        for (uint16_t i = 0; i < size; i++)
        {
            ground_mavlink->rx_circular_buffer_->write(ground_mavlink->raw_rx_msg_[i]);
        }
        //listen to more data
        HAL_UARTEx_ReceiveToIdle_DMA(huart, ground_mavlink->raw_rx_msg_, sizeof(pixhawk_mavlink->raw_rx_msg_));
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

        for (uint16_t i = 0; i < sizeof(pixhawk_mavlink->raw_rx_msg_); i++)
        {
            ground_mavlink->rx_circular_buffer_->write(ground_mavlink->raw_rx_msg_[i]);
        }

        //listen to more data
        HAL_UARTEx_ReceiveToIdle_DMA(huart, ground_mavlink->raw_rx_msg_, sizeof(pixhawk_mavlink->raw_rx_msg_));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_RESET); //turn off blue light
    }

    if(huart == ground_mavlink_uart){
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_SET); //turn on blue light

        
        for (uint16_t i = 0; i < sizeof(pixhawk_mavlink->raw_rx_msg_); i++)
        {
            ground_mavlink->rx_circular_buffer_->write(ground_mavlink->raw_rx_msg_[i]);
        }

        //listen to more data
        HAL_UARTEx_ReceiveToIdle_DMA(huart, ground_mavlink->raw_rx_msg_, sizeof(pixhawk_mavlink->raw_rx_msg_));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

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
        HAL_UARTEx_ReceiveToIdle_DMA(huart, pixhawk_mavlink->raw_rx_msg_, MAVLINK_MAX_PACKET_LEN);
    }
    if(huart == ground_mavlink_uart){
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_SET); //turn on red light
        HAL_UART_DMAStop(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, ground_mavlink->raw_rx_msg_, MAVLINK_MAX_PACKET_LEN);
    }

 }
