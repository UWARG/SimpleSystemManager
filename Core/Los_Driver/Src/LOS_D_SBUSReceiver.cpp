#include "../Inc/LOS_D_SBUSReceiver.hpp"
#include "../Inc/LOS_D_SBUSSender.hpp"

SBUSReceiver* SBUSReceiver::singleton_ = NULL;

SBUSReceiver* SBUSReceiver::getInstance(UART_HandleTypeDef* uart){
    if (singleton_ == NULL)
    	singleton_ = new SBUSReceiver(uart);

    // returning the instance pointer
    return singleton_;
}

SBUSReceiver::SBUSReceiver(UART_HandleTypeDef* uart) : uart_(uart)
{
    for(int i = 0; i < SBUS_INPUT_CHANNELS; i++)
    {
        received_sbus_.ch[i] = 1000;
    }
    received_sbus_.ch17 = false;
    received_sbus_.ch18 = false;
    received_sbus_.failsafe = false;
    received_sbus_.lost_frame = false;
    received_sbus_.new_data = false;

    HAL_UART_Receive_DMA (uart_, raw_sbus_, SBUS_FRAME_SIZE);
}

SBus SBUSReceiver::GetSBUS(){
	if(received_sbus_.new_data == false)
		HAL_UART_Receive_DMA (uart_, raw_sbus_, SBUS_FRAME_SIZE);
    return received_sbus_;
}

RCControl SBUSReceiver::GetRCControl(){
	if(received_sbus_.new_data == false)
		HAL_UART_Receive_DMA (uart_, raw_sbus_, SBUS_FRAME_SIZE);
    cast_rccontrol();
    return received_rccontrol_;
}

void SBUSReceiver::parse()
{
    if ((raw_sbus_[0] == HEADER_) && (raw_sbus_[24] == FOOTER_)) {

        //exactal parsing
        received_sbus_.ch[0]  = static_cast<int16_t>(raw_sbus_[1] |
                                            ((raw_sbus_[2] << 8) & 0x07FF));
        received_sbus_.ch[1]  = static_cast<int16_t>((raw_sbus_[2] >> 3) |
                                            ((raw_sbus_[3] << 5) & 0x07FF));
        received_sbus_.ch[2]  = static_cast<int16_t>((raw_sbus_[3] >> 6) |
                                            (raw_sbus_[4] << 2) |
                                            ((raw_sbus_[5] << 10) & 0x07FF));
        received_sbus_.ch[3]  = static_cast<int16_t>((raw_sbus_[5] >> 1) |
                                            ((raw_sbus_[6] << 7) & 0x07FF));
        received_sbus_.ch[4]  = static_cast<int16_t>((raw_sbus_[6] >> 4) |
                                            ((raw_sbus_[7] << 4) & 0x07FF));
        received_sbus_.ch[5]  = static_cast<int16_t>((raw_sbus_[7] >> 7) |
                                            (raw_sbus_[8] << 1) |
                                            ((raw_sbus_[9] << 9) & 0x07FF));
        received_sbus_.ch[6]  = static_cast<int16_t>((raw_sbus_[9] >> 2) |
                                            ((raw_sbus_[10] << 6) & 0x07FF));
        received_sbus_.ch[7]  = static_cast<int16_t>((raw_sbus_[10] >> 5) |
                                            ((raw_sbus_[11] << 3) & 0x07FF));
        received_sbus_.ch[8]  = static_cast<int16_t>(raw_sbus_[12] |
                                            ((raw_sbus_[13] << 8) & 0x07FF));
        received_sbus_.ch[9]  = static_cast<int16_t>((raw_sbus_[13] >> 3) |
                                            ((raw_sbus_[14] << 5) & 0x07FF));
        received_sbus_.ch[10] = static_cast<int16_t>((raw_sbus_[14] >> 6) |
                                            (raw_sbus_[15] << 2) |
                                            ((raw_sbus_[16] << 10) & 0x07FF));
        received_sbus_.ch[11] = static_cast<int16_t>((raw_sbus_[16] >> 1) |
                                            ((raw_sbus_[17] << 7) & 0x07FF));
        received_sbus_.ch[12] = static_cast<int16_t>((raw_sbus_[17] >> 4) |
                                            ((raw_sbus_[18] << 4) & 0x07FF));
        received_sbus_.ch[13] = static_cast<int16_t>((raw_sbus_[18] >> 7) |
                                            (raw_sbus_[19] << 1) |
                                            ((raw_sbus_[20] << 9) & 0x07FF));
        received_sbus_.ch[14] = static_cast<int16_t>((raw_sbus_[20] >> 2) |
                                            ((raw_sbus_[21] << 6) & 0x07FF));
        received_sbus_.ch[15] = static_cast<int16_t>((raw_sbus_[21] >> 5) |
                                            ((raw_sbus_[22] << 3) & 0x07FF));

        /* CH 17 */
        received_sbus_.ch17 = raw_sbus_[23] & CH17_MASK_;
        /* CH 18 */
        received_sbus_.ch18 = raw_sbus_[23] & CH18_MASK_;
        /* Grab the lost frame */
        received_sbus_.lost_frame = raw_sbus_[23] & LOST_FRAME_MASK_;
        /* Grab the failsafe */
        received_sbus_.failsafe = raw_sbus_[23] & FAILSAFE_MASK_;

        received_sbus_.new_data = true;

    }else{
    	received_sbus_.new_data = false;
    }
}

void SBUSReceiver::cast_rccontrol()
{
    for(uint8_t i = 0; i < 16; i++){
        received_rccontrol_.ControlSignals[i] = sbus_to_rccontrol(received_sbus_.ch[i]);
    }
}

 float SBUSReceiver::sbus_to_rccontrol(uint16_t channel_value)
 {
    if(channel_value < SBUS_RANGE_MIN)
        channel_value = SBUS_RANGE_MIN;
    if(channel_value > SBUS_RANGE_MAX)
        channel_value = SBUS_RANGE_MAX;
    return static_cast<float>((channel_value - SBUS_RANGE_MIN) * (100.0f / SBUS_RANGE_RANGE));
 }

 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_SET);
	 SBUSReceiver::getInstance(huart)->parse();
	 HAL_UART_Receive_DMA (huart, SBUSReceiver::getInstance(huart)->raw_sbus_, SBUS_FRAME_SIZE);
	 SBUSSender::getInstance(huart)->assemble_packet();
	 HAL_UART_Transmit(huart, SBUSSender::getInstance(huart)->send_buf_, SBUS_FRAME_SIZE, 10);
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_RESET);
 }

 void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
 {
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_SET);
	 HAL_UART_DMAStop(huart);
	 HAL_UART_Receive_DMA(huart, SBUSReceiver::getInstance(huart)->raw_sbus_, SBUS_FRAME_SIZE);
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_RESET);
 }
