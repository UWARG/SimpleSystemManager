#include "../Inc/LOS_D_SBUSSender.hpp"

SBUSSender* SBUSSender::singleton_ = NULL;

SBUSSender* SBUSSender::getInstance(UART_HandleTypeDef* uart){
    if (singleton_ == nullptr)
      singleton_ = new SBUSSender(uart);
       
    // returning the instance pointer
    return singleton_;
}

SBUSSender::SBUSSender(UART_HandleTypeDef* uart) : uart_(uart){
	for(int i = 0; i < SBUS_INPUT_CHANNELS; i++)
	{
		send_sbus_.ch[i] = 1000;
	}
	send_sbus_.ch17 = false;
	send_sbus_.ch18 = false;
	send_sbus_.failsafe = false;
	send_sbus_.lost_frame = false;
	send_sbus_.new_data = false;

	HAL_UART_Transmit_DMA(uart_, send_buf_, SBUS_FRAME_SIZE);
}

void SBUSSender::SetChannelValue(uint8_t channel, float value){
	send_sbus_.ch[channel] = rccontrol_to_sbus(value);
}

void SBUSSender::SetSBusValue(SBus values){
    send_sbus_ = values;
}

void SBUSSender::SetRCControlValue(RCControl values)
{
    for(uint8_t i = 0; i < 16; i++){
      send_sbus_.ch[i] = rccontrol_to_sbus(values.ControlSignals[i]);
    }
}

void SBUSSender::assemble_packet()
{
  send_buf_[0] = HEADER_;
  send_buf_[1] = static_cast<uint8_t>((send_sbus_.ch[0] & 0x07FF));
  send_buf_[2] = static_cast<uint8_t>((send_sbus_.ch[0] & 0x07FF) >> 8 |
            (send_sbus_.ch[1] & 0x07FF) << 3);
  send_buf_[3] = static_cast<uint8_t>((send_sbus_.ch[1] & 0x07FF) >> 5 |
            (send_sbus_.ch[2] & 0x07FF) << 6);
  send_buf_[4] = static_cast<uint8_t>((send_sbus_.ch[2] & 0x07FF) >> 2);
  send_buf_[5] = static_cast<uint8_t>((send_sbus_.ch[2] & 0x07FF) >> 10 |
            (send_sbus_.ch[3] & 0x07FF) << 1);
  send_buf_[6] = static_cast<uint8_t>((send_sbus_.ch[3] & 0x07FF) >> 7 |
            (send_sbus_.ch[4] & 0x07FF) << 4);
  send_buf_[7] = static_cast<uint8_t>((send_sbus_.ch[4] & 0x07FF) >> 4 |
            (send_sbus_.ch[5] & 0x07FF) << 7);
  send_buf_[8] = static_cast<uint8_t>((send_sbus_.ch[5] & 0x07FF) >> 1);
  send_buf_[9] = static_cast<uint8_t>((send_sbus_.ch[5] & 0x07FF) >> 9  |
            (send_sbus_.ch[6] & 0x07FF) << 2);
  send_buf_[10] = static_cast<uint8_t>((send_sbus_.ch[6] & 0x07FF) >> 6  |
             (send_sbus_.ch[7] & 0x07FF) << 5);
  send_buf_[11] = static_cast<uint8_t>((send_sbus_.ch[7] & 0x07FF) >> 3);
  send_buf_[12] = static_cast<uint8_t>((send_sbus_.ch[8] & 0x07FF));
  send_buf_[13] = static_cast<uint8_t>((send_sbus_.ch[8] & 0x07FF) >> 8 |
             (send_sbus_.ch[9]  & 0x07FF) << 3);
  send_buf_[14] = static_cast<uint8_t>((send_sbus_.ch[9] & 0x07FF) >> 5 |
             (send_sbus_.ch[10] & 0x07FF) << 6);
  send_buf_[15] = static_cast<uint8_t>((send_sbus_.ch[10] & 0x07FF) >> 2);
  send_buf_[16] = static_cast<uint8_t>((send_sbus_.ch[10] & 0x07FF) >> 10 |
             (send_sbus_.ch[11] & 0x07FF) << 1);
  send_buf_[17] = static_cast<uint8_t>((send_sbus_.ch[11] & 0x07FF) >> 7  |
             (send_sbus_.ch[12] & 0x07FF) << 4);
  send_buf_[18] = static_cast<uint8_t>((send_sbus_.ch[12] & 0x07FF) >> 4  |
             (send_sbus_.ch[13] & 0x07FF) << 7);
  send_buf_[19] = static_cast<uint8_t>((send_sbus_.ch[13] & 0x07FF) >> 1);
  send_buf_[20] = static_cast<uint8_t>((send_sbus_.ch[13] & 0x07FF) >> 9  |
             (send_sbus_.ch[14] & 0x07FF) << 2);
  send_buf_[21] = static_cast<uint8_t>((send_sbus_.ch[14] & 0x07FF) >> 6  |
             (send_sbus_.ch[15] & 0x07FF) << 5);
  send_buf_[22] = static_cast<uint8_t>((send_sbus_.ch[15] & 0x07FF) >> 3);
  send_buf_[23] = 0x00 | (send_sbus_.ch17 * CH17_MASK_) | (send_sbus_.ch18 * CH18_MASK_) |
             (send_sbus_.failsafe * FAILSAFE_MASK_) |
             (send_sbus_.lost_frame * LOST_FRAME_MASK_);
  send_buf_[24] = FOOTER_;
}

uint16_t rccontrol_to_sbus(float rccontrol)
{
    if (rccontrol < 0)
      rccontrol = 0;
    if (rccontrol > 100)
      rccontrol = 100;
    return static_cast<uint16_t>(SBUS_RANGE_MIN + (rccontrol * SBUS_RANGE_RANGE / 100.0f));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	SBUSSender::getInstance(huart)->assemble_packet();
	HAL_UART_Transmit_DMA (huart, SBUSSender::getInstance(huart)->send_buf_, SBUS_FRAME_SIZE);
}
