#include "../Inc/LOS_D_SBUSSender.hpp"

SBUSSender::SBUSSender(UART_HandleTypeDef* uart) : uart_(uart){

}

void SBUSSender::SetChannelValue(uint8_t channel, uint16_t value){

}

void SBUSSender::SetSBusValue(SBus values){
    send_sbus_ = values;
}

void SBUSSender::SendData(){
    assemble_packet();
    HAL_StatusTypeDef ret = HAL_UART_Transmit(uart_, send_buf_, SBUS_FRAME_SIZE, 100);
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
