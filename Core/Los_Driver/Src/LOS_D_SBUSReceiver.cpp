#include "../Inc/LOS_D_SBUSReceiver.hpp"

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

}

SBus SBUSReceiver::GetResult(){
        read();
        return received_sbus_;
}

 void SBUSReceiver::read()
 {
    received_sbus_.new_data = false;
    //if(HAL_UART_Receive(uart_, raw_sbus_, SBUS_FRAME_SIZE, 100) == HAL_OK)
    if(parse() == true)
        received_sbus_.new_data = true;
}

bool SBUSReceiver::parse()
{

    for (uint8_t i = 0; i < 50; i++){
    	HAL_UART_Receive(uart_, &cur_byte_, 1, 10);

        if (state_ == 0) {
            if ((cur_byte_ == HEADER_) && ((prev_byte_ == FOOTER_) ||
                ((prev_byte_ & 0x0F) == FOOTER2_))) {
                raw_sbus_[state_++] = cur_byte_;
            } else {
                state_ = 0;
            }
        } else if (state_ < PAYLOAD_LEN_ + HEADER_LEN_) {
            raw_sbus_[state_++] = cur_byte_;
        } else if (state_ < PAYLOAD_LEN_ + HEADER_LEN_ + FOOTER_LEN_) {
            state_ = 0;
            prev_byte_ = cur_byte_;
            if ((cur_byte_ == FOOTER_) || ((cur_byte_ & 0x0F) == FOOTER2_)) {

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

                return true;
            } else {
                return false;
            }
        } else {
            state_ = 0;
        }
        prev_byte_ = cur_byte_;   
    }

    return false;
}
