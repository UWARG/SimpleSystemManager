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

/*---------------------- Receiving ----------------------*/

bool MAVLink::readMessage()
{
    //if Data Reception process is not ongoing, then activate it
    if(uart_->RxState != HAL_UART_STATE_BUSY_RX){
        HAL_UARTEx_ReceiveToIdle_DMA(uart_, raw_rx_msg_, MAVLINK_MAX_PACKET_LEN);
    }

    uint8_t byte = 0;
	mavlink_status_t status = {};
    mavlink_message_t rx_msg;

    while(rx_circular_buffer_->read(&byte, 1)){
        if( mavlink_parse_char(MAVLINK_COMM_1, byte, &(rx_msg), &status)){
            mavlink_reset_channel_status(MAVLINK_COMM_1);
            rx_msg_ = rx_msg;
            return true;
        }
    }
    return false;
}


bool MAVLink::receiveMessage(MAVLinkMessage_t& mavlink_message){
	bool success = false;

	if (rx_msg_.sysid == plane_system_id &&
        rx_msg_.compid == plane_component_id) {
		success = true;

		switch (rx_msg_.msgid) {
			case MAVLINK_MSG_ID_HEARTBEAT:
            {
                mavlink_message.type = MAVLinkMessageType::HEARTBEAT;
                mavlink_msg_heartbeat_decode(&rx_msg_, &(mavlink_message.heartbeat));

                break;
            }
            case MAVLINK_MSG_ID_PARAM_VALUE:
            {
                mavlink_message.type = MAVLinkMessageType::ACK;

                mavlink_param_value_t param = {};
                mavlink_msg_param_value_decode(&rx_msg_, &param);
                mavlink_message.ack.type = MAVLinkACKType::PARAM;
                memcpy(mavlink_message.ack.param.param_id, param.param_id, 16);
                mavlink_message.ack.param.param_type = param.param_type;
                mavlink_message.ack.param.param_value = param.param_value;

                break;
            }
			case MAVLINK_MSG_ID_COMMAND_ACK:
            {
                mavlink_message.type = MAVLinkMessageType::ACK;

                mavlink_command_ack_t command_ack = {};
				mavlink_msg_command_ack_decode(&rx_msg_, &command_ack);
                mavlink_message.ack.type = MAVLinkACKType::COMMAND;
                mavlink_message.ack.command = command_ack.command;
                mavlink_message.ack.ack_result = command_ack.result;

                break;
            }
            case MAVLINK_MSG_ID_MISSION_ACK:
            {
                mavlink_message.type = MAVLinkMessageType::ACK;

                mavlink_mission_ack_t mission_ack = {};
                mavlink_msg_mission_ack_decode(&rx_msg_, &mission_ack);
                mavlink_message.ack.type = MAVLinkACKType::MISSION;
                mavlink_message.ack.mission = mission_ack.mission_type;
                /* Do not be alarmed by `type`. It's actually the result according to docs. */
                mavlink_message.ack.ack_result = mission_ack.type;

                break;
            }
			default:
				success = false;
                break;
		}
	}

	return success;
}

bool MAVLink::checkMessageACK(const MAVLinkMessage_t mavlink_message, const MAVLinkACK_t expected_ack)
{
    if (mavlink_message.type != MAVLinkMessageType::ACK) {
        /* Not an ACK message! */
        return false;
    }

    bool valid = true;
    const MAVLinkACK_t ack = mavlink_message.ack;

    switch (ack.type) {
        case MAVLinkACKType::PARAM:
            valid = compareParamId(ack.param.param_id, expected_ack.param.param_id) &&
                    (ack.param.param_type == expected_ack.param.param_type) &&
                    (ack.param.param_value - expected_ack.param.param_value < 0.01f);
            break;
        case MAVLinkACKType::COMMAND:
            valid = (ack.ack_result == MAV_RESULT_ACCEPTED) &&
                    (ack.command == expected_ack.command);
            break;
        case MAVLinkACKType::MISSION:
            valid = (ack.ack_result == MAV_MISSION_ACCEPTED) &&
                    (ack.mission == expected_ack.mission);
            break;
    }

    return valid;
}

/* -------------------- sending message -------------------*/
void MAVLink::sendHeartbeat()
{
	mavlink_heartbeat_t heartbeat = {};
	heartbeat.type = MAV_TYPE_GCS; /* We are representing ground (control) station. */
	heartbeat.autopilot = MAV_AUTOPILOT_INVALID;

	mavlink_message_t message = {};
	mavlink_msg_heartbeat_encode(system_id, component_id, &message, &heartbeat);

	writeMessage(message);
}

MAVLinkACK_t MAVLink::sendInitialConfigs()
{
    /* Tell the plane to VTOL hover over a waypoint when it's reached in guided mode. */
    mavlink_param_set_t vtol_hover_param = {};
    vtol_hover_param.target_system = plane_system_id;
    vtol_hover_param.target_system = plane_component_id;
    strcpy(vtol_hover_param.param_id, "Q_GUIDED_MODE");
    vtol_hover_param.param_value = 1;
    vtol_hover_param.param_type = MAV_PARAM_TYPE_UINT8;

    mavlink_message_t message = {};
    mavlink_msg_param_set_encode(system_id, component_id, &message, &vtol_hover_param);

    MAVLinkACK_t expected_ack = {};
    expected_ack.type = MAVLinkACKType::PARAM;
    strcpy(expected_ack.param.param_id, "Q_GUIDED_MODE");
    expected_ack.param.param_value = 1;
    expected_ack.param.param_type = MAV_PARAM_TYPE_UINT8;

    writeMessage(message);

    return expected_ack;
}

MAVLinkACK_t MAVLink::sendArmDisarm(const bool arm)
{
    mavlink_command_long_t command_long = {};
    command_long.command = MAV_CMD_COMPONENT_ARM_DISARM;
    command_long.param1 = (float)arm;

    MAVLinkACK_t expected_ack = {};
    expected_ack.type = MAVLinkACKType::COMMAND;
    expected_ack.command = MAV_CMD_COMPONENT_ARM_DISARM;

    sendCommandLong(command_long);

    return expected_ack;
}

MAVLinkACK_t MAVLink::sendFlightModeChange(const PLANE_MODE flight_mode)
{
    mavlink_command_long_t command_long = {};
    command_long.command = MAV_CMD_DO_SET_MODE;
    command_long.param2 = (float)flight_mode;

    MAVLinkACK_t expected_ack = {};
    expected_ack.type = MAVLinkACKType::COMMAND;
    expected_ack.command = MAV_CMD_DO_SET_MODE;

    sendCommandLong(command_long);

    return expected_ack;
}

MAVLinkACK_t MAVLink::sendVTOLTakeOff(const float altitude)
{
    mavlink_command_long_t command_long = {};
    command_long.command = MAV_CMD_NAV_VTOL_TAKEOFF;
    command_long.param7 = altitude;

    MAVLinkACK_t expected_ack = {};
    expected_ack.type = MAVLinkACKType::COMMAND;
    expected_ack.command = MAV_CMD_NAV_VTOL_TAKEOFF;

    sendCommandLong(command_long);

    return expected_ack;
}

MAVLinkACK_t MAVLink::sendWaypointNav(const float x, const float y, const float z, const float acceptable_range)
{
	mavlink_command_long_t command_long = {};
	command_long.command = MAV_CMD_NAV_WAYPOINT;
	command_long.confirmation = 1;
	command_long.param2 = acceptable_range;
	command_long.param5 = x;
	command_long.param6 = y;
	command_long.param7 = z;

    MAVLinkACK_t expected_ack = {};
    expected_ack.type = MAVLinkACKType::COMMAND;
    expected_ack.command = MAV_CMD_NAV_WAYPOINT;

	sendCommandLong(command_long);

    return expected_ack;
}

MAVLinkACK_t MAVLink::sendClearMissions()
{
    mavlink_mission_clear_all_t clear_all = {};
    clear_all.target_system = plane_system_id;
    clear_all.target_component = plane_component_id;
    clear_all.mission_type = MAV_MISSION_TYPE_ALL;

    mavlink_message_t message = {};
    mavlink_msg_mission_clear_all_encode(system_id, component_id, &message, &clear_all);

    MAVLinkACK_t expected_ack = {};
    expected_ack.type = MAVLinkACKType::MISSION;
    expected_ack.mission = MAV_MISSION_TYPE_ALL;

    writeMessage(message);

    return expected_ack;
}

/* ---------------- Helper Methods ---------------------------*/
void MAVLink::writeMessage(const mavlink_message_t &msg){
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	HAL_StatusTypeDef ret = HAL_UART_Transmit(uart_, buf, len, 1000);
}

void MAVLink::sendCommandLong(mavlink_command_long_t command_long)
{
    command_long.target_system = plane_system_id;
	command_long.target_component = plane_component_id;

    mavlink_message_t message = {};
	mavlink_msg_command_long_encode(system_id, component_id, &message, &command_long);

	writeMessage(message);
}

bool MAVLink::compareParamId(const char id1[16], const char id2[16])
{
    /* Accommodate edge case where the type is exactly 16 characters and
     * doesn't have a terminating character.
     */
    char id1_str[17] = {0};
    char id2_str[17] = {0};
    memcpy(id1_str, id2, 16);
    memcpy(id2_str, id2, 16);
    return (strcmp(id1_str, id2_str) == 0);
}




