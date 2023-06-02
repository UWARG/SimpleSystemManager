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

enum class MAVLinkACKType {
    PARAM,
    COMMAND,
    MISSION
};

enum class MAVLinkMessageType {
    ACK,
    HEARTBEAT
};

typedef struct MAVLinkACK {
    MAVLinkACKType type;
    mavlink_param_value_t param;
    uint8_t ack_result; /* Only applies to command and mission ACKs. */
    uint16_t command;
    uint16_t mission;
} MAVLinkACK_t;

typedef struct MAVLinkMessage {
	MAVLinkMessageType type;
	mavlink_heartbeat_t heartbeat;
	MAVLinkACK_t ack;
} MAVLinkMessage_t;

class MAVLink {
	public:

        uint8_t rx_circular_buffer_ptr_[1000];
        CircularBuffer* rx_circular_buffer_;
        uint8_t raw_rx_msg_[MAVLINK_MAX_PACKET_LEN];
        mavlink_message_t rx_msg_;

        
        /* Constructor */
		MAVLink(UART_HandleTypeDef* uart_handle);
        ~MAVLink();

        /* ------------------ Commands Available to Send ------------------*/

        /* Send a heartbeat message to sync with ArduPilot. This needs to be done
         * at ideally 1Hz and no less than 0.2Hz.
         */
		void sendHeartbeat();

        /* Set initial config parameters. This needs to be done before flight. */
        MAVLinkACK_t sendInitialConfigs();

        /* Arm (arm=1) or disarm (arm=0) the plane. */
        MAVLinkACK_t sendArmDisarm(const bool arm);

        /* Change the flight mode of the plane.
         *
         * Plane flight modes we should use:
         * - PLANE_MODE_AUTO
         * - PLANE_MODE_GUIDED
         * - PLANE_MODE_QLOITER
         * - PLANE_MODE_QLAND
         * - PLANE_MODE_MANUAL
         */
        MAVLinkACK_t sendFlightModeChange(const PLANE_MODE flight_mode);

        /* Do a VTOL take-off. The plane must be in AUTO mode. */
        MAVLinkACK_t sendVTOLTakeOff(const float altitude);

        /* Add a waypoint to navigate. The plane will consider the waypoint reached when it's
         * within acceptable_range of the waypoint. The plane must be in GUIDED mode. */
		MAVLinkACK_t sendWaypointNav(const float x, const float y, const float z, const float acceptable_range);

        /* Clear all missions, including VTOL take-off and waypoint navigation. */
        MAVLinkACK_t sendClearMissions();

        /* ---------------------- Receiving Methods -------------------*/

        /*
            Trigger after receiving a message
            Figuare the message type in this function and further calling correct consequent function
        */
		bool receiveMessage(MAVLinkMessage_t& mavlink_message);

        /* Check an ACK message against an expected ACK results. */
        bool checkMessageACK(const MAVLinkMessage_t mavlink_message, const MAVLinkACK_t expected_ack);

        /* 
            a helper function on receiveing and parsing data
            need to be constantly called in a loop to constantly parsing the incoming message
        */
        bool readMessage();

	private:
        UART_HandleTypeDef* uart_;
		/* We are representing ground station, so our system ID and
		 * component ID are 255 and 1. */
		const uint8_t system_id = 255;
		const uint8_t component_id = 1;
		/* Plane system ID and component ID */
		const uint8_t plane_system_id = 1;
		const uint8_t plane_component_id = 1;

		mavlink_status_t last_status;


        /* ------------------- Helper Methods --------------*/

		void writeMessage(const mavlink_message_t &msg);
        void sendCommandLong(mavlink_command_long_t command_long);
        bool compareParamId(const char id1[16], const char id2[16]);
};

/* Some Functions that can be further worked on

    1. Recognizing the existance of other devices, through heartbeat
    2. Automatically handles the ack message, that is, after sending out a message, it wait for ack
    3. Make the driver able to handle more types of message, not only ack and heartbeat

*/


#endif /* LOS_DRIVER_SSM_MAVLINK_DRIVER_INC_SSM_D_MAVLINK_HPP_ */
