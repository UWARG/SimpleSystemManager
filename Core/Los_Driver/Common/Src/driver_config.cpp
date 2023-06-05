
#include "../Inc/driver_config.hpp"

/*
    creating mavlink instance
*/

MAVLink pixhawk_mavlink_instance(pixhawk_mavlink_uart);
MAVLink* pixhawk_mavlink = &pixhawk_mavlink_instance;

MAVLink ground_mavlink_instance(ground_mavlink_uart);
MAVLink* ground_mavlink = &ground_mavlink_instance;