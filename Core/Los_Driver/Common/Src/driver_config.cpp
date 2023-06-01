
#include "../Inc/driver_config.hpp"

/*
    creating mavlink instance
*/

MAVLink pixhawk_mavlink_instance(pixhawk_mavlink_uart);
MAVLink* pixhawk_mavlink = &pixhawk_mavlink_instance;