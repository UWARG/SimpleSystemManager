#ifndef DRIVER_CONFIG_HPP_
#define DRIVER_CONFIG_HPP_

#include <cstdint>
#include "main.h"
#include "usart.h"

/*
    UART Mapping
*/

#define sbus_uart       &huart2
#define pixhawk_mavlink_uart    &huart3

/*
    MAVlink Instances
*/
#include "../../SSM_MAVLink_Driver/Inc/SSM_D_MAVLink.hpp"

extern MAVLink* pixhawk_mavlink;


#endif
