/*
 * Simple_Sytem_Manager.hpp
 *
 *  Created on: May 15, 2023
 *      Author: yudon
 */

#ifndef SIMPLE_SM_INC_SIMPLE_SYTEM_MANAGER_HPP_
#define SIMPLE_SM_INC_SIMPLE_SYTEM_MANAGER_HPP_

#include "main.h"
#include "../../Los_Driver/SBUS/Inc/LOS_D_SBUSReceiver.hpp"
#include "../../Los_Driver/SBUS/Inc/LOS_D_SBUSSender.hpp"
#include "../../Los_Driver/Common/Inc/driver_config.hpp"

enum Drone_State
{
    manual,         //high level manual and autonomous
    autonomous
};

enum Flight_Mode {
    /*  flight mode = flight mode number*/
    /*  this is okay to configure*/
    stablize = 1, 
    alt_hold,
    loiter,
    RTL,
    autoo,
    acro
};

enum Drone_Arm{
    armed,
    disarmed
};

class SSM
{
    public:
    /*
        telling compiler this class should be a singlton
        no copy of this instance should be created
    */
        SSM (const SSM*) = delete;
        SSM &operator= (const SSM&) = delete;

    /*
        get the instance of the singlton just in case we need it
    */
        static SSM* getInstance();

    /*
        the function can be called when the drone only wants to be
        controlled manually. no automonous mode or control is activiated.
        Purpose of this function is for first stage testing
        It should have a simple function call flow
    */
        void execute_manual_mode();

    /*
        this function should be called in loop by main()
        execute in the integration of all the function calls needed for
        a single execution of SM
    */
        void execute();

    private:
        Drone_State status_;
        Flight_Mode flight_mode_;
        Drone_Arm drone_arm_;
        static SSM* singleton_;

        SSM();
        ~SSM();

        void config();

    /*
        fetch ppm/sbus data from receiver
        this data is supposed to be the rc control input
    */
        void fetch_command(SBus &sbus_data);

    /*
        check the ppm signal quality by checking the rssi
        or by learning whether or not we received anything
    */
        bool check_connection(SBus &sbus_data);

    /*
        transmit the packaged ppm/sbus signal to pixhawk
    */
        bool transmit_command(SBus &sbus_data);

    /*
        set the flag and other parameters to the correct configuration
    */
        void set_manual();
        void set_auto();

    /*
        fetch teh auto command from the RFD 900
    */
        void fetch_auto_command();

    /*
        by decode the telementry data fetched
        determine the quality of the command
    */
        bool check_auto_connection();

    /*
        encode the telementry data into MAVLink format
        and transmit the data to pixhawk
    */
        void transmit_MAVLink();

    /*
        waiting for the response MAVlink from pixhawk
    */
        void receive_MAVLink();

    /*
        transmit the telementry data to RFD
    */
        void telem_to_ground();
};


#endif /* SIMPLE_SM_INC_SIMPLE_SYTEM_MANAGER_HPP_ */
