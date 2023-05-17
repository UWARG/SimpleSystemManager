/*
 * Simple_Sytem_Manager.hpp
 *
 *  Created on: May 15, 2023
 *      Author: yudon
 */

#ifndef SIMPLE_SM_INC_SIMPLE_SYTEM_MANAGER_HPP_
#define SIMPLE_SM_INC_SIMPLE_SYTEM_MANAGER_HPP_

#include "../../Los_Driver/Inc/LOS_D_SBUSReceiver.hpp"

enum drone_state
{
    manual,         //high level manual and autonomous
    autonomous
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
        static SSM &getInstance();

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
        drone_state status_;

        SSM();
        ~SSM();

        void config();

    /*
        fetch ppm/sbus data from receiver
        this data is supposed to be the rc control input
    */
        void fetch_command();

    /*
        check the ppm signal quality by checking the rssi
        or by learning whether or not we received anything
    */
        bool check_connection();

    /*
        transmit the packaged ppm/sbus signal to pixhawk
    */
        bool transmit_command();

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
