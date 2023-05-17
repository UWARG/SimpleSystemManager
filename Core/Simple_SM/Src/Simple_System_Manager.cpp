/*
 * Simple_System_Manager.cpp
 *
 *  Created on: May 15, 2023
 *      Author: yudon
 */
#include "../Inc/Simple_Sytem_Manager.hpp"

SSM& SSM::getInstance(){
    static SSM singleton;
    return singleton;
}

void SSM::execute_manual_mode()
{
    config();

}

void SSM::fetch_command()
{

}

bool SSM::transmit_command()
{

}


void SSM::config(){

}

