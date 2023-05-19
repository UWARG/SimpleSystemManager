/*
 * Simple_System_Manager.cpp
 *
 *  Created on: May 15, 2023
 *      Author: yudon
 */
#include "../Inc/Simple_Sytem_Manager.hpp"

SSM* SSM::singleton_= NULL;

SSM* SSM::getInstance(){
    if (singleton_ == NULL)
      singleton_ = new SSM();
       
    // returning the instance pointer
    return singleton_;
}


void SSM::execute_manual_mode()
{
    SBus sbus_data;
    fetch_command(sbus_data);
    transmit_command(sbus_data);
}

SSM::SSM(){
    config();
}

SSM::~SSM(){
    delete sbus_receive_handle_;
    delete sbus_sender_handle_;
}

void SSM::config(){
    // sbus_receive_handle_ = new SBUSReceiver(&huart2);
    // sbus_sender_handle_ = new SBUSSender(&huart2);
}

void SSM::fetch_command(SBus &sbus_data)
{
    RCControl command = sbus_receive_handle_->GetRCControl();
    sbus_data = sbus_receive_handle_->GetSBUS();
    /* map the signal to the corresponse flight mode*/
    if(command.mode >= 0 && command.mode < (100.0f/6.0f) )
    {
        flight_mode_ = static_cast<Flight_Mode>(1);
    } else if (command.mode >  (100.0f/6.0f) && command.mode < 2 * (100.0f/6.0f) )
    {
        flight_mode_ = static_cast<Flight_Mode>(2);
    }else if (command.mode >  2 * (100.0f/6.0f) && command.mode < 3 * (100.0f/6.0f) )
    {
        flight_mode_ = static_cast<Flight_Mode>(3);
    }else if (command.mode >  3 * (100.0f/6.0f) && command.mode < 4 * (100.0f/6.0f) )
    {
        flight_mode_ = static_cast<Flight_Mode>(4);
    }else if (command.mode >  4 * (100.0f/6.0f) && command.mode < 5 * (100.0f/6.0f) )
    {
        flight_mode_ = static_cast<Flight_Mode>(5);
    }else if (command.mode >  5 * (100.0f/6.0f) && command.mode <= 100 )
    {
        flight_mode_ = static_cast<Flight_Mode>(6);
    }

    if(flight_mode_ == stablize || flight_mode_ == alt_hold || flight_mode_ == loiter || flight_mode_ == acro)
        status_ = manual;
    if(flight_mode_ == RTL || flight_mode_ == autoo)
        status_ = autonomous;
}

bool SSM::transmit_command(SBus &sbus_data)
{
    sbus_sender_handle_->SetSBusValue(sbus_data);
    //sbus_sender_handle_->SendData();
}

