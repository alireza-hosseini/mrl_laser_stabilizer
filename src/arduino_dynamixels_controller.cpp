/*
 * Copyright (c) 2015, Alireza Hosseini.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Alireza Hosseini */

#include "mrl_laser_stabilizer/arduino_dynamixels_controller.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <mrl_common_protocol/stabilizer.hpp>
#include "mrl_laser_stabilizer/socket.hpp"
#include <std_msgs/Float64.h>

ArduinoDynamixelsController::ArduinoDynamixelsController(bool udp_based = false, QHostAddress robot_ip = QHostAddress(), int robot_port = 0):
    host_address_(robot_ip),port_(robot_port)
{
    if(udp_based)
    {
        socket_ = new Socket(robot_ip,robot_port);
        socket_->bind(socket_->getPort());
    }
    else
    {
        ros::NodeHandle nh_;
        roll_publisher_ = nh_.advertise<std_msgs::Float64>("/dynamixel_controller/roll_controller/command",10);
        pitch_publisher_ = nh_.advertise<std_msgs::Float64>("/dynamixel_controller/pitch_controller/command",10);
    }
}

//--------------------------------------------
void ArduinoDynamixelsController::run(bool send_over_udp)
{
    if(send_over_udp)
    {
        CommonProtocol::Stabilizer::StabilizerCommand dxl_cmd;
        dxl_cmd.roll=150 - (alignPitch()*-1);
        dxl_cmd.pitch=(150 - (alignRoll()));
        socket_->send(dxl_cmd.serialize());
    }
    else
    {
        std_msgs::Float64 roll;
        std_msgs::Float64 pitch;

        pitch.data = -1*(((double)alignPitch()/180)*M_PI);
        roll.data = -1*(((double)alignRoll()/180)*M_PI);

        ROS_DEBUG_STREAM("Pitch: "<<pitch.data<<" Roll: "<<roll.data);

        roll_publisher_.publish(roll);
        pitch_publisher_.publish(pitch);
    }
}

//--------------------------------------------
ArduinoDynamixelsController::~ArduinoDynamixelsController()
{
}

