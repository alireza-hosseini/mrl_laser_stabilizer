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

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "mrl_laser_stabilizer/usb_stabilizer_controller.hpp"


namespace mrl_laser_stabilizer {

UsbStabilizerController::UsbStabilizerController(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

UsbStabilizerController::~UsbStabilizerController() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
}

bool UsbStabilizerController::init() {
    ros::init(init_argc,init_argv,"viana_laser_stabilizer");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    n = new ros::NodeHandle("~");

    imu_sub = n->subscribe<sensor_msgs::Imu>("/imu/data", 10,
                                             ( boost::function < void(const sensor_msgs::Imu::ConstPtr&)>)
                                             boost::bind(&UsbStabilizerController::xsensCallback,this,_1));

    dynamixelsController = new UsbDynamixelsController();
    run();
    return true;
}


void UsbStabilizerController::run() {
    ros::Rate loop_rate(100);
    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close" << std::endl;
    emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void UsbStabilizerController::xsensCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    double yaw = atan2(2*(msg->orientation.x*msg->orientation.y + msg->orientation.z*msg->orientation.w),1-2*(msg->orientation.y*msg->orientation.y+msg->orientation.z*msg->orientation.z)) * 180/M_PI;
    double pitch = asin(2*(msg->orientation.x* msg->orientation.z - msg->orientation.y*msg->orientation.w)) * 180/M_PI;
    double roll = atan2(2*(msg->orientation.x*msg->orientation.w+msg->orientation.y*msg->orientation.z),1-2*(msg->orientation.z*msg->orientation.z+msg->orientation.w*msg->orientation.w)) * 180/M_PI;

    roll += 180;
    if(roll>180)
        roll=360-roll;
    else
        roll*=-1;
    roll+=5;
    pitch-=10;

    if((fabs(roll)<65)&&(fabs(pitch)<65))
    {
        dynamixelsController->setRPY(roll,pitch,yaw);
        dynamixelsController->run();
    }
}

}

int main(int argc, char **argv) {
    mrl_laser_stabilizer::UsbStabilizerController *UsbStabilizerController;
    UsbStabilizerController = new mrl_laser_stabilizer::UsbStabilizerController(argc,argv);
    UsbStabilizerController->init();

    return 0;
}
