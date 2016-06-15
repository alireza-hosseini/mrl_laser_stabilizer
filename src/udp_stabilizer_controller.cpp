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
#include "mrl_laser_stabilizer/udp_stabilizer_controller.hpp"

namespace mrl_laser_stabilizer {

UdpStabilizerController::UdpStabilizerController(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

UdpStabilizerController::~UdpStabilizerController() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
}

bool UdpStabilizerController::init() {
    ros::init(init_argc,init_argv,"viana_laser_stabilizer");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    n = new ros::NodeHandle("~");
    n->param<std::string>("device_ip",device_ip,"192.168.10.227");
    n->param<int>("device_port",device_port,3028);

    imu_sub = n->subscribe<sensor_msgs::Imu>("/imu/data", 10,
                                             ( boost::function < void(const sensor_msgs::Imu::ConstPtr&)>)
                                             boost::bind(&UdpStabilizerController::xsensCallback,this,_1));
    dynamixelsController = new UdpDynamixelsController(QHostAddress(QString::fromStdString(device_ip)),device_port);
    run();
    return true;
}


void UdpStabilizerController::run() {
    ros::Rate loop_rate(10);
    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close" << std::endl;
    emit rosShutdown();
}

void UdpStabilizerController::xsensCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    double yaw = atan2(2*(msg->orientation.x*msg->orientation.y + msg->orientation.z*msg->orientation.w),1-2*(msg->orientation.y*msg->orientation.y+msg->orientation.z*msg->orientation.z)) * 180/M_PI;
    double pitch = asin(2*(msg->orientation.x* msg->orientation.z - msg->orientation.y*msg->orientation.w)) * 180/M_PI;
    double roll = atan2(2*(msg->orientation.x*msg->orientation.w+msg->orientation.y*msg->orientation.z),1-2*(msg->orientation.z*msg->orientation.z+msg->orientation.w*msg->orientation.w)) * 180/M_PI;

    roll += 180;
    if(roll>180)
        roll=360-roll;
    else
        roll*=-1;
    //       roll*=-1;
    pitch*=-1;
    //        ROS_INFO("Received RPY: (%f,%f,%f)",roll,pitch,yaw);

    if((fabs(roll)<55)&&(fabs(pitch)<35))
    {
        dynamixelsController->setRPY(roll,pitch,yaw);
        dynamixelsController->run();
    }
}

}  // namespace viana_laser_stabilizer


int main(int argc, char **argv) {
    mrl_laser_stabilizer::UdpStabilizerController *UdpStabilizerController;
    UdpStabilizerController = new mrl_laser_stabilizer::UdpStabilizerController(argc,argv);
    UdpStabilizerController->init();

    return 0;
}
