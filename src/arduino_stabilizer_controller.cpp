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


/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "mrl_laser_stabilizer/arduino_stabilizer_controller.hpp"
#include <tf/tf.h>
#include <angles/angles.h>

namespace mrl_laser_stabilizer {

ArduinoStabilizerController::ArduinoStabilizerController(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

ArduinoStabilizerController::~ArduinoStabilizerController() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
}

bool ArduinoStabilizerController::init() {
    ros::init(init_argc,init_argv,"arduino_laser_stabilizer");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    n = new ros::NodeHandle("~");
    n->param<std::string>("robot_ip",robot_ip_,"192.168.10.227");
    n->param<int>("robot_port",robot_port_,3028);

    n->param<std::string>("arduino_ip",arduino_ip_,"192.168.10.2");
    n->param<int>("arduino_port",arduino_port_,8888);
    n->param<bool>("udp_controller",udp_controller_enabled_,false);


    dynamixelsController = new ArduinoDynamixelsController(udp_controller_enabled_,QHostAddress(robot_ip_.c_str()),robot_port_);
    arduino_socket_ = new Socket(QHostAddress(arduino_ip_.c_str()),arduino_port_);
    arduino_socket_->bind(arduino_port_);

    imu_publisher_ = n->advertise<sensor_msgs::Imu>("/imu/data",10);
    run();
    return true;
}


void ArduinoStabilizerController::run() {
    ros::Rate loop_rate(10);
    while ( ros::ok() ) {
        onArduinoDataReceived();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void ArduinoStabilizerController::arduinoCallback(const sensor_msgs::Imu::ConstPtr& msg)
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
    //ROS_INFO("Received RPY: (%f,%f,%f)",roll,pitch,yaw);

    if((fabs(roll)<65)&&(fabs(pitch)<65))
    {
        dynamixelsController->setRPY(roll,pitch,yaw);
        dynamixelsController->run();
    }
}

void ArduinoStabilizerController::onArduinoDataReceived()
{
    qDebug()<<"waiting";
    while (arduino_socket_->hasPendingDatagrams()) {
        QByteArray byte_array;
        byte_array.resize(arduino_socket_->pendingDatagramSize());
        arduino_socket_->readDatagram(byte_array.data(),byte_array.size());
        QList<QByteArray> values = byte_array.split(',');
        if(values.size() == 3)
        {
            double yaw = QString(values.at(0)).toDouble();
            double pitch = QString(values.at(1)).toDouble();
            double roll = QString(values.at(2)).toDouble();
            qDebug()<<yaw<<" "<<pitch<<" "<<roll;

            dynamixelsController->setRPY(roll,pitch,yaw);
            dynamixelsController->run(udp_controller_enabled_);
            geometry_msgs::Quaternion orientation;
            orientation = tf::createQuaternionMsgFromRollPitchYaw(angles::normalize_angle(angles::from_degrees(roll)),angles::normalize_angle(angles::from_degrees(pitch*-1)),0);
            sensor_msgs::Imu imu;
            imu.orientation = orientation;
            imu.header.frame_id = "/imu";
            imu.header.stamp = ros::Time::now();
            imu_publisher_.publish(imu);
        }
    }
}

}  // namespace viana_laser_stabilizer


int main(int argc, char **argv) {
    mrl_laser_stabilizer::ArduinoStabilizerController *arduinoStabilizerController;
    arduinoStabilizerController = new mrl_laser_stabilizer::ArduinoStabilizerController(argc,argv);
    arduinoStabilizerController->init();

    return 0;
}
