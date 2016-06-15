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
** Ifdefs
*****************************************************************************/

#ifndef UDP_STABILIZERCONTROLLER_HPP_
#define UDP_STABILIZERCONTROLLER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include "mrl_laser_stabilizer/arduino_dynamixels_controller.hpp"
#include <sensor_msgs/Imu.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace mrl_laser_stabilizer {

/*****************************************************************************
** Class
*****************************************************************************/

class ArduinoStabilizerController : public QObject {
    Q_OBJECT
public:
    ArduinoStabilizerController(int argc, char** argv );
    virtual ~ArduinoStabilizerController();
	bool init();
    void run();
    void arduinoCallback(const sensor_msgs::Imu::ConstPtr& msg);

signals:
    void rosShutdown();

public slots:
    void onArduinoDataReceived();

private:
	int init_argc;
	char** init_argv;
    ArduinoDynamixelsController *dynamixelsController;

    std::string arduino_ip_;
    int arduino_port_;

    std::string robot_ip_;
    int robot_port_;

    ros::NodeHandle *n;
    Socket *arduino_socket_;

    bool udp_controller_enabled_;
    ros::Publisher imu_publisher_;

};

}

#endif /* udp_StabilizerController_HPP_ */
