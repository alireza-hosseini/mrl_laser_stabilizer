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

#include "mrl_laser_stabilizer/abstract_dynamixels_controller.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

AbstractDynamixelsController::AbstractDynamixelsController():roll_(0),pitch_(0),RPe1(0),RPe2(0)
{
    RPe1 = 0;
    RPe2 = 0;
}
//--------------------------------------------
AbstractDynamixelsController::~AbstractDynamixelsController()
{
}
//--------------------------------------------
short int  AbstractDynamixelsController::alignPitch( )
{
    float Kp,Ki,Kd;
    short int cmd;
    float  pitch=0;

    for(int i=1;i<10;++i)
        pitch += pitch_;
    pitch/=10;
    pitch*=-1;

    if(abs(((int)pitch))< 5 )
    {
        Kp=1;
        //0.9999956;
        Ki=0;
        //0.9535;
        Kd=0;
        //0.0000002;
    }
    else
    {

        Kp=1;
        //0.9999956;
        Ki=0;
        //0.9535;
        Kd=0;
        //0.0000002;
    }

    Pe=pitch;
    TotalP=/*TotalP+*/(Kp*Pe)-(Ki*Pe1)+(Kd*Pe2);
    //   Pe2=Pe1;
    //   Pe1=Pe;

    // qDebug()<<"Pitch :"<< TotalP ;


    float num=TotalP;
    if (num<-120)//120
    {
        num=-120;
        Pe=0;
    }
    if(num>120)//120
    {
        num=120;
        Pe=0;
    }

    cmd=num;
    cmd=Pe;
    return cmd;
}
//--------------------------------------------
short int AbstractDynamixelsController::alignRoll()
{
    float Kp,Ki,Kd;
    float roll=0;
    short int cmd;

    for(int i=1;i<10;++i)
        roll += roll_;
    roll/=10;

    roll*=-1;

    if(abs(((int)roll))< 5 )
    {
        Kp=1;
        //0.9999956;
        Ki=0;
        //0.9535;
        Kd=0;
        //0.0000002;
    }
    else
    {
        Kp=1;
        //0.9999956;
        Ki=0;
        //0.9535;
        Kd=0;
        //0.0000002;
    }

    RPe=roll;
    TotalR=/*TotalR+*/(Kp*RPe)-(Ki*RPe1)+(Kd*RPe2);

    //     RPe2=RPe1;
    //     RPe1=RPe;

    float num=TotalR;
    if (num<-120)
    {
        num=-120;
        RPe=0;
    }
    if(num>120)
    {
        num=120;
        RPe=0;
    }
    cmd=num;

    return cmd;

}
void AbstractDynamixelsController::setRPY(float roll, float pitch, float yaw)
{
    roll_=roll; pitch_=pitch; yaw_=yaw;
}

float AbstractDynamixelsController::roll() const
{
    return roll_;
}

void AbstractDynamixelsController::setRoll(float roll)
{
    roll_ = roll;
}
float AbstractDynamixelsController::pitch() const
{
    return pitch_;
}

void AbstractDynamixelsController::setPitch(float pitch)
{
    pitch_ = pitch;
}
float AbstractDynamixelsController::yaw() const
{
    return yaw_;
}

void AbstractDynamixelsController::setYaw(float yaw)
{
    yaw_ = yaw;
}


