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

#ifndef ABSTRACT_DYNAMIXELS_CONTROLLER_HPP
#define ABSTRACT_DYNAMIXELS_CONTROLLER_HPP

#include <QObject>
#include <QThread>
#include <iostream>
#include <math.h>
#include <QDataStream>

using namespace std;


typedef struct _DynamixelsCommand
{
    short int pitch;
    short int roll;
    short int  chksum;

    _DynamixelsCommand()
    {
        pitch = 0;
        roll = 0;
        chksum = 0;
    }
    QByteArray serialize()
    {        
        QByteArray bytearray;
        QDataStream stream(&bytearray, QIODevice::ReadWrite);
        void* buf = this;
        //stream << pitch << roll << chksum;
        stream.writeRawData((char*)buf,sizeof(_DynamixelsCommand));
        return bytearray;
    }
}DynamixelsCommand;

class AbstractDynamixelsController : public QObject
{
public:
    AbstractDynamixelsController();
    ~AbstractDynamixelsController();

    virtual void setRPY(float roll, float pitch, float yaw);


    float roll() const;
    void setRoll(float roll);

    float pitch() const;
    void setPitch(float pitch);

    float yaw() const;
    void setYaw(float yaw);

protected:
    short int  alignPitch( );
    short int  alignRoll( );

private:

    float roll_;
    float pitch_;
    float yaw_;

    float RPe1,RPe2,RPe;
    float Pe1,Pe2,Pe;
    float TotalR;
    float TotalP;

};

#endif //DYNAMIXELS_CONTROLLER_HPP
