////////////////////////////////////////////////////////////////////////////////////
///
///  \file ControlDigitalVideoSensor.cpp
///  \brief This file contains the implementation of a JAUS message.
///
///  <br>Author(s): Daniel Barber, Jonathan Harris
///  <br>Created: 21 March 2013
///  <br>Copyright (c) 2013
///  <br>Applied Cognition and Training in Immersive Virtual Environments
///  <br>(ACTIVE) Laboratory
///  <br>Institute for Simulation and Training (IST)
///  <br>University of Central Florida (UCF)
///  <br>All rights reserved.
///  <br>Email: dbarber@ist.ucf.edu, jharris@ist.ucf.edu
///  <br>Web:  http://active.ist.ucf.edu
///
///  Redistribution and use in source and binary forms, with or without
///  modification, are permitted provided that the following conditions are met:
///      * Redistributions of source code must retain the above copyright
///        notice, this list of conditions and the following disclaimer.
///      * Redistributions in binary form must reproduce the above copyright
///        notice, this list of conditions and the following disclaimer in the
///        documentation and/or other materials provided with the distribution.
///      * Neither the name of the ACTIVE LAB, IST, UCF, nor the
///        names of its contributors may be used to endorse or promote products
///        derived from this software without specific prior written permission.
/// 
///  THIS SOFTWARE IS PROVIDED BY THE ACTIVE LAB''AS IS'' AND ANY
///  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
///  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
///  DISCLAIMED. IN NO EVENT SHALL UCF BE LIABLE FOR ANY
///  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
///  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
///  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
///  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
///  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
///  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///
////////////////////////////////////////////////////////////////////////////////////
#include "jaus/environment/digitalvideo/ControlDigitalVideoSensor.h"

using namespace JAUS;

ControlDigitalVideoSensor::ControlDigitalVideoSensor(const Address& dest, const Address& src) : 
Message(CONTROL_DIGITAL_VIDEO_SENSOR_STREAM, dest, src) { }

ControlDigitalVideoSensor::ControlDigitalVideoSensor(const ControlDigitalVideoSensor& message) : 
Message(CONTROL_DIGITAL_VIDEO_SENSOR_STREAM)
{
    *this = message;
}

int ControlDigitalVideoSensor::WriteMessageBody(Packet& packet) const
{
    UInt startPos = packet.GetWritePos();
    packet.Write(mSensor.GetSensorId());        //required
    packet.Write(mSensor.GetStreamState());     //required
    return packet.GetWritePos() - startPos;
}

int ControlDigitalVideoSensor::ReadMessageBody(const Packet& packet)
{
    UInt startPos = packet.GetReadPos();
    UShort sensorID = 0;
    Byte streamState = 0;
    packet.Read(sensorID);
    packet.Read(streamState);
    DigitalVideoSensor dvs = DigitalVideoSensor(sensorID, streamState);
    mSensor = dvs;
    return packet.GetReadPos() - startPos;
}

bool ControlDigitalVideoSensor::IsLargeDataSet(const UInt maxPayloadSize) const
{
    UInt dataSetSize = USHORT_SIZE + BYTE_SIZE;
    return dataSetSize > maxPayloadSize;
}

void ControlDigitalVideoSensor::PrintMessageBody() const
{
    std::cout << "<Sensor>" <<  std::endl;
    mSensor.PrintSensorFields();
    std::cout << "</Sensor>" <<  std::endl;
}

/*  End of File */
