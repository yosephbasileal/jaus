////////////////////////////////////////////////////////////////////////////////////
///
///  \file SetAnalogVideoSensorConfigurations.cpp
///  \brief This file contains the implementation of a JAUS message.
///
///  <br>Author(s): Daniel Barber, Jonathan Harris
///  <br>Created: 22 March 2013
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
#include "jaus/environment/analogvideo/SetAnalogVideoSensorConfigurations.h"

using namespace JAUS;

SetAnalogVideoSensorConfigurations::SetAnalogVideoSensorConfigurations(const Address& dest, const Address& src) : 
Message(SET_ANALOG_VIDEO_SENSOR_CONFIGURATION, dest, src)
{
    mSensorList = List();
}

SetAnalogVideoSensorConfigurations::SetAnalogVideoSensorConfigurations(const SetAnalogVideoSensorConfigurations& message) : 
Message(SET_ANALOG_VIDEO_SENSOR_CONFIGURATION)
{
    *this = message;
}

SetAnalogVideoSensorConfigurations::~SetAnalogVideoSensorConfigurations()
{
    mSensorList.clear();
}

int SetAnalogVideoSensorConfigurations::WriteMessageBody(Packet& packet) const
{
    UInt startPos = packet.GetWritePos();
    Byte requestId = mRequestId;
    UShort count = (UShort)mSensorList.size();
    List::const_iterator sensor = mSensorList.begin();
    packet.Write(requestId);
    packet.Write(count);
    while(sensor != mSensorList.end())
    {
        sensor->WriteMessageBody(packet);
        sensor++;
    }
    return packet.GetWritePos() - startPos;
}

int SetAnalogVideoSensorConfigurations::ReadMessageBody(const Packet& packet)
{
    UInt startPos = packet.GetReadPos();
    Byte requestId = 0;
    UShort count = 0;
    packet.Read(requestId);
    packet.Read(count);
    mRequestId = requestId;
    for(int i = 0; i < (int)count; i++)
    {
        UShort sensorID = 0;
        Byte analogFormat = 0;
        packet.Read(sensorID);
        packet.Read(analogFormat);
        AnalogVideoSensorConfiguration avsc = AnalogVideoSensorConfiguration(sensorID, analogFormat);
        mSensorList.push_back(avsc);
    }
    return packet.GetReadPos() - startPos;
}

bool SetAnalogVideoSensorConfigurations::IsLargeDataSet(const UInt maxPayloadSize) const
{
    if(maxPayloadSize > (BYTE_SIZE + ((BYTE_SIZE + USHORT_SIZE) * mSensorList.size())))
    {
        return false;
    }

    return true;
}

void SetAnalogVideoSensorConfigurations::PrintMessageBody() const
{
    std::cout << "Sensor Count: " << mSensorList.size() << std::endl;
    UShort count = (UShort)mSensorList.size();
    List::const_iterator sensor = mSensorList.begin();
    while(sensor != mSensorList.end())
    {
        std::cout << "<Sensor>" <<  std::endl;
        sensor->PrintSensorFields();
        std::cout << "</Sensor>" <<  std::endl;
        sensor++;
    }
}

/*  End of File */
