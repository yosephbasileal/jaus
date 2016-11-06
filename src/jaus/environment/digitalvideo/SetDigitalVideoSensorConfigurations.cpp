////////////////////////////////////////////////////////////////////////////////////
///
///  \file SetDigitalVideoSensorConfigurations.cpp
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
#include "jaus/environment/digitalvideo/SetDigitalVideoSensorConfigurations.h"

using namespace JAUS;

SetDigitalVideoSensorConfigurations::SetDigitalVideoSensorConfigurations(const Address& dest, const Address& src) : 
Message(SET_DIGITAL_VIDEO_SENSOR_CONFIGURATION, dest, src)
{
    mSensorList = List();
}

SetDigitalVideoSensorConfigurations::SetDigitalVideoSensorConfigurations(const SetDigitalVideoSensorConfigurations& message) : 
Message(SET_DIGITAL_VIDEO_SENSOR_CONFIGURATION)
{
    *this = message;
}

SetDigitalVideoSensorConfigurations::~SetDigitalVideoSensorConfigurations()
{
    mSensorList.clear();
}

int SetDigitalVideoSensorConfigurations::WriteMessageBody(Packet& packet) const
{
    UInt startPos = packet.GetWritePos();
    Byte requestId = mRequestId;
    UShort count = (UShort)mSensorList.size();
    List::const_iterator digitalVideoSensorConfiguration = mSensorList.begin();
    packet.Write(requestId);
    packet.Write(count);
    while(digitalVideoSensorConfiguration != mSensorList.end())
    {
        packet.Write(digitalVideoSensorConfiguration->GetPresenceVector());      //required
        packet.Write(digitalVideoSensorConfiguration->GetSensorId());            //required
        if((digitalVideoSensorConfiguration->GetPresenceVector() & DigitalVideoSensorConfiguration::PresenceVector::MinimumBitRate) > 0)
        {
            packet.Write(digitalVideoSensorConfiguration->GetMinimumBitRate());
        }
        if((digitalVideoSensorConfiguration->GetPresenceVector() & DigitalVideoSensorConfiguration::PresenceVector::MaximumBitRate) > 0)
        {
            packet.Write(digitalVideoSensorConfiguration->GetMaximumBitRate());
        }
        if((digitalVideoSensorConfiguration->GetPresenceVector() & DigitalVideoSensorConfiguration::PresenceVector::FrameRate) > 0)
        {
            packet.Write(digitalVideoSensorConfiguration->GetFrameRate());
        }
        if((digitalVideoSensorConfiguration->GetPresenceVector() & DigitalVideoSensorConfiguration::PresenceVector::FrameSize) > 0)
        {
            packet.Write(digitalVideoSensorConfiguration->GetFrameSize());
        }
        if((digitalVideoSensorConfiguration->GetPresenceVector() & DigitalVideoSensorConfiguration::PresenceVector::DigitalFormat) > 0)
        {
            packet.Write(digitalVideoSensorConfiguration->GetDigitalFormat());
        }
        digitalVideoSensorConfiguration++;
    }

    return packet.GetWritePos() - startPos;
}

int SetDigitalVideoSensorConfigurations::ReadMessageBody(const Packet& packet)
{
    UInt startPos = packet.GetReadPos();
    Byte requestId = 0;
    UShort count = 0;
    packet.Read(requestId);
    packet.Read(count);
    mRequestId = requestId;
    for(int i = 0; i < (int)count; i++)
    {
        Byte presenceVector = 0;
        UShort sensorID = 0;
        packet.Read(presenceVector);
        packet.Read(sensorID);
        DigitalVideoSensorConfiguration dvsc = DigitalVideoSensorConfiguration(presenceVector, sensorID);
        if((presenceVector & DigitalVideoSensorConfiguration::PresenceVector::MinimumBitRate) > 0)
        {
            UShort minimumBitRate = 0;
            packet.Read(minimumBitRate);
            dvsc.SetMinimumBitRate(minimumBitRate);
        }
        if((presenceVector & DigitalVideoSensorConfiguration::PresenceVector::MaximumBitRate) > 0)
        {
            UShort maximumBitRate = 0;
            packet.Read(maximumBitRate);
            dvsc.SetMaximumBitRate(maximumBitRate);
        }
        if((presenceVector & DigitalVideoSensorConfiguration::PresenceVector::FrameRate) > 0)
        {
            Byte frameRate = 0;
            packet.Read(frameRate);
            dvsc.SetFrameRate(frameRate);
        }
        if((presenceVector & DigitalVideoSensorConfiguration::PresenceVector::FrameSize) > 0)
        {
            Byte frameSize = 0;
            packet.Read(frameSize);
            dvsc.SetFrameSize(frameSize);
        }
        if((presenceVector & DigitalVideoSensorConfiguration::PresenceVector::DigitalFormat) > 0)
        {
            Byte digitalFormat = 0;
            packet.Read(digitalFormat);
            dvsc.SetDigitalFormat(digitalFormat);
        }

        mSensorList.push_back(dvsc);
    }
    return packet.GetReadPos() - startPos;
}

bool SetDigitalVideoSensorConfigurations::IsLargeDataSet(const UInt maxPayloadSize) const
{
    //Quick Check to possibly avoid the time consuming loop (assume worst case of all fields supported)
    if(maxPayloadSize > BYTE_SIZE + ((BYTE_SIZE * 4 + USHORT_SIZE * 3) * mSensorList.size()))
    {
        return false;
    }

    UInt currentSize = 0;
    List::const_iterator iter = mSensorList.begin();
    while(iter != mSensorList.end())
    {
        currentSize += iter->GetSize();        
        iter++;
    }
    
    return currentSize > maxPayloadSize;
}

void SetDigitalVideoSensorConfigurations::PrintMessageBody() const
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
