////////////////////////////////////////////////////////////////////////////////////
///
///  \file SetStillImageSensorConfigurations.cpp
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
#include "jaus/environment/stillimage/SetStillImageSensorConfigurations.h"

using namespace JAUS;

SetStillImageSensorConfigurations::SetStillImageSensorConfigurations(const Address& dest, const Address& src, Byte requestId) : 
Message(SET_STILL_IMAGE_SENSOR_CONFIGURATION, dest, src)
{
    mRequestId = requestId;
    mSensorList = List();
}

SetStillImageSensorConfigurations::SetStillImageSensorConfigurations(const SetStillImageSensorConfigurations& message) : 
Message(SET_STILL_IMAGE_SENSOR_CONFIGURATION)
{
    *this = message;
}

SetStillImageSensorConfigurations::~SetStillImageSensorConfigurations()
{
    mSensorList.clear();
}

int SetStillImageSensorConfigurations::WriteMessageBody(Packet& packet) const
{
    UInt startPos = packet.GetWritePos();
    UShort count = (UShort)mSensorList.size();
    List::const_iterator sensor = mSensorList.begin();
    packet.Write(mRequestId);           //required; send the requester ID
    packet.Write((UShort)mSensorList.size());   //required; Send the number of sensors
    while(sensor != mSensorList.end())
    {
        packet.Write(sensor->GetPresenceVector());              //required
        packet.Write(sensor->GetSensorId());                    //required
        if((sensor->GetPresenceVector() & StillImageSensorConfiguration::PresenceVector::FrameSize) > 0)
        {
            packet.Write(sensor->GetFrameSize());
        }
        if((sensor->GetPresenceVector() & StillImageSensorConfiguration::PresenceVector::ImageFormat) > 0)
        {
            packet.Write(sensor->GetImageFormat());
        }
        sensor++;
    }

    return packet.GetWritePos() - startPos;
}

int SetStillImageSensorConfigurations::ReadMessageBody(const Packet& packet)
{
    UInt startPos = packet.GetReadPos();
    
    UShort count = 0;
    packet.Read(mRequestId);
    packet.Read(count);
    for(int i = 0; i < (int)count; i++)
    {
        Byte presenceVector = 0;
        UShort sensorID = 0;
        packet.Read(presenceVector);
        packet.Read(sensorID);
        StillImageSensorConfiguration sisc = StillImageSensorConfiguration(presenceVector, sensorID);
        if((presenceVector & StillImageSensorConfiguration::PresenceVector::FrameSize) > 0)
        {
            Byte frameSize  = 0;
            packet.Read(frameSize);
            sisc.SetFrameSize(frameSize);
        }
        if((presenceVector & StillImageSensorConfiguration::PresenceVector::ImageFormat) > 0)
        {
            Byte imageFormat  = 0;
            packet.Read(imageFormat);
            sisc.SetImageFormat(imageFormat);
        }

        mSensorList.push_back(sisc);
    }
    return packet.GetReadPos() - startPos;
}

bool SetStillImageSensorConfigurations::IsLargeDataSet(const UInt maxPayloadSize) const
{
    //Quick Check to possibly avoid the time consuming loop (assume worst case of all fields supported)
    if(maxPayloadSize > (BYTE_SIZE) + ((BYTE_SIZE * 3 + USHORT_SIZE * 1 ) * mSensorList.size()))
    {
        return false;
    }

    UInt currentSize = BYTE_SIZE; //initialize to the requestId size then add the sive of each sensor
    List::const_iterator iter = mSensorList.begin();
    while(iter != mSensorList.end())
    {
        currentSize += iter->GetSize();        
        iter++;
    }
    
    return currentSize > maxPayloadSize;
}

void SetStillImageSensorConfigurations::PrintMessageBody() const
{
    std::cout << "Request Id: " << (UInt) mRequestId << std::endl;
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
