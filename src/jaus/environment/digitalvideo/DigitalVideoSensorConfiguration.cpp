////////////////////////////////////////////////////////////////////////////////////
///
///  \file DigitalVideoSensorConfiguration.cpp
///  \brief Data structure representing a Digital Video Sensor Configuration Data Record.
///
///  <br>Author(s): Daniel Barber, Jonathan Harris
///  <br>Created: 20 March 2013
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
#include <string>
#include "jaus/environment/digitalvideo/DigitalVideoSensorConfiguration.h"

using namespace JAUS;

DigitalVideoSensorConfiguration::DigitalVideoSensorConfiguration(Byte PresenceVector, UShort  SensorId) : 
mPresenceVector(PresenceVector), mSensorId(SensorId)
{
    mMinimumBitRate = 200;
    mMaximumBitRate = 200;
    mFrameRate = 25;
    mFrameSize = (UInt) DigitalVideoSensorConfiguration::qvga_320x240;
    mDigitalFormat = (Byte) DigitalVideoSensorConfiguration::MPEG_4_Visual;
    mSize = BYTE_SIZE + USHORT_SIZE;     //Size is inilized to the size of the required fields
}

int DigitalVideoSensorConfiguration::ReadMessageBody(const Packet& packet) 
{
    UInt startPos = packet.GetReadPos();
    packet.Read(mPresenceVector);
    packet.Read(mSensorId);
    if((mPresenceVector & DigitalVideoSensorConfiguration::PresenceVector::MinimumBitRate) > 0)
    {
        packet.Read(mMinimumBitRate);
    }
    if((mPresenceVector & DigitalVideoSensorConfiguration::PresenceVector::MaximumBitRate) > 0)
    {
        packet.Read(mMaximumBitRate);
    }
    if((mPresenceVector & DigitalVideoSensorConfiguration::PresenceVector::FrameRate) > 0)
    {
        packet.Read(mFrameRate);
    }
    if((mPresenceVector & DigitalVideoSensorConfiguration::PresenceVector::FrameSize) > 0)
    {
        packet.Read(mFrameSize);
    }
    if((mPresenceVector & DigitalVideoSensorConfiguration::PresenceVector::DigitalFormat) > 0)
    {
        packet.Read(mDigitalFormat);
    }
    return packet.GetReadPos() - startPos;
}

int DigitalVideoSensorConfiguration::WriteMessageBody(Packet& packet) const
{ 
    UInt startPos = packet.GetWritePos();

    packet.Write(mPresenceVector);
        packet.Write(mSensorId);           //required
        if((mPresenceVector & DigitalVideoSensorConfiguration::PresenceVector::MinimumBitRate) > 0)
        {
            packet.Write(mMinimumBitRate);
        }
        if((mPresenceVector & DigitalVideoSensorConfiguration::PresenceVector::MaximumBitRate) > 0)
        {
            packet.Write(mMaximumBitRate);
        }
        if((mPresenceVector & DigitalVideoSensorConfiguration::PresenceVector::FrameRate) > 0)
        {
            packet.Write(mFrameRate);
        }
        if((mPresenceVector & DigitalVideoSensorConfiguration::PresenceVector::FrameSize) > 0)
        {
            packet.Write(mFrameSize);
        }
        if((mPresenceVector & DigitalVideoSensorConfiguration::PresenceVector::DigitalFormat) > 0)
        {
            packet.Write(mDigitalFormat);
        }
    return packet.GetWritePos() - startPos;
}

void DigitalVideoSensorConfiguration::PrintSensorFields() const
{
    std::cout << "Size: " << (UInt) mSize << std::endl 
              << "PresenceVector:  " << (UInt) mPresenceVector  << std::endl      
              << "Sensor Id:  " << (UInt) mSensorId  << std::endl
              << "MinimumBitRate:  " << (UInt) mMinimumBitRate  << std::endl
              << "MaximumBitRate:  " << (UInt) mMaximumBitRate  << std::endl
              << "FrameRate:  " << (UInt) mFrameRate  << std::endl
              << "FrameSize:  " << (UInt) mFrameSize  << std::endl
              << "DigitalFormat:  " << (UInt) mDigitalFormat  << std::endl;
}

/** End of File */
