////////////////////////////////////////////////////////////////////////////////////
///
///  \file StillImageSensorConfiguration.cpp
///  \brief Data structure representing a Still Image Sensor Configuration Data Record.
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
#include <string>
#include "jaus/environment/stillimage/StillImageSensorConfiguration.h"

using namespace JAUS;

StillImageSensorConfiguration::StillImageSensorConfiguration(Byte PresenceVector, UShort  SensorId) : 
mPresenceVector(PresenceVector), mSensorId(SensorId)
{
   
}

int StillImageSensorConfiguration::ReadMessageBody(const Packet& packet) 
{
    UInt startPos = packet.GetReadPos();
    packet.Read(mPresenceVector);
    packet.Read(mSensorId);
    if((mPresenceVector & StillImageSensorConfiguration::PresenceVector::FrameSize) > 0)
    {
        packet.Read(mFrameSize);
    }
    if((mPresenceVector & StillImageSensorConfiguration::PresenceVector::ImageFormat) > 0)
    {
        packet.Read(mImageFormat);
    }
    return packet.GetReadPos() - startPos;
}

int StillImageSensorConfiguration::WriteMessageBody(Packet& packet) const
{ 
    UInt startPos = packet.GetWritePos();
    packet.Write(mPresenceVector);
    packet.Write(mSensorId);
    if((mPresenceVector & StillImageSensorConfiguration::PresenceVector::FrameSize) > 0)
    {
        packet.Write(mFrameSize);
    }
    if((mPresenceVector & StillImageSensorConfiguration::PresenceVector::ImageFormat) > 0)
    {
        packet.Write(mImageFormat);
    }
    return packet.GetWritePos() - startPos;
}

void StillImageSensorConfiguration::PrintSensorFields() const
{
    std::cout << "Size: " << (UInt) GetSize() << std::endl 
              << "Presence Vector:  " << (UInt) mPresenceVector  << std::endl
              << "Sensor Id:  " << (UInt) mSensorId  << std::endl
              << "Frame Sizes:  " << (UInt) mFrameSize  << std::endl
              << "Image Formats: " << (UInt) mImageFormat << std::endl;
}

UInt StillImageSensorConfiguration::GetSize() const 
{
    UInt size = BYTE_SIZE + USHORT_SIZE;

    if((mPresenceVector & PresenceVector::FrameSize) > 0)
    {
        size += BYTE_SIZE;
    }
    if((mPresenceVector & PresenceVector::ImageFormat) > 0)
    {
        size += BYTE_SIZE;
    }

    return size;
}

/** End of File */
