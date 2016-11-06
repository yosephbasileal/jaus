////////////////////////////////////////////////////////////////////////////////////
///
///  \file StillImageSensorCapabilities.cpp
///  \brief Data structure representing a Still Image Sensor Capabilities Data Record.
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
#include "jaus/environment/stillimage/StillImageSensorCapabilities.h"

using namespace JAUS;

StillImageSensorCapabilities::StillImageSensorCapabilities(Byte PresenceVector, UShort  SensorId) : 
mPresenceVector(PresenceVector), mSensorId(SensorId)
{
    mSupportedFrameSizes = SupportedFrameSizesMask::qvga_320x240;
    mSupportedImageFormats = SupportedImageFormatsMask::JPEG;
}

int StillImageSensorCapabilities::ReadMessageBody(const Packet& packet) 
{
    UInt startPos = packet.GetReadPos();
    packet.Read(mPresenceVector);
    packet.Read(mSensorId);
    if((mPresenceVector & StillImageSensorCapabilities::PresenceVector::SupportedFrameSizes) > 0)
    {
        packet.Read(mSupportedFrameSizes);
    }
    if((mPresenceVector & StillImageSensorCapabilities::PresenceVector::SupportedImageFormats) > 0)
    {
        packet.Read(mSupportedImageFormats);
    }
    return packet.GetReadPos() - startPos;
}

int StillImageSensorCapabilities::WriteMessageBody(Packet& packet) const
{ 
    UInt startPos = packet.GetWritePos();
    packet.Write(mPresenceVector);
    packet.Write(mSensorId);
    if((mPresenceVector & StillImageSensorCapabilities::PresenceVector::SupportedFrameSizes) > 0)
    {
        packet.Write(mSupportedFrameSizes);
    }
    if((mPresenceVector & StillImageSensorCapabilities::PresenceVector::SupportedImageFormats) > 0)
    {
        packet.Write(mSupportedImageFormats);
    }
    return packet.GetWritePos() - startPos;
}

void StillImageSensorCapabilities::PrintSensorFields() const
{
    std::cout << "Size: " << (UInt) GetSize() << std::endl 
              << "Presence Vector:  " << (UInt) mPresenceVector  << std::endl
              << "Sensor Id:  " << (UInt) mSensorId  << std::endl
              << "Supported Frame Sizes:  " << (UInt) mSupportedFrameSizes  << std::endl
              << "Supported Image Formats: " << (UInt) mSupportedImageFormats << std::endl;
}

UInt StillImageSensorCapabilities::GetSize() const
{
    UInt size = BYTE_SIZE + USHORT_SIZE;

    if((mPresenceVector & PresenceVector::SupportedFrameSizes) > 0)
    {
        size += UINT_SIZE;
    }
    if((mPresenceVector & PresenceVector::SupportedImageFormats) > 0)
    {
        size += USHORT_SIZE;
    }
    return size;
}

/** End of File */
