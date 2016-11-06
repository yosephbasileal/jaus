////////////////////////////////////////////////////////////////////////////////////
///
///  \file RangeSensorConfiguration.cpp
///  \brief Data structure representing a Range Sensor Capabilities Data Record.
///
///  <br>Author(s): Daniel Barber, Jonathan Harris
///  <br>Created: 18 March 2013
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
#include "jaus/environment/range/RangeSensorConfiguration.h"

using namespace JAUS;

RangeSensorConfiguration::RangeSensorConfiguration(UShort presenceVector, UShort  sensorId) : 
mPresenceVector(presenceVector), mSensorId(sensorId)
{
    mHorizontalFieldOfViewStartAngle = 0;
    mHorizontalFieldOfViewStopAngle = 1;
    mVerticalFieldOfViewStartAngle = 0;
    mVerticalFieldOfViewStopAngle = 1;
    mUpdateRate = 1000;
    mMinimumRange = 0;
    mMaximumRange = 1,000,000;
    mSensorState = RangeSensorConfiguration::Active;
}

int RangeSensorConfiguration::ReadMessageBody(const Packet& packet) 
{
    UInt startPos = packet.GetReadPos();
    packet.Read(mPresenceVector);
    packet.Read(mSensorId);
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::HorizontalFieldOfViewStartAngle) > 0)
    {
        packet.Read(mHorizontalFieldOfViewStartAngle);
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::HorizontalFieldOfViewStopAngle) > 0)
    {
        packet.Read(mHorizontalFieldOfViewStopAngle);
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::VerticalFieldOfViewStartAngle) > 0)
    {
        packet.Read(mVerticalFieldOfViewStartAngle);
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::VerticalFieldOfViewStopAngle) > 0)
    {
        packet.Read(mVerticalFieldOfViewStopAngle);
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::UpdateRate) > 0)
    {
        packet.Read(mUpdateRate);
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::MinimumRange) > 0)
    {
        packet.Read(mMinimumRange);
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::MaximumRange) > 0)
    {
        packet.Read(mMaximumRange);
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::SensorState) > 0)
    {
        packet.Read(mSensorState);
    }
    return packet.GetReadPos() - startPos;
}

int RangeSensorConfiguration::WriteMessageBody(Packet& packet) const
{ 
    UInt startPos = packet.GetWritePos();
    packet.Write(mPresenceVector);
    packet.Write(mSensorId);
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::HorizontalFieldOfViewStartAngle) > 0)
    {
        packet.Write(mHorizontalFieldOfViewStartAngle);
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::HorizontalFieldOfViewStopAngle) > 0)
    {
        packet.Write(mHorizontalFieldOfViewStopAngle);
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::VerticalFieldOfViewStartAngle) > 0)
    {
        packet.Write(mVerticalFieldOfViewStartAngle);
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::VerticalFieldOfViewStopAngle) > 0)
    {
        packet.Write(mVerticalFieldOfViewStopAngle);
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::UpdateRate) > 0)
    {
        packet.Write(mUpdateRate);
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::MinimumRange) > 0)
    {
        packet.Write(mMinimumRange);
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::MaximumRange) > 0)
    {
        packet.Write(mMaximumRange);
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::SensorState) > 0)
    {
        packet.Write(mSensorState);
    }
    return packet.GetWritePos() - startPos;
}

void RangeSensorConfiguration::PrintSensorFields() const
{
    std::cout << "Size: " << GetSize() << std::endl 
              << "PresenceVector:  " << (UInt) mPresenceVector  << std::endl      
              << "Sensor Id:  " << (UInt) mSensorId  << std::endl
              << "Horizontal Field Of View Start Angle:  " << (UInt) mHorizontalFieldOfViewStartAngle  << std::endl
              << "Horizontal Field Of View Stop Angle:  " << (UInt) mHorizontalFieldOfViewStopAngle  << std::endl
              << "Vertical Field Of View Start Angle:  " << (UInt) mVerticalFieldOfViewStartAngle  << std::endl
              << "Vertical Field Of View Stop Angle:  " << (UInt) mVerticalFieldOfViewStopAngle  << std::endl
              << "Update Rate:  " << (UInt) mUpdateRate  << std::endl
              << "Minimum Range:  " << (UInt) mMinimumRange  << std::endl
              << "Maximum Range:  " << (UInt) mMaximumRange  << std::endl
              << "Sensor State:  " << (UInt) mSensorState  << std::endl;
}

UInt RangeSensorConfiguration::GetSize() const
{
    UInt size = USHORT_SIZE + USHORT_SIZE;
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::HorizontalFieldOfViewStartAngle) > 0)
    {
        size += UINT_SIZE;
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::HorizontalFieldOfViewStopAngle) > 0)
    {
        size += UINT_SIZE;
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::VerticalFieldOfViewStartAngle) > 0)
    {
        size += UINT_SIZE;
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::VerticalFieldOfViewStopAngle) > 0)
    {
        size += UINT_SIZE;
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::UpdateRate) > 0)
    {
        size += USHORT_SIZE;
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::MinimumRange) > 0)
    {
        size += UINT_SIZE;
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::MaximumRange) > 0)
    {
        size += UINT_SIZE;
    }
    if((mPresenceVector & RangeSensorConfiguration::PresenceVector::SensorState) > 0)
    {
        size += BYTE_SIZE;
    }
    return size;
}

/** End of File */
