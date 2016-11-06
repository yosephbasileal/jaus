////////////////////////////////////////////////////////////////////////////////////
///
///  \file RangeSensorCapabilities.cpp
///  \brief Data structure representing a Range Sensor Capabilities Data Record.
///
///  <br>Author(s): Daniel Barber, Jonathan Harris
///  <br>Created: 13 March 2013
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
#include "jaus/environment/range/RangeSensorCapabilities.h"

using namespace JAUS;

RangeSensorCapabilities::RangeSensorCapabilities(UShort PresenceVector, UShort  SensorId, std::string SensorName) : 
mPresenceVector(PresenceVector), mSensorId(SensorId), mSensorName(SensorName) 
{
    mSupportedStates = 0;
    mMinimumHorizontalFieldOfViewStartAngle = 0;
    mMaximumHorizontalFieldOfViewStopAngle = 1;
    mMinimumVerticalFieldOfViewStartAngle = 0;
    mMaximumVerticalFieldOfViewStopAngle = 1;
    mMinimumUpdateRate = 0;
    mMaximumUpdateRate = 1000;
    mMinimumRange = 0;
    mMaximumRange = 1,000,000;
    mSupportedCompression = 0x1;
    mCoordinateTransformationSupported = 0;
    mSize = USHORT_SIZE + USHORT_SIZE + BYTE_SIZE;     //Size is inilized to the size of the required fields
}

int RangeSensorCapabilities::ReadMessageBody(const Packet& packet) 
{
    UInt startPos = packet.GetReadPos();
    Byte nameLength;
       packet.Read(mPresenceVector);
        packet.Read(mSensorId);
        packet.Read(nameLength);
        packet.Read(mSensorName, (UInt) nameLength);
        if((mPresenceVector & RangeSensorCapabilities::PresenceVector::SupportedStates) > 0)
        {
            packet.Read(mSupportedStates);
        }
        if((mPresenceVector & RangeSensorCapabilities::PresenceVector::MinimumHorizontalFieldOfViewStartAngle) > 0)
        {
            packet.Read(mMinimumHorizontalFieldOfViewStartAngle);
        }
        if((mPresenceVector & RangeSensorCapabilities::PresenceVector::MaximumHorizontalFieldOfViewStopAngle) > 0)
        {
            packet.Read(mMaximumHorizontalFieldOfViewStopAngle);
        }
        if((mPresenceVector & RangeSensorCapabilities::PresenceVector::MinimumVerticalFieldOfViewStartAngle) > 0)
        {
            packet.Read(mMinimumVerticalFieldOfViewStartAngle);
        }
        if((mPresenceVector & RangeSensorCapabilities::PresenceVector::MaximumVerticalFieldOfViewStopAngle) > 0)
        {
            packet.Read(mMaximumVerticalFieldOfViewStopAngle);
        }
        if((mPresenceVector & RangeSensorCapabilities::PresenceVector::MaximumUpdateRate) > 0)
        {
            packet.Read(mMaximumUpdateRate);
        }
        if((mPresenceVector & RangeSensorCapabilities::PresenceVector::MaximumUpdateRate) > 0)
        {
            packet.Read(mMaximumUpdateRate);
        }
        if((mPresenceVector & RangeSensorCapabilities::PresenceVector::MinimumRange) > 0)
        {
            packet.Read(mMinimumRange);
        }
        if((mPresenceVector & RangeSensorCapabilities::PresenceVector::MaximumRange) > 0)
        {
            packet.Read(mMaximumRange);
        }
        if((mPresenceVector & RangeSensorCapabilities::PresenceVector::SupportedCompression) > 0)
        {
            packet.Read(mSupportedCompression);
        }
        if((mPresenceVector & RangeSensorCapabilities::PresenceVector::CoordinateTransformationSupported) > 0)
        {
            packet.Read(mCoordinateTransformationSupported);
        }
    return packet.GetReadPos() - startPos;
}

int RangeSensorCapabilities::WriteMessageBody(Packet& packet) const
{ 
    UInt startPos = packet.GetWritePos();
    packet.Write(mPresenceVector);              //required
    packet.Write(mSensorId);                    //required
    packet.Write((Byte)mSensorName.length());   //Must write the length of the string as a byte before the string
    packet.Write(mSensorName);                  //required
    if((mPresenceVector & RangeSensorCapabilities::PresenceVector::SupportedStates) > 0)
    {
        packet.Write(mSupportedStates);
    }
    if((mPresenceVector & RangeSensorCapabilities::PresenceVector::MinimumHorizontalFieldOfViewStartAngle) > 0)
    {
        packet.Write(mMinimumHorizontalFieldOfViewStartAngle);
    }
    if((mPresenceVector & RangeSensorCapabilities::PresenceVector::MaximumHorizontalFieldOfViewStopAngle) > 0)
    {
        packet.Write(mMaximumHorizontalFieldOfViewStopAngle);
    }
    if((mPresenceVector & RangeSensorCapabilities::PresenceVector::MinimumVerticalFieldOfViewStartAngle) > 0)
    {
        packet.Write(mMinimumVerticalFieldOfViewStartAngle);
    }
    if((mPresenceVector & RangeSensorCapabilities::PresenceVector::MaximumVerticalFieldOfViewStopAngle) > 0)
    {
        packet.Write(mMaximumVerticalFieldOfViewStopAngle);
    }
    if((mPresenceVector & RangeSensorCapabilities::PresenceVector::MaximumUpdateRate) > 0)
    {
        packet.Write(mMaximumUpdateRate);
    }
    if((mPresenceVector & RangeSensorCapabilities::PresenceVector::MaximumUpdateRate) > 0)
    {
        packet.Write(mMaximumUpdateRate);
    }
    if((mPresenceVector & RangeSensorCapabilities::PresenceVector::MinimumRange) > 0)
    {
        packet.Write(mMinimumRange);
    }
    if((mPresenceVector & RangeSensorCapabilities::PresenceVector::MaximumRange) > 0)
    {
        packet.Write(mMaximumRange);
    }
    if((mPresenceVector & RangeSensorCapabilities::PresenceVector::SupportedCompression) > 0)
    {
        packet.Write(mSupportedCompression);
    }
    if((mPresenceVector & RangeSensorCapabilities::PresenceVector::CoordinateTransformationSupported) > 0)
    {
        packet.Write(mCoordinateTransformationSupported);
    }
    return packet.GetWritePos() - startPos;
}

void RangeSensorCapabilities::PrintSensorFields() const
{
    std::cout << "Size: " << (UInt) mSize << std::endl 
              << "PresenceVector:  " << (UInt) mPresenceVector  << std::endl      
              << "Sensor Id:  " << (UInt) mSensorId  << std::endl
              << "Sensor Name:  " << mSensorName  << std::endl
              << "mSupportedStates:  " << (UInt) mSupportedStates  << std::endl
              << "Minimum Horizontal Field Of View Start Angle:  " << (UInt) mMinimumHorizontalFieldOfViewStartAngle  << std::endl
              << "Maximum Horizontal Field Of View Stop Angle:  " << (UInt) mMaximumHorizontalFieldOfViewStopAngle  << std::endl
              << "Minimum Vertical Field Of View Start Angle:  " << (UInt) mMinimumVerticalFieldOfViewStartAngle  << std::endl
              << "Maximum Vertical Field Of View Stop Angle:  " << (UInt) mMaximumVerticalFieldOfViewStopAngle  << std::endl
              << "Minimum Update Rate:  " << (UInt) mMinimumUpdateRate  << std::endl
              << "Maximum Update Rate:  " << (UInt) mMaximumUpdateRate  << std::endl
              << "Minimum Range:  " << (UInt) mMinimumRange  << std::endl
              << "Maximum Range:  " << (UInt) mMaximumRange  << std::endl
              << "Supported Compressione:  " << (UInt) mSupportedCompression  << std::endl
              << "Coordinate Transformation Supported:  " << (UInt) mCoordinateTransformationSupported  << std::endl;
}

/** End of File */

