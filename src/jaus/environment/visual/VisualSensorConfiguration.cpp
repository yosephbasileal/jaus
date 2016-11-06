////////////////////////////////////////////////////////////////////////////////////
///
///  \file VisualSensorConfiguration.cpp
///  \brief Data structure representing a Range Sensor Configuration Data Record.
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
#include "jaus/environment/visual/VisualSensorConfiguration.h"

using namespace JAUS;

VisualSensorConfiguration::VisualSensorConfiguration(UShort PresenceVector, UShort  SensorId) : 
mPresenceVector(PresenceVector), mSensorId(SensorId) 
{
    mSensorState = Active;
    mZoomMode = 0;
    mZoomLevel = 0;         
    mFocalLength = 0;      
    mHorizontalFieldOfView = 0;
    mVerticalFieldOfView = 0;
    mFocusMode = 0;
    mFocusValue = 0;
    mWhiteBalance = AutoWiteBalance;
    mImagingMode = Color;
    mExposureMode = AutoExposure;
    mMeteringMode = AutoMetering;
    mShutterSpeed = 0;
    mAperture = 0;
    mLightSensitivity = AutoLight;
    mImageStabilization = 0;
}

int VisualSensorConfiguration::ReadMessageBody(const Packet& packet) 
{
    UInt startPos = packet.GetReadPos();
    packet.Read(mPresenceVector);
    packet.Read(mSensorId);
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::SensorState) > 0)
    {
        packet.Read(mSensorState);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ZoomMode) > 0)
    {
        packet.Read(mZoomMode);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ZoomLevel) > 0)
    {
        packet.Read(mZoomLevel);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::FocalLength) > 0)
    {
        packet.Read(mFocalLength);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::HorizontalFieldOfView) > 0)
    {
        packet.Read(mHorizontalFieldOfView);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::VerticalFieldOfView) > 0)
    {
        packet.Read(mVerticalFieldOfView);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::FocusMode) > 0)
    {
        packet.Read(mFocusMode);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::FocusValue) > 0)
    {
        packet.Read(mFocusValue);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::WhiteBalance) > 0)
    {
        packet.Read(mWhiteBalance);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ImagingMode) > 0)
    {
        packet.Read(mImagingMode);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ExposureMode) > 0)
    {
        packet.Read(mExposureMode);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::MeteringMode) > 0)
    {
        packet.Read(mMeteringMode);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ShutterSpeed) > 0)
    {
        packet.Read(mShutterSpeed);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::Aperture) > 0)
    {
        packet.Read(mAperture);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::LightSensitivity) > 0)
    {
        packet.Read(mLightSensitivity);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ImageStabilization) > 0)
    {
        packet.Read(mImageStabilization);
    }
    return packet.GetReadPos() - startPos;
}

int VisualSensorConfiguration::WriteMessageBody(Packet& packet) const
{ 
    UInt startPos = packet.GetWritePos();
    packet.Write(mPresenceVector); 
    packet.Write(mSensorId); 
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::SensorState) > 0)
    {
        packet.Write(mSensorState);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ZoomMode) > 0)
    {
        packet.Write(mZoomMode);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ZoomLevel) > 0)
    {
        packet.Write(mZoomLevel);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::FocalLength) > 0)
    {
        packet.Write(mFocalLength);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::HorizontalFieldOfView) > 0)
    {
        packet.Write(mHorizontalFieldOfView);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::VerticalFieldOfView) > 0)
    {
        packet.Write(mVerticalFieldOfView);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::FocusMode) > 0)
    {
        packet.Write(mFocusMode);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::FocusValue) > 0)
    {
        packet.Write(mFocusValue);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::WhiteBalance) > 0)
    {
        packet.Write(mWhiteBalance);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ImagingMode) > 0)
    {
        packet.Write(mImagingMode);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ExposureMode) > 0)
    {
        packet.Write(mExposureMode);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::MeteringMode) > 0)
    {
        packet.Write(mMeteringMode);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ShutterSpeed) > 0)
    {
        packet.Write(mShutterSpeed);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::Aperture) > 0)
    {
        packet.Write(mAperture);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::LightSensitivity) > 0)
    {
        packet.Write(mLightSensitivity);
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ImageStabilization) > 0)
    {
        packet.Write(mImageStabilization);
    }
    return packet.GetWritePos() - startPos;
}

void VisualSensorConfiguration::PrintSensorFields() const
{
    std::cout << "Size: " << GetSize() << std::endl 
              << "mPresenceVector:  " << (UInt) mPresenceVector  << std::endl
              << "SensorId:  " << (UInt) mSensorId  << std::endl
              << "mSensorState:  " << (UInt) mSensorState  << std::endl
              << "mZoomMode:  " << (UInt) mZoomMode  << std::endl
              << "mZoomLevel:  " << (UInt) mZoomLevel  << std::endl
              << "mFocalLength:  " << (UInt) mFocalLength  << std::endl
              << "mHorizontalFieldOfView:  " << (UInt) mHorizontalFieldOfView  << std::endl
              << "mVerticalFieldOfView:  " << (UInt) mVerticalFieldOfView  << std::endl
              << "mFocusMode:  " << (UInt) mFocusMode  << std::endl
              << "mFocusValue:  " << (UInt) mFocusValue  << std::endl
              << "mWhiteBalance:  " << (UInt) mWhiteBalance  << std::endl
              << "mImagingMode:  " << (UInt) mImagingMode  << std::endl
              << "mExposureMode:  " << (UInt) mExposureMode  << std::endl
              << "mMeteringModes:  " << (UInt) mMeteringMode  << std::endl
              << "mShutterSpeed:  " << (UInt) mShutterSpeed  << std::endl
              << "mAperture:  " << (UInt) mAperture  << std::endl
              << "mLightSensitivity:  " << (UInt) mLightSensitivity  << std::endl
              << "mImageStabilization:  " << (UInt) mImageStabilization  << std::endl;
}

UInt VisualSensorConfiguration::GetSize() const
{
    UInt size = USHORT_SIZE + USHORT_SIZE;     //Size is inilized to the size of the required fields

    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::SensorState) > 0)
    {
        size += BYTE_SIZE;
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ZoomMode) > 0)
    {
        size += BYTE_SIZE;
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ZoomLevel) > 0)
    {
        size += USHORT_SIZE;
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::FocalLength) > 0)
    {
        size += UINT_SIZE;
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::HorizontalFieldOfView) > 0)
    {
        size += UINT_SIZE;
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::VerticalFieldOfView) > 0)
    {
        size += UINT_SIZE;
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::FocusMode) > 0)
    {
        size += BYTE_SIZE;
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::FocusValue) > 0)
    {
        size += USHORT_SIZE;
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::WhiteBalance) > 0)
    {
        size += BYTE_SIZE;
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ImagingMode) > 0)
    {
        size += BYTE_SIZE;
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ExposureMode) > 0)
    {
        size += BYTE_SIZE;
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::MeteringMode) > 0)
    {
        size += BYTE_SIZE;
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ShutterSpeed) > 0)
    {
        size += USHORT_SIZE;
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::Aperture) > 0)
    {
        size += USHORT_SIZE;
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::LightSensitivity) > 0)
    {
        size += BYTE_SIZE;
    }
    if((mPresenceVector & VisualSensorConfiguration::PresenceVector::ImageStabilization) > 0)
    {
        size += BYTE_SIZE;
    }
    return size;
}
/** End of File */
