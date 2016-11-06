////////////////////////////////////////////////////////////////////////////////////
///
///  \file VisualSensorCapabilities.cpp
///  \brief Data structure representing a Range Sensor Capabilities Data Record.
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
#include "jaus/environment/visual/VisualSensorCapabilities.h"

using namespace JAUS;

VisualSensorCapabilities::VisualSensorCapabilities(UShort PresenceVector, UShort  SensorId, std::string SensorName) : 
mPresenceVector(PresenceVector), mSensorId(SensorId), mSensorName(SensorName) 
{
    mSupportedStates = 0x0;
    mZoomModes = 0x0;
    mFocusModes = 0x0;
    mWhiteBalance = 0x0;
    mImagingModes = 0x0;
    mExposureModes = 0x0;
    mMeteringModes = 0x0;
    mMinimumShutterSpeed = 0;
    mMaximumShutterSpeed = 60;
    mMinimumAperture = 0;
    mMaximumAperture = 128;
    mMinimumFocalLength = 0;
    mMaximumFocalLength = 2;
    mLightSensitivityLevels = 0x0;
    mImageStabilization = 0;
}

int VisualSensorCapabilities::ReadMessageBody(const Packet& packet) 
{
    UInt startPos = packet.GetReadPos();
    Byte nameLength = 0;
    packet.Read(mPresenceVector);
    packet.Read(mSensorId);
    packet.Read(nameLength);
    packet.Read(mSensorName, (UInt) nameLength);
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::SupportedStates) > 0)
    {
        packet.Read(mSupportedStates);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::ZoomModes) > 0)
    {
        packet.Read(mZoomModes);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::FocusModes) > 0)
    {
        packet.Read(mFocusModes);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::WhiteBalance) > 0)
    {
        packet.Read(mWhiteBalance);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::ImagingModes) > 0)
    {
        packet.Read(mImagingModes);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::ExposureModes) > 0)
    {
        packet.Read(mExposureModes);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MeteringModes) > 0)
    {
        packet.Read(mMeteringModes);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MinimumShutterSpeed) > 0)
    {
        packet.Read(mMinimumShutterSpeed);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MaximumShutterSpeed) > 0)
    {
        packet.Read(mMaximumShutterSpeed);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MinimumAperture) > 0)
    {
        packet.Read(mMinimumAperture);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MaximumAperture) > 0)
    {
        packet.Read(mMaximumAperture);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MinimumFocalLength) > 0)
    {
        packet.Read(mMinimumFocalLength);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MaximumFocalLength) > 0)
    {
        packet.Read(mMaximumFocalLength);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::LightSensitivityLevels) > 0)
    {
        packet.Read(mLightSensitivityLevels);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::ImageStabilization) > 0)
    {
        packet.Read(mImageStabilization);
    }
    return packet.GetReadPos() - startPos;
}

int VisualSensorCapabilities::WriteMessageBody(Packet& packet) const
{ 
    UInt startPos = packet.GetWritePos();
    packet.Write(mPresenceVector);
    packet.Write(mSensorId);  
    packet.Write((Byte) mSensorName.length());
    packet.Write(mSensorName);
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::SupportedStates) > 0)
    {
        packet.Write(mSupportedStates);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::ZoomModes) > 0)
    {
        packet.Write(mZoomModes);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::FocusModes) > 0)
    {
        packet.Write(mFocusModes);
    }        
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::WhiteBalance) > 0)
    {
        packet.Write(mWhiteBalance);
    } 
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::ImagingModes) > 0)
    {
        packet.Write(mImagingModes);
    } 
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::ExposureModes) > 0)
    {
        packet.Write(mExposureModes);
    } 
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MeteringModes) > 0)
    {
        packet.Write(mMeteringModes);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MinimumShutterSpeed) > 0)
    {
        packet.Write(mMinimumShutterSpeed);
    } 
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MaximumShutterSpeed) > 0)
    {
        packet.Write(mMaximumShutterSpeed);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MinimumAperture) > 0)
    {
        packet.Write(mMinimumAperture);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MaximumAperture) > 0)
    {
        packet.Write(mMaximumAperture);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MinimumFocalLength) > 0)
    {
        packet.Write(mMinimumFocalLength);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MaximumFocalLength) > 0)
    {
        packet.Write(mMaximumFocalLength);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::LightSensitivityLevels) > 0)
    {
        packet.Write(mLightSensitivityLevels);
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::ImageStabilization) > 0)
    {
        packet.Write(mImageStabilization);
    }
    return packet.GetWritePos() - startPos;
}

void VisualSensorCapabilities::PrintSensorFields() const
{
    std::cout << "Size: " << GetSize() << std::endl 
        << "Presence Vector:  " << (UInt) mPresenceVector  << std::endl
        << "SensorId:  " << (UInt) mSensorId  << std::endl
        << "Sensor Name:  " << mSensorName  << std::endl
        << "Supported States:  " << (UInt) mSupportedStates  << std::endl
        << "Zoom Modes:  " << (UInt) mZoomModes  << std::endl
        << "Focus Modes:  " << (UInt) mFocusModes  << std::endl
        << "White Balance:  " << (UInt) mWhiteBalance  << std::endl
        << "Imaging Modes:  " << (UInt) mImagingModes  << std::endl
        << "Exposure Modes:  " << (UInt) mExposureModes  << std::endl
        << "Metering Modes:  " << (UInt) mMeteringModes  << std::endl
        << "Minimum Shutter Speed:  " << (UInt) mMinimumShutterSpeed  << std::endl
        << "Maximum Shutter Speed:  " << (UInt) mMaximumShutterSpeed  << std::endl
        << "Minimum Aperture:  " << (UInt) mMinimumAperture  << std::endl
        << "Maximum Aperture:  " << (UInt) mMaximumAperture  << std::endl
        << "Minimum Focal Length:  " << (UInt) mMinimumFocalLength  << std::endl
        << "Maximum Focal Length:  " << (UInt) mMaximumFocalLength  << std::endl
        << "Light SensitivityLevels:  " << (UInt) mLightSensitivityLevels  << std::endl
        << "Image Stabilization:  " << (UInt) mImageStabilization  << std::endl;
}

UInt VisualSensorCapabilities::GetSize() const
{
    UInt size = USHORT_SIZE + USHORT_SIZE + BYTE_SIZE;     //Size is inilized to the size of the required fields
    size += GetSensorName().length();

    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::SupportedStates) > 0)
    {
        size += BYTE_SIZE;
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::ZoomModes) > 0)
    {
        size += BYTE_SIZE;
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::FocusModes) > 0)
    {
        size += BYTE_SIZE;
    }        
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::WhiteBalance) > 0)
    {
        size += BYTE_SIZE;
    } 
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::ImagingModes) > 0)
    {
        size += BYTE_SIZE;
    } 
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::ExposureModes) > 0)
    {
        size += BYTE_SIZE;
    } 
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MeteringModes) > 0)
    {
        size += BYTE_SIZE;
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MinimumShutterSpeed) > 0)
    {
        size += USHORT_SIZE;
    } 
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MaximumShutterSpeed) > 0)
    {
        size += USHORT_SIZE;
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MinimumAperture) > 0)
    {
        size += USHORT_SIZE;
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MaximumAperture) > 0)
    {
        size += USHORT_SIZE;
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MinimumFocalLength) > 0)
    {
        size += UINT_SIZE;
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::MaximumFocalLength) > 0)
    {
        size += UINT_SIZE;
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::LightSensitivityLevels) > 0)
    {
        size += BYTE_SIZE;
    }
    if((mPresenceVector & VisualSensorCapabilities::PresenceVector::ImageStabilization) > 0)
    {
        size += BYTE_SIZE;
    }
    return size;
}

/** End of File */
