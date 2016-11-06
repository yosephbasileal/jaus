////////////////////////////////////////////////////////////////////////////////////
///
///  \file VisualSensorCapabilities.h
///  \brief Data structure representing a Visual Sensor Capabilities Data Record.
///
///  <br>Author(s): Daniel Barber, Jonathan Harris
///  <br>Created: 19 March 2013
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
#ifndef __JAUS_ENVIRONMENT_SENSING_VISUAL_SENSOR__CAPABILITIES__H
#define __JAUS_ENVIRONMENT_SENSING_VISUAL_SENSOR__CAPABILITIES__H

#include "jaus/core/Types.h"
#include "jaus/core/Message.h"
#include "jaus/environment/EnvironmentCodes.h"

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class VisualSensorCapabilities
    ///   \brief Data structure representing a single visual Sensor Capability
    ///   use within Report Visual Sensor Capabilities message.  
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_ENVIRONMENT_DLL VisualSensorCapabilities
    {
    public:
        class JAUS_ENVIRONMENT_DLL PresenceVector : public JAUS::PresenceVector
        {
        public:
            static const UShort SupportedStates         = 0x0001;
            static const UShort ZoomModes               = 0x0002;
            static const UShort FocusModes              = 0x0004;
            static const UShort WhiteBalance            = 0x0008;
            static const UShort ImagingModes            = 0x0010;
            static const UShort ExposureModes           = 0x0020;
            static const UShort MeteringModes           = 0x0040;
            static const UShort MinimumShutterSpeed    = 0x0080;
            static const UShort MaximumShutterSpeed    = 0x0100;
            static const UShort MinimumAperture         = 0x0200; 
            static const UShort MaximumAperture         = 0x0400;
            static const UShort MinimumFocalLength      = 0x0800;
            static const UShort MaximumFocalLength     = 0x1000;
            static const UShort LightSensitivityLevels  = 0x2000;
            static const UShort ImageStabilization      = 0x4000;
        };
        class JAUS_ENVIRONMENT_DLL SupportedStatesMask
        {
        public:
            static const Byte Active    = 0x01;
            static const Byte Standby   = 0x02;
            static const Byte Off       = 0x04;
        };
        class JAUS_ENVIRONMENT_DLL ZoomModesMask
        {
        public:
            static const Byte Mixed         = 0x01;
            static const Byte AnalogOnly    = 0x02;
            static const Byte DigitalOnly   = 0x04;
            static const Byte None          = 0x08;
        };
        class JAUS_ENVIRONMENT_DLL FocusModesMask
        {
        public:
            static const Byte AutoFocus     = 0x01;
            static const Byte ManualFocus   = 0x02;
        };
        class JAUS_ENVIRONMENT_DLL WhiteBalanceMask
        {
        public:
            static const Byte Auto          = 0x01;
            static const Byte Daylight      = 0x02;
            static const Byte Cloudy        = 0x04;
            static const Byte Shade         = 0x08;
            static const Byte Tungsten      = 0x10;
            static const Byte Flurescent    = 0x20;
            static const Byte Flash         = 0x40;
        };
        class JAUS_ENVIRONMENT_DLL ImagingModesMask
        {
        public:
            static const Byte Color     = 0x01;
            static const Byte Greyscale = 0x02;
            static const Byte Infrared  = 0x04;
            static const Byte Lowlight  = 0x08;
        };
        class JAUS_ENVIRONMENT_DLL ExposureModesMask
        {
        public:
            static const Byte Auti              = 0x01;
            static const Byte Manual            = 0x02;
            static const Byte ShutterPriority   = 0x04;
            static const Byte AperturePriority  = 0x08;
        };
        class JAUS_ENVIRONMENT_DLL MeteringModesMask
        {
        public:
            static const Byte Auto              = 0x01;
            static const Byte CenterWeighted    = 0x02;
            static const Byte Spot              = 0x04;
        };
        class JAUS_ENVIRONMENT_DLL LightSensitivityMask
        {
        public:
            static const Byte ISO_100   = 0x01;
            static const Byte ISO_200   = 0x02;
            static const Byte ISO_400   = 0x04;
            static const Byte ISO_800   = 0x08;
            static const Byte ISO_1600   = 0x10;
            static const Byte ISO_3200   = 0x10;
        };
        VisualSensorCapabilities(UShort presenceVector = 0 , UShort SensorId = -1, std::string SensorName  = "");
        ~VisualSensorCapabilities() {}
        virtual UShort GetPresenceVector() const { return mPresenceVector; }
        virtual UInt GetPresenceVectorSize() const { return USHORT_SIZE; }
        virtual UShort GetSensorId() const { return mSensorId; }
        virtual std::string GetSensorName() const { return mSensorName; }
        virtual Byte GetSuppoertedStates() const { return mSupportedStates; }
        virtual UInt GetSize() const;
        virtual Byte GetZoomModes() const {return mZoomModes; }
        virtual Byte GetFocusModes() const {return mFocusModes; }
        virtual Byte GetWhiteBalance() const {return mWhiteBalance; }
        virtual Byte GetImagingModes() const {return mImagingModes; }
        virtual Byte GetExposureModes() const {return mExposureModes; }
        virtual Byte GetMeteringModes() const {return mMeteringModes; }
        virtual UShort GetMinimumShutterSpeed() const {return mMinimumShutterSpeed; }
        virtual UShort GetMaximumShutterSpeed() const {return mMaximumShutterSpeed; }
        virtual UShort GetMinimumAperture() const {return mMinimumAperture; }
        virtual UShort GetMaximumAperture() const {return mMaximumAperture; }
        virtual UInt GetMinimumFocalLength() const {return mMinimumFocalLength; }
        virtual UInt GetMaximumFocalLength() const {return mMaximumFocalLength; }
        virtual Byte GetLightSensitivityLevels() const {return mLightSensitivityLevels; }
        virtual Byte GetImageStabilization() const {return mImageStabilization; }
        virtual void SetSensorId(UShort sensorId) 
        { 
            mSensorId = sensorId; 
        }
        virtual void SetSensorName(std::string sensorName) 
        {
            mSensorName = sensorName; 
        }
        virtual void SetSuppoertedStates(Byte supportedStatesMask) 
        { 
            mSupportedStates = supportedStatesMask;
        }
        virtual void SetZoomModes(Byte ZoomModeMask) 
        { 
            mZoomModes = ZoomModeMask;
        }
        virtual void SetFocusModes(Byte focusModesMask) 
        { 
            mFocusModes = focusModesMask;
        }
        virtual void SetWhiteBalance(Byte whiteBalanceMask) 
        { 
            mWhiteBalance = whiteBalanceMask;
        }
        virtual void SetImagingModes(Byte imagingModesMask) 
        { 
            mImagingModes = imagingModesMask;
        }
        virtual void SetExposureModes(Byte exposureModesMask) 
        { 
             mExposureModes = exposureModesMask;
        }
        virtual void SetMeteringModes(Byte meteringModesMask) 
        { 
            mMeteringModes = meteringModesMask;
        }
        virtual void SetMinimumShutterSpeed(UShort minimumShutterSpeed) 
        { 
             mMinimumShutterSpeed = minimumShutterSpeed;
        }
        virtual void SetMaximumShutterSpeed(UShort maximumShutterSpeed) 
        { 
            mMaximumShutterSpeed = maximumShutterSpeed;
        }
        virtual void SetMinimumAperture(UShort minimumAperture) 
        { 
            mMinimumAperture = minimumAperture;
        }
        virtual void SetMaximumAperture(UShort maximumAperture) 
        { 
            mMaximumAperture = maximumAperture;
        }
        virtual void SetMinimumFocalLength(UInt minimumFocalLength) 
        { 
            mMinimumFocalLength = minimumFocalLength;
        }
        virtual void SetMaximumFocalLength(UInt maximumFocalLength) 
        { 
            mMaximumFocalLength = maximumFocalLength;
        }
        virtual void SetLightSensitivityLevels(Byte lightSensitivityLevelsMask) 
        { 
            mLightSensitivityLevels = lightSensitivityLevelsMask;
        }
        virtual void SetImageStabilization(Byte imageStabilization) 
        { 
            mImageStabilization = imageStabilization;
        }
        virtual void PrintSensorFields() const;
        virtual int ReadMessageBody(const Packet& packet);
        virtual int WriteMessageBody(Packet& packet) const;
    private:
        UShort mPresenceVector;         ///<  Presence Vector. Required; Required
        UShort mSensorId;               ///<  SensorID value of "0" is invalid; Required
        std::string mSensorName;        ///<  Variable Length String; Count=byte
        Byte mSupportedStates;           ///<  Optional; Supported state; Bit Mask: 0=Active;  1=Standby; 2=Off;
        Byte mZoomModes;                ///<  Optional; Suppoerted Zoom Modes; Bit Mask: 0=Mixed, 1=AnalogOnly, 2=DigitalOnly, 3=None
        Byte mFocusModes;               ///<  Optional; Suppoerted Focus Modes; Bit Mask: 0=AutoFocus, 1=ManualFocus
        Byte mWhiteBalance;             ///<  Optional; Suppoerted White Balance Presets; Bit Mask: 0=Auto, 1=Daylight, 2=Cloudy, 3=Shade, 4=Tungsten, 5=Flurescent, 6=Flash
        Byte mImagingModes;               ///<  Optional; Supported Image modes; Bit Mask: 0=Color, 1=Grayscale, 2=Infrared, 3=Lowlight
        Byte mExposureModes;            ///<  Optional; Supported Exposure modes; Bit Mask: 0=Auto, 1=Manual, 2=ShutterPriority, 3=AperturePriority
        Byte mMeteringModes;            ///<  Optional; Supported Exposure metering modes; Bit Mask: 0=Auto, 1=CenterWeighted, 2=Spot
        UShort mMinimumShutterSpeed;    ///<  Optional; Minimum Shutter Speed as a scaled integer between 0.0 and 60.0
        UShort mMaximumShutterSpeed;    ///<  Optional; Maximum Shutter Speed as a scaled integer between 0.0 and 60.0
        UShort mMinimumAperture;        ///<  Optional; Minimum aperture value in f-stop value as a scaled integer between 0.1 amd 128
        UShort mMaximumAperture;        ///<  Optional; Maximum aperture value in f-stop value as a scaled integer between 0.1 amd 128
        UInt mMinimumFocalLength;       ///<  Optional; Minimum effective focal length in meters as a scaled integer between 0.0 and 2.0
        UInt mMaximumFocalLength;       ///<  Optional; Maximum effective focal length in meters as a scaled integer between 0.0 and 2.0
        Byte mLightSensitivityLevels;   ///<  Optional; ISO Film Speed equivalent; Bit Mask: 0=Auto, 1=ISO 100, 2=ISO 200, 3=ISO 400, 4=ISO 800, 5=ISO 1600, 6=ISO 3200
        Byte mImageStabilization;       ///<  Optional; Boolean value 0 = Does Not Supports ImageStabilization; 1 = Supports ImageStabilization;
    };
}

#endif
/*  End of File */
