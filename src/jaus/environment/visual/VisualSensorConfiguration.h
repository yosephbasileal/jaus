////////////////////////////////////////////////////////////////////////////////////
///
///  \file VisualSensorConfiguration.h
///  \brief Data structure representing a Visual Sensor Configuration Data Record.
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
#ifndef __JAUS_ENVIRONMENT_SENSING_VISUAL_SENSOR__CONFIGURATION__H
#define __JAUS_ENVIRONMENT_SENSING_VISUAL_SENSOR__CONFIGURATION__H

#include "jaus/core/Types.h"
#include "jaus/core/Message.h"
#include "jaus/environment/EnvironmentCodes.h"

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class VisualSensorConfiguration
    ///   \brief Data structure representing a single visual Sensor Configuration
    ///   use within Report Visual Sensor Configuration message.  
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_ENVIRONMENT_DLL VisualSensorConfiguration
    {
    public:
        class JAUS_ENVIRONMENT_DLL PresenceVector : public JAUS::PresenceVector
        {
        public:
            static const UShort SensorState             = 0x0001;
            static const UShort ZoomMode                = 0x0002;
            static const UShort ZoomLevel               = 0x0004;
            static const UShort FocalLength            = 0x0008;
            static const UShort HorizontalFieldOfView   = 0x0010;
            static const UShort VerticalFieldOfView     = 0x0020;
            static const UShort FocusMode               = 0x0040;
            static const UShort FocusValue              = 0x0080;
            static const UShort WhiteBalance            = 0x0100;
            static const UShort ImagingMode             = 0x0200;
            static const UShort ExposureMode            = 0x0400; 
            static const UShort MeteringMode            = 0x0800;
            static const UShort ShutterSpeed            = 0x1000;
            static const UShort Aperture                = 0x2000;
            static const UShort LightSensitivity        = 0x4000;
            static const UShort ImageStabilization      = 0x8000;
        };
        enum JAUS_ENVIRONMENT_DLL SensorState {   Active = 0, Standby = 1, Off = 2 };
        enum JAUS_ENVIRONMENT_DLL ZoomMode 
        { 
            Mixed       = 0,
            AnalogOnly  = 1, 
            DigitalOnly = 2,
            None        =3
        };
        enum JAUS_ENVIRONMENT_DLL FocusMode { AutoFocus = 0, ManualFocus = 1 };
        enum JAUS_ENVIRONMENT_DLL WhiteBalanceMask
        {
            AutoWiteBalance          = 0,
            Daylight      = 1,
            Cloudy        = 2,
            Shade         = 3,
            Tungsten      = 4,
            Flurescent    = 5,
            Flash         = 6
        };
        enum JAUS_ENVIRONMENT_DLL ImagingMode
        {
            Color     = 0,
            Greyscale = 1,
            Infrared  = 2,
            Lowlight  = 3
        };
        enum JAUS_ENVIRONMENT_DLL ExposureMode
        {
            AutoExposure        = 0,
            Manual              = 1,
            ShutterPriority     = 2,
            AperturePriority    = 3
        };
        enum JAUS_ENVIRONMENT_DLL MeteringMode
        {
            AutoMetering    = 0,
            CenterWeighted  = 1,
            Spot            = 2
        };
        enum JAUS_ENVIRONMENT_DLL LightSensitivity
        {
            AutoLight   = 0,
            ISO_100     = 1,
            ISO_200     = 2,
            ISO_400     = 3,
            ISO_800     = 4,
            ISO_1600    = 5,
            ISO_3200    = 6
        };
        VisualSensorConfiguration(UShort presenceVector = 0 , UShort SensorId = -1);
        ~VisualSensorConfiguration() {}
        virtual UShort GetPresenceVector() const { return mPresenceVector; }
        virtual UInt GetPresenceVectorSize() const { return USHORT_SIZE; }
        virtual UShort GetSensorID() const { return mSensorId; }
        virtual Byte GetSensorState() const { return mSensorState; }
        virtual UInt GetSize() const;
        virtual Byte GetZoomMode() const {return mZoomMode; }
        virtual UShort GetZoomLevel() const {return mZoomLevel; }
        virtual UInt GetFocalLength() const { return mFocalLength; }
        virtual UInt GetHorizontalFieldOfView() const { return mHorizontalFieldOfView; }
        virtual UInt GetVerticalFieldOfView() const { return mVerticalFieldOfView; }
        virtual Byte GetFocusMode() const { return mFocusMode; }
        virtual UShort GetFocusValue() const { return mFocusValue; }
        virtual Byte GetWhiteBalance() const { return mWhiteBalance; }
        virtual Byte GetImagingMode() const { return mImagingMode; }
        virtual Byte GetExposureMode() const { return mExposureMode; }
        virtual Byte GetMeteringMode() const { return mMeteringMode; }
        virtual UShort GetShutterSpeed() const { return mShutterSpeed; }
        virtual UShort GetAperture() const { return mAperture; }
        virtual Byte GetLightSensitivity() const { return mLightSensitivity; }
        virtual Byte GetImageStabilization() const {return mImageStabilization; }
        virtual void SetSensorId(UShort sensorId) 
        { 
            mSensorId = sensorId; 
        }
        virtual void SetSensorState(Byte sensorState)
        { 
            mSensorState = sensorState;
        }
        virtual void SetZoomMode(Byte zoomMode)
        { 
            mZoomMode = zoomMode;
        }
        virtual void SetZoomLevel(UShort zoomLevel)
        { 
            mZoomLevel = zoomLevel;
        }
        virtual void SetFocalLength(UInt focalLength)
        { 
            mFocalLength = focalLength;
        }
        virtual void SetHorizontalFieldOfView(UInt horizontalFieldOfView)
        { 
            mHorizontalFieldOfView = horizontalFieldOfView;
        }
        virtual void SetVerticalFieldOfView(UInt verticalFieldOfView)
        { 
            mVerticalFieldOfView = verticalFieldOfView;
        }
        virtual void SetFocusMode(Byte focusMode)
        { 
            mFocusMode = focusMode;
        }
        virtual void SetFocusValue(UShort focusValue)
        { 
            mFocusValue = focusValue;
        }
        virtual void SetWhiteBalance(Byte whiteBalance)
        { 
            mWhiteBalance = whiteBalance;
        }
        virtual void SetImagingMode(Byte imagingMode)
        {
            mImagingMode = imagingMode;
        }
        virtual void SetExposureMode(Byte exposureMode)
        {
            mExposureMode = exposureMode;
        }
        virtual void SetMeteringMode(Byte meteringMode)
        {
            mMeteringMode = meteringMode;
        }
        virtual void SetShutterSpeed(UShort shutterSpeed)
        { 
            mShutterSpeed = shutterSpeed;
        }
        virtual void SetAperture(UShort aperture)
        { 
            mAperture = aperture;
        }
        virtual void SetLightSensitivity(Byte lightSensitivity)
        { 
            mLightSensitivity = lightSensitivity;
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
        Byte mSensorState;              ///<  Optional; Current State;  0=Active;  1=Standby; 2=Off;
        Byte mZoomMode;                 ///<  Optional; Zoom Mode; 0=Mixed, 1=AnalogOnly, 2=DigitalOnly, 3=Off
        UShort mZoomLevel;              ///<  Optional; Percent scaled integer between 0.0 and 100.0
        UInt mFocalLength;              ///<  Optional; Effective focal length scaled Integer between 0.0 and 2.0
        UInt mHorizontalFieldOfView;    ///<  Optional; Radians; scaled integer between -pi and pi
        UInt mVerticalFieldOfView;      ///<  Optional; Radians; scaled integer between -pi and pi
        Byte mFocusMode;                ///<  Optional; Suppoerted Focus Modes; Bit Mask: 0=AutoFocus, 1=ManualFocus
        UShort mFocusValue;             ///<  Optional; Percent scaled integer between 0.0 and 100.0
        Byte mWhiteBalance;             ///<  Optional; White Balance 0=Auto, 1=Daylight, 2=Cloudy, 3=Shade, 4=Tungsten, 5=Flurescent, 6=Flash
        Byte mImagingMode;              ///<  Optional; Image mode 0=Color, 1=Grayscale, 2=Infrared, 3=Lowlight
        Byte mExposureMode;             ///<  Optional; Exposure mode 0=Auto, 1=Manual, 2=ShutterPriority, 3=AperturePriority
        Byte mMeteringMode;             ///<  Optional; Exposure metering 0=Auto, 1=CenterWeighted, 2=Spot
        UShort mShutterSpeed;           ///<  Optional; Shutter Speed as a scaled integer between 0.0 and 60.0
        UShort mAperture;               ///<  Optional; f-stop value as a scaled integer between 0.1 and 128
        Byte mLightSensitivity;         ///<  Optional; ISO Film Speed equivalent; 0=Auto, 1=ISO 100, 2=ISO 200, 3=ISO 400, 4=ISO 800, 5=ISO 1600, 6=ISO 3200
        Byte mImageStabilization;       ///<  Optional; Boolean value 0 = Does Not Supports ImageStabilization; 1 = Supports ImageStabilization;
    };
}

#endif
/*  End of File */
