////////////////////////////////////////////////////////////////////////////////////
///
///  \file StillImageData.h
///  \brief Data structure representing a Still Image Data Record.
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
#ifndef __JAUS_ENVIRONMENT_SENSING_STILL_IMAGE_DATA__H
#define __JAUS_ENVIRONMENT_SENSING_STILL_IMAGE_DATA__H

#include "jaus/core/Types.h"
#include "jaus/core/Message.h"
#include "jaus/environment/EnvironmentCodes.h"

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class StillImageData
    ///   \brief Data structure representing a single Still Image Data
    ///   use within Report Still Image Data message.  
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_ENVIRONMENT_DLL StillImageData
    {
    public:
        class JAUS_ENVIRONMENT_DLL PresenceVector : public JAUS::PresenceVector
        {
        public:
            static const Byte TimeStamp   = 0x0001;
        };
        enum JAUS_ENVIRONMENT_DLL CoordinateSystem
        {
            Native  = 0,
            Vehicle = 1,
        };
        enum JAUS_ENVIRONMENT_DLL ImageFrameFormat
        {
            JPEG = 0,
            GIF  = 1,
            PNG  = 2,
            BMP  = 3,
            TIFF = 4,
            PPM  = 5,
            PGM  = 6,
            PNM  = 7,
            NEF  = 8,
            CR2  = 9,
            DNG  = 10
        };
        StillImageData(Byte presenceVector = 0 , UShort SensorID = -1, Byte reportCoordinateSystem =0);
        ~StillImageData() { }
        virtual Byte GetPresenceVector() const { return mPresenceVector; }
        virtual UInt GetPresenceVectorSize() const { return BYTE_SIZE; }
        virtual UShort GetSensorId() const { return mSensorId; }
        virtual UInt GetSize() const;
        virtual Byte GetReportCoordinateSystem() const { return mReportCoordinateSystem; }
        virtual UInt GetTimeStamp() const { return mTimeStamp; }
        virtual Byte GetImageFrameFormat() const { return mImageFrameFormat; }
        virtual Packet& GetImageFrame() { return mFrameData; }
        virtual const Packet& GetImageFrame() const { return mFrameData; }
        virtual void SetSensorId(UShort sensorId) 
        { 
            mSensorId = sensorId; 
        }
        virtual void SetReportCoordinateSystem(Byte reportCoordinateSystem) 
        { 
            mReportCoordinateSystem = reportCoordinateSystem; 
        }
        virtual void SetTimeStamp(UInt timeStamp) 
        { 
            mTimeStamp = timeStamp;
        }
        virtual void SetImageFrameData(Packet data, Byte format) 
        { 
            mImageFrameFormat = format;
            mFrameData = data;
        }
        virtual void PrintSensorFields() const;
        virtual int ReadMessageBody(const Packet& packet);
        virtual int WriteMessageBody(Packet& packet) const;
    private:
        Byte mPresenceVector;           ///<  Presence Vector. Required; Required
        UShort mSensorId;               ///<  SensorID value of "0" is invalid; Required
        Byte mReportCoordinateSystem;   ///<  Enum: 0 = Native Coordinate System; 1 = Vehicle Coordinate System
        UInt mTimeStamp;                ///<  Optional; Bits 0-9: miliseconds; 10-15:Seconds; 16-21: minutes ; 22-26: hours; 27-31:days
        Byte mImageFrameFormat;         ///<  Required; 
        Packet mFrameData;              ///<  Byte array to hold the raw image data
    };
}

#endif
/*  End of File */
