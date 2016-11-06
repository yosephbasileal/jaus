////////////////////////////////////////////////////////////////////////////////////
///
///  \file DigitalVideoSensorConfiguration.h
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
#ifndef __JAUS_ENVIRONMENT_SENSING_DIGITAL_VIDEO_SENSOR__CONFIGURATION__H
#define __JAUS_ENVIRONMENT_SENSING_DIGITAL_VIDEO_SENSOR__CONFIGURATION__H

#include "jaus/core/Types.h"
#include "jaus/core/Message.h"
#include "jaus/environment/EnvironmentCodes.h"

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class DigitalVideoSensorConfiguration
    ///   \brief Data structure representing a single Digital Video Sensor Capability
    ///   use within Report Digital Video Sensor Capabilities message.  
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_ENVIRONMENT_DLL DigitalVideoSensorConfiguration
    {
    public:
        class JAUS_ENVIRONMENT_DLL PresenceVector : public JAUS::PresenceVector
        {
        public:
            static const Byte MinimumBitRate    = 0x0001;
            static const Byte MaximumBitRate    = 0x0002;
            static const Byte FrameRate         = 0x0004;
            static const Byte FrameSize         = 0x0010;
            static const Byte DigitalFormat     = 0x0020;
        };
        enum JAUS_ENVIRONMENT_DLL FrameSize
        {
            sqcif_128x96      = 0,
            qcif_176x144      = 1,
            cif_352x288       = 2,
            cif4_704x576      = 3,
            cif16_1408x1152   = 4,
            qqvga_160x120     = 5,
            qvga_320x240      = 6,
            vga_640x480       = 7,
            svga_800x600      = 8,
            xga_1024x768      = 9,
            uxga_1600x1200    = 10,
            qxga_2048x1536    = 11,
            sxga_1280x1024    = 12,
            qsxga_2560x2048   = 13,
            hsxga_5120x4096   = 14,
            wvga_852x480      = 15,
            wxga_1366x768     = 16,
            wsxga_1600x1024   = 17,
            wuxga_1920x1200   = 18,
            woxga_2560x1600   = 19,
            wqsxga_3200x2048  = 20,
            wquxga_3840x2400  = 21,
            whsxga_6400x4096  = 22,
            whuxga_7680x4800  = 23,
            cga_320x200       = 24,
            ega_640x350       = 25,
            hd480_852x480     = 26,
            hd720_1280x720    = 27,
            hd1080_1920x1280  = 28
        };
        enum JAUS_ENVIRONMENT_DLL DigitalFormat
        {
            AVI           = 0,
            MJPEG         = 1,
            MPEG_2        = 2,
            H_263         = 3,
            H_263Plus     = 4,
            MPEG_4_Visual = 5,
            MPEG_4_AVC    = 6
        };
        DigitalVideoSensorConfiguration(Byte presenceVector = 0 , UShort SensorID = -1);
        ~DigitalVideoSensorConfiguration() {}
        virtual Byte GetPresenceVector() const { return mPresenceVector; }
        virtual UInt GetPresenceVectorSize() const { return BYTE_SIZE; }
        virtual UShort GetSensorId() const { return mSensorId; }
        virtual UInt GetSize() const {return mSize; }
        virtual UShort GetMinimumBitRate() const {return mMinimumBitRate; }
        virtual UShort GetMaximumBitRate() const {return mMaximumBitRate; }
        virtual Byte GetFrameRate() const {return mFrameRate; }
        virtual Byte GetFrameSize() const {return mFrameSize; }
        virtual Byte GetDigitalFormat() const {return mDigitalFormat; }
        virtual void SetSensorId(UShort sensorId) 
        { 
            mSensorId = sensorId; 
        }
        virtual void SetMinimumBitRate(UShort minimumBitRate) 
        { 
            if((mPresenceVector & PresenceVector::MinimumBitRate) > 0)
            {
                mSize += USHORT_SIZE;
                mPresenceVector &= PresenceVector::MinimumBitRate;
            }
            mMinimumBitRate = minimumBitRate;
        }
        virtual void SetMaximumBitRate(UShort maximumBitRate) 
        { 
            if((mPresenceVector & PresenceVector::MaximumBitRate) > 0)
            {
                mSize += USHORT_SIZE;
                mPresenceVector &= PresenceVector::MaximumBitRate;
            }
            mMaximumBitRate = maximumBitRate;
        }
        virtual void SetFrameRate(Byte frameRate) 
        { 
            if((mPresenceVector & PresenceVector::FrameRate) > 0)
            {
                mSize += BYTE_SIZE;
                mPresenceVector &= PresenceVector::FrameRate;
            }
            mFrameRate = frameRate;
        }
        virtual void SetFrameSize(UInt frameSize) 
        { 
            if((mPresenceVector & PresenceVector::FrameSize) > 0)
            {
                mSize += UINT_SIZE;
                mPresenceVector &= PresenceVector::FrameSize;
            }
            mFrameSize = frameSize;
        }
        virtual void SetDigitalFormat(Byte digitalFormat) 
        { 
            if((mPresenceVector & PresenceVector::DigitalFormat) > 0)
            {
                mSize += BYTE_SIZE;
                mPresenceVector &= PresenceVector::DigitalFormat;
            }
            mDigitalFormat = digitalFormat;
        }
        virtual void PrintSensorFields() const;
        virtual int ReadMessageBody(const Packet& packet);
        virtual int WriteMessageBody(Packet& packet) const;
    private:
        UInt mSize;                     ///<  The size in bytes of this record
        Byte mPresenceVector;           ///<  Presence Vector. Required; Required
        UShort mSensorId;               ///<  SensorID value of "0" is invalid; Required
        UShort mMinimumBitRate;         ///<  Optional; kilobits per second (default = 200 kbps)
        UShort mMaximumBitRate;         ///<  Optional; kilobits per second (default = 200 kbps)
        Byte mFrameRate;                ///<  Optional; frames per second (default = 25 fps)
        Byte mFrameSize;               ///<  Optional; Frame size enum: see standard for values
        Byte mDigitalFormat;           ///<  Optional; enum: 0=AVI, 1=MJPEG, 2=MPEG-2(H.262), 3=H.263, 4=H.263+, 5=MPEG-4 Visual, 6=MPEG-4 AVC 
    };
}

#endif
/*  End of File */
