////////////////////////////////////////////////////////////////////////////////////
///
///  \file DigitalVideoSensorCapabilities.h
///  \brief Data structure representing a Digital Video Sensor Capabilities Data Record.
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
#ifndef __JAUS_ENVIRONMENT_SENSING_DIGITAL_VIDEO_SENSOR_CAPABILITIES__H
#define __JAUS_ENVIRONMENT_SENSING_DIGITAL_VIDEO_SENSOR_CAPABILITIES__H

#include "jaus/core/Types.h"
#include "jaus/core/Message.h"
#include "jaus/environment/EnvironmentCodes.h"

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class DigitalVideoSensorCapabilities
    ///   \brief Data structure representing a single Digital Video Sensor Capability
    ///   use within Report Digital Video Sensor Capabilities message.  
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_ENVIRONMENT_DLL DigitalVideoSensorCapabilities
    {
    public:
        class JAUS_ENVIRONMENT_DLL PresenceVector : public JAUS::PresenceVector
        {
        public:
            static const Byte MinimumBitRate              = 0x0001;
            static const Byte MaximumBitRate              = 0x0002;
            static const Byte MinimumFrameRate            = 0x0004;
            static const Byte MaximumFrameRate            = 0x0008;
            static const Byte SupportedFrameSizes         = 0x0010;
            static const Byte SupportedDigitalFormats     = 0x0020;
        };
        class JAUS_ENVIRONMENT_DLL SupportedFrameSizesMask
        {
        public:
            static const UInt sqcif_128x96      = 0x00000001;
            static const UInt qcif_176x144      = 0x00000002;
            static const UInt cif_352x288       = 0x00000004;
            static const UInt cif4_704x576      = 0x00000008;
            static const UInt cif16_1408x1152   = 0x00000010;
            static const UInt qqvga_160x120     = 0x00000020;
            static const UInt qvga_320x240      = 0x00000040;
            static const UInt vga_640x480       = 0x00000080;
            static const UInt svga_800x600      = 0x00000100;
            static const UInt xga_1024x768      = 0x00000200;
            static const UInt uxga_1600x1200    = 0x00000400;
            static const UInt qxga_2048x1536    = 0x00000800;
            static const UInt sxga_1280x1024    = 0x00001000;
            static const UInt qsxga_2560x2048   = 0x00002000;
            static const UInt hsxga_5120x4096   = 0x00004000;
            static const UInt wvga_852x480      = 0x00008000;
            static const UInt wxga_1366x768     = 0x00010000;
            static const UInt wsxga_1600x1024   = 0x00020000;
            static const UInt wuxga_1920x1200   = 0x00040000;
            static const UInt woxga_2560x1600   = 0x00080000;
            static const UInt wqsxga_3200x2048  = 0x00100000;
            static const UInt wquxga_3840x2400  = 0x00200000;
            static const UInt whsxga_6400x4096  = 0x00400000;
            static const UInt whuxga_7680x4800  = 0x00800000;
            static const UInt cga_320x200       = 0x01000000;
            static const UInt ega_640x350       = 0x02000000;
            static const UInt hd480_852x480     = 0x04000000;
            static const UInt hd720_1280x720    = 0x08000000;
            static const UInt hd1080_1920x1280  = 0x10000000;
        };
        class JAUS_ENVIRONMENT_DLL SupportedDigitalFormatsMask
        {
        public:
            static const Byte AVI           = 0x0001;
            static const Byte MJPEG         = 0x0002;
            static const Byte MPEG_2        = 0x0004;
            static const Byte H_263         = 0x0008;
            static const Byte H_263Plus     = 0x0010;
            static const Byte MPEG_4_Visual = 0x0020;
            static const Byte MPEG_4_AVC    = 0x0040;
        };
        DigitalVideoSensorCapabilities(Byte presenceVector = 0 , UShort SensorId = -1);
        ~DigitalVideoSensorCapabilities() {}
        virtual Byte GetPresenceVector() const { return mPresenceVector; }
        virtual UInt GetPresenceVectorSize() const { return BYTE_SIZE; }
        virtual UShort GetSensorId() const { return mSensorId; }
        virtual UInt GetSize() const {return mSize; }
        virtual UShort GetMinimumBitRate() const {return mMinimumBitRate; }
        virtual UShort GetMaximumBitRate() const {return mMaximumBitRate; }
        virtual Byte GetMinimumFrameRate() const {return mMinimumFrameRate; }
        virtual Byte GetMaximumFrameRate() const {return mMaximumFrameRate; }
        virtual UInt GetSupportedFrameSizes() const {return mSupportedFrameSizes; }
        virtual Byte GetSupportedDigitalFormats() const {return mSupportedDigitalFormats; }
        virtual void SetPresenceVector(Byte presenceVector) 
        { 
            mPresenceVector = presenceVector; 
        }
        virtual void SetSesorId(UShort sensorId) 
        { 
            mSensorId = sensorId; 
        }
        virtual void SetMinimumBitRate(UShort minimumBitRate) 
        { 
            if((mPresenceVector & PresenceVector::MinimumBitRate) == 0)
            {
                mSize += USHORT_SIZE;
                mPresenceVector |= PresenceVector::MinimumBitRate;
            }
            mMinimumBitRate = minimumBitRate;
        }
        virtual void SetMaximumBitRate(UShort maximumBitRate) 
        { 
            if((mPresenceVector & PresenceVector::MaximumBitRate) == 0)
            {
                mSize += USHORT_SIZE;
                mPresenceVector |= PresenceVector::MaximumBitRate;
            }
            mMaximumBitRate = maximumBitRate;
        }
        virtual void SetMinimumFrameRate(Byte minimumFrameRate) 
        { 
            if((mPresenceVector & PresenceVector::MinimumFrameRate) == 0)
            {
                mSize += BYTE_SIZE;
                mPresenceVector |= PresenceVector::MinimumFrameRate;
            }
            mMinimumFrameRate = minimumFrameRate;
        }
        virtual void SetMaximumFrameRate(Byte maximumFrameRate) 
        { 
            if((mPresenceVector & PresenceVector::MaximumFrameRate) == 0)
            {
                mSize += BYTE_SIZE;
                mPresenceVector |= PresenceVector::MaximumFrameRate;
            }
            mMaximumFrameRate = maximumFrameRate;
        }
        virtual void SetSupportedFrameSizes(UInt supportedFrameSizes) 
        { 
            if((mPresenceVector & PresenceVector::SupportedFrameSizes) == 0)
            {
                mSize += UINT_SIZE;
                mPresenceVector |= PresenceVector::SupportedFrameSizes;
            }
            mSupportedFrameSizes = supportedFrameSizes;
        }
        virtual void SetSupportedDigitalFormats(UInt supportedDigitalFormats) 
        { 
            if((mPresenceVector & PresenceVector::SupportedDigitalFormats) == 0)
            {
                mSize += BYTE_SIZE;
                mPresenceVector |= PresenceVector::SupportedDigitalFormats;
            }
            mSupportedDigitalFormats = supportedDigitalFormats;
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
        Byte mMinimumFrameRate;         ///<  Optional; frames per second (default = 25 fps)
        Byte mMaximumFrameRate;         ///<  Optional; frames per second (default = 25 fps)
        UInt mSupportedFrameSizes;       ///<  Optional; Bit Mask see standard for values
        Byte mSupportedDigitalFormats;  ///<  Optional; Bit Mask 0=AVI, 1=MJPEG, 2=MPEG-2(H.262), 3=H.263, 4=H.263+, 5=MPEG-4 Visual, 6=MPEG-4 AVC 
    };
}

#endif
/*  End of File */
