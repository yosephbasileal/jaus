////////////////////////////////////////////////////////////////////////////////////
///
///  \file StillImageSensorCapabilities.h
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
#ifndef __JAUS_ENVIRONMENT_SENSING_STILL_IMAGE_SENSOR__CAPABILITIES__H
#define __JAUS_ENVIRONMENT_SENSING_STILL_IMAGE_SENSOR__CAPABILITIES__H

#include "jaus/core/Types.h"
#include "jaus/core/Message.h"
#include "jaus/environment/EnvironmentCodes.h"

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class StillImageSensorCapabilities
    ///   \brief Data structure representing a single Still Image Sensor Capability
    ///   use within Report Still Image Capabilities message.  
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_ENVIRONMENT_DLL StillImageSensorCapabilities
    {
    public:
        class JAUS_ENVIRONMENT_DLL PresenceVector : public JAUS::PresenceVector
        {
        public:
            static const Byte SupportedFrameSizes   = 0x0001;
            static const Byte SupportedImageFormats  = 0x0002;
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
        class JAUS_ENVIRONMENT_DLL SupportedImageFormatsMask
        {
        public:
            static const UInt JPEG = 0x00000001;
            static const UInt GIF  = 0x00000002;
            static const UInt PNG  = 0x00000004;
            static const UInt BMP  = 0x00000008;
            static const UInt TIFF = 0x00000010;
            static const UInt PPM  = 0x00000020;
            static const UInt PGM  = 0x00000040;
            static const UInt PNM  = 0x00000080;
            static const UInt NEF  = 0x00000100;
            static const UInt CR2  = 0x00000200;
            static const UInt DNG  = 0x00000400;
        };
        StillImageSensorCapabilities(Byte presenceVector = 0 , UShort SensorId = -1);
        ~StillImageSensorCapabilities() {}
        virtual UShort GetPresenceVector() const { return mPresenceVector; }
        virtual UInt GetPresenceVectorSize() const { return BYTE_SIZE; }
        virtual UShort GetSensorId() const { return mSensorId; }
        virtual UInt GetSize() const;
        virtual UInt GetSupportedFrameSizes() const {return mSupportedFrameSizes; }
        virtual UShort GetSupportedImageFormats() const {return mSupportedImageFormats; }
        virtual void SetSensorId(UShort sensorId) 
        { 
            mSensorId = sensorId; 
        }
        virtual void SetSupportedFrameSizes(UInt supportedFrameSizes) 
        { 
            mSupportedFrameSizes = supportedFrameSizes;
        }
        virtual void SetSupportedImageFormats(UShort supportedImageFormats) 
        { 
            mSupportedImageFormats = supportedImageFormats;
        }
        virtual void PrintSensorFields() const;
        virtual int ReadMessageBody(const Packet& packet);
        virtual int WriteMessageBody(Packet& packet) const;
    private:
        Byte mPresenceVector;           ///<  Presence Vector. Required; Required
        UShort mSensorId;               ///<  SensorID value of "0" is invalid; Required
        UInt mSupportedFrameSizes;       ///<  Optional; Bit Mask see standard for values
        UShort mSupportedImageFormats;  ///<  Optional; Bit Mask see standard for values
    };
}

#endif
/*  End of File */
