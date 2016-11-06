////////////////////////////////////////////////////////////////////////////////////
///
///  \file AnalogVideoSensorCapabilities.h
///  \brief Data structure representing a Analog Video Sensor Capabilities Data Record.
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
#ifndef __JAUS_ENVIRONMENT_SENSING_ANALOG_VIDEO_SENSOR__CAPABILITIES__H
#define __JAUS_ENVIRONMENT_SENSING_ANALOG_VIDEO_SENSOR__CAPABILITIES__H

#include "jaus/core/Types.h"
#include "jaus/core/Message.h"
#include "jaus/environment/EnvironmentCodes.h"

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class AnalogVideoSensorCapabilities
    ///   \brief Data structure representing a single Analog Video Sensor Capability
    ///   use within Report Digital Video Sensor Capabilities message.  
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_ENVIRONMENT_DLL AnalogVideoSensorCapabilities
    {
    public:
        class JAUS_ENVIRONMENT_DLL SupportedAnalogFormatsMask
        {
        public:
            static const Byte NTSCM     = 0x01;
            static const Byte NTSCj     = 0x02;
            static const Byte PALN      = 0x04;
            static const Byte PALM      = 0x08;
            static const Byte SECAML    = 0x10;
            static const Byte SECAMBG   = 0x20;
        };
        AnalogVideoSensorCapabilities(UShort sensorID = -1, Byte supportedAnalogFormats = 0);
        ~AnalogVideoSensorCapabilities() {}
        virtual UShort GetSensorId() const { return mSensorId; }
        virtual Byte GetSupportedAnalogFormats() const {return mSupportedAnalogFormats; }
        virtual void SetSensorId(UShort sensorId) { mSensorId = sensorId; }
        virtual void SetSupportedAnalogFormats(Byte supportedAnalogFormats) 
        { 
            mSupportedAnalogFormats = supportedAnalogFormats;
        }
        virtual void PrintSensorFields() const;
        virtual int ReadMessageBody(const Packet& packet);
        virtual int WriteMessageBody(Packet& packet) const;
    private:
        UInt mSize;
        UShort mSensorId;               ///<  SensorID value of "0" is invalid; Required
        Byte mSupportedAnalogFormats;   ///<  Required; Bit Mask: 0=NTSCM, 1=NTSCJ, 2=PALN, 3=PALM, 4=SECAML, 5=SECAMBG,
    };
}

#endif
/*  End of File */
