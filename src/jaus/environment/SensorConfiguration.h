////////////////////////////////////////////////////////////////////////////////////
///
///  \file SensorConfiguration.h
///  \brief Data structure representing a Sensor Data configuration.
///
///  <br>Author(s): Daniel Barber, Jonathan Harris
///  <br>Created: 25 March 2013
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
#ifndef __JAUS_ENVIRONMENT_SENSING_SENSOR_CONFIGURATION__H
#define __JAUS_ENVIRONMENT_SENSING_SENSOR_CONFIGURATION__H

#include "jaus/core/Types.h"
#include "jaus/core/Message.h"
#include "jaus/environment/EnvironmentCodes.h"

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class SensorConfiguration
    ///   \brief Data structure representing a single Sensor configuration
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_ENVIRONMENT_DLL SensorConfiguration
    {
    public:
        class JAUS_ENVIRONMENT_DLL SensorError{
        public:
            
            SensorError(Byte errorCode = 0, std::string errorMessage = "") : 
            mErrorCode(errorCode), mErrorMessage(errorMessage) 
            {
                mSize = BYTE_SIZE + USHORT_SIZE + mErrorMessage.length();
            }
            virtual UInt GetSize() const { return mSize; }
            virtual std::string GetMessage() const { return mErrorMessage; }
            virtual Byte GetMessageLength() const { return mErrorMessage.length(); }
            virtual Byte GetSensorErrorCode() const { return mErrorCode; }
            virtual void SetErrorCode(Byte errorCode) { mErrorCode = errorCode; }
            virtual void SetMessage(std::string errorMessage) { 
                mErrorMessage = errorMessage; 
                mSize = BYTE_SIZE + USHORT_SIZE + mErrorMessage.length();
            }
        protected:
            UInt mSize;
            Byte mErrorCode;
            std::string mErrorMessage;
        };
        SensorConfiguration(UShort sensorId = -1, Byte variant =0) : mSensorId(sensorId), mVariant(variant) {}
        ~SensorConfiguration() {}
        virtual UShort GetSensorId() const { return mSensorId; }
        virtual SensorError GetSensorError() const { return mSensorError; }
        virtual Byte GetSensorConfigurationVariant() const { return mVariant; }
        virtual UInt GetSize() const 
        { 
            UInt size;
            size = (BYTE_SIZE * 2) + USHORT_SIZE;
            if(mVariant !=0)
            {
                size += mSensorError.GetSize();
            }
            return size;
        }
        virtual void SetSensorId(UShort sensorId) 
        { 
            mSensorId = sensorId; 
        }
        virtual void SetError(Byte sensorVaraint, Byte errorCode, std::string errorMessage) 
        { 
            mVariant = sensorVaraint;
            mSensorError = SensorError(errorCode,errorMessage); 
        }
        virtual void PrintSensorFields() const;
        virtual int ReadMessageBody(const Packet& packet); 
        virtual int WriteMessageBody(Packet& packet) const;
    private:
        Byte mVariant;              ///<  Variant 0=Success; 1=RangeError; 2=VisualError; 3=DigitalVideoError; 4=AnalogError; 5=StillImage
        UShort mSensorId;           ///<  SensorID value of "0" is invalid; Required
        SensorError mSensorError;
    };
}

#endif
/*  End of File */
