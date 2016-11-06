////////////////////////////////////////////////////////////////////////////////////
///
///  \file RangeSensorCapabilities.h
///  \brief Data structure representing a Range Sensor Capabilities Data Record.
///
///  <br>Author(s): Daniel Barber, Jonathan Harris
///  <br>Created: 13 March 2013
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
#ifndef __JAUS_ENVIRONMENT_SENSING_RANGE_SENSOR__CAPABILITIES__H
#define __JAUS_ENVIRONMENT_SENSING_RANGE_SENSOR__CAPABILITIES__H

#include "jaus/environment/range/Range.h"
#include "jaus/core/Types.h"


namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class RangeSensorCapabilities
    ///   \brief Data structure representing a single range Capability
    ///   use within Report Range Sensor Capabilities message.  
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_ENVIRONMENT_DLL RangeSensorCapabilities
    {
    public:
        class JAUS_ENVIRONMENT_DLL PresenceVector : public JAUS::PresenceVector
        {
        public:
            static const UShort SupportedStates                         = 0x0001;
            static const UShort MinimumHorizontalFieldOfViewStartAngle  = 0x0002;
            static const UShort MaximumHorizontalFieldOfViewStopAngle   = 0x0004;
            static const UShort MinimumVerticalFieldOfViewStartAngle    = 0x0008;
            static const UShort MaximumVerticalFieldOfViewStopAngle     = 0x0010;
            static const UShort MinimumUpdateRate                       = 0x0020;
            static const UShort MaximumUpdateRate                       = 0x0040;
            static const UShort MinimumRange                            = 0x0080;
            static const UShort MaximumRange                            = 0x0100; 
            static const UShort SupportedCompression                    = 0x0200;
            static const UShort CoordinateTransformationSupported       = 0x0400;
        };
        class JAUS_ENVIRONMENT_DLL SupportedStatesMask
        {
        public:
            static const Byte Active    = 0x01;
            static const Byte Standby   = 0x02;
            static const Byte Off       = 0x04;
        };
        class JAUS_ENVIRONMENT_DLL SupportedCompressionMask
        {
        public:
            static const Byte NoCompression = 0x01;
            static const Byte DEFLATE       = 0x02;
            static const Byte bzip2         = 0x04;
            static const Byte LZMA          = 0x08;
        };
        RangeSensorCapabilities(UShort presenceVector = 0 , UShort SensorId = 0, std::string SensorName  = "");
        ~RangeSensorCapabilities() {}
        virtual UShort GetPresenceVector() const { return mPresenceVector; }
        virtual UInt GetPresenceVectorSize() const { return USHORT_SIZE; }
        virtual UShort GetSensorId() const { return mSensorId; }
        virtual std::string GetSensorName() const { return mSensorName; }
        virtual Byte GetSuppoertedStates() const { return mSupportedStates; }
        virtual UInt GetMinimumHorizontalFieldOfViewStartAngle() const  
        {
            return mMinimumHorizontalFieldOfViewStartAngle;
        }
        virtual UInt GetMaximumHorizontalFieldOfViewStopAngle()const  
        {
            return mMaximumHorizontalFieldOfViewStopAngle;
        }
        virtual UInt GetMinimumVerticalFieldOfViewStartAngle() const  
        {
            return mMinimumVerticalFieldOfViewStartAngle;
        }
        virtual UInt GetMaximumVerticalFieldOfViewStopAngle()const  
        {
            return mMaximumVerticalFieldOfViewStopAngle;
        }
        virtual UShort GetMinimumUpdateRate() const { return mMinimumUpdateRate; }
        virtual UShort GetMaximumUpdateRate() const { return mMaximumUpdateRate; }
        virtual UShort GetMinimunRange() const { return mMinimumRange; }
        virtual UShort GetMaximumRange() const { return mMaximumRange; }
        virtual UInt GetSize() const {return mSize; }
        virtual Byte GetSupportedCompression( ) const { return mSupportedCompression; }
        virtual Byte GetTransformationSupported() const { return mCoordinateTransformationSupported; }
        virtual void SetSensorId(UShort sensorId) 
        { 
            mSensorId = sensorId; 
        }
        virtual void SetSensorName(std::string sensorName) 
        {
            mSize += sensorName.length();
            mSensorName = sensorName; 
        }
        virtual void SetSuppoertedStates(Byte suppoertedStates)
        {
            mSupportedStates = suppoertedStates;
        }
        virtual void SetMinimumHorizontalFieldOfViewStartAngle(UInt minimumHorizontalFieldOfViewStartAngle) 
        {
            mSize += UINT_SIZE;
            mPresenceVector &= PresenceVector::MinimumHorizontalFieldOfViewStartAngle;
            mMinimumHorizontalFieldOfViewStartAngle = minimumHorizontalFieldOfViewStartAngle; 
        }
        virtual void SetMaximumHorizontalFieldOfViewStopAngle(UInt maximumHorizontalFieldOfViewStopAngle) 
        {
            mSize += UINT_SIZE;
            mPresenceVector &= PresenceVector::MaximumHorizontalFieldOfViewStopAngle;
            mMaximumHorizontalFieldOfViewStopAngle = maximumHorizontalFieldOfViewStopAngle;
        }
        virtual void SetMinimumVerticalFieldOfViewStartAngle(UInt minimumVerticalFieldOfViewStartAngle) 
        {
            mSize += UINT_SIZE;
            mPresenceVector &= PresenceVector::MinimumVerticalFieldOfViewStartAngle;
            mMinimumVerticalFieldOfViewStartAngle = minimumVerticalFieldOfViewStartAngle;
        }
        virtual void SetMaximumVerticalFieldOfViewStopAngle(UInt maximumVerticalFieldOfViewStopAngle) 
        {
            mSize += UINT_SIZE;
            mPresenceVector &= PresenceVector::MaximumVerticalFieldOfViewStopAngle;
            mMaximumVerticalFieldOfViewStopAngle = maximumVerticalFieldOfViewStopAngle;
        }
        virtual void SetMinimumUpdateRate(UShort minimumUpdateRate) 
        { 
            mSize += USHORT_SIZE;
            mPresenceVector &= PresenceVector::MinimumUpdateRate;
            mMinimumUpdateRate = minimumUpdateRate; 
        }
        virtual void SetMaximumUpdateRate(UShort maximumUpdateRate) 
        {
            mSize += USHORT_SIZE;
            mPresenceVector &= PresenceVector::MaximumUpdateRate;
            mMaximumUpdateRate = maximumUpdateRate; 
        }
        virtual void SetMinimumRange(UInt MinimumRange)
        {
            mSize += UINT_SIZE;
            mPresenceVector &= PresenceVector::MinimumRange;
            mMinimumRange = MinimumRange; 
        }
        virtual void SetMaximumRange(UInt MaximumRange)
        {
            mSize += UINT_SIZE;
            mPresenceVector &= PresenceVector::MaximumRange;
            mMaximumRange = MaximumRange; 
        }
        virtual void SetSupportedCompression(Byte chosenAlgorithm) 
        {
            mSize += BYTE_SIZE;
            mPresenceVector &= PresenceVector::SupportedCompression;
            mSupportedCompression = chosenAlgorithm; 
        }
        virtual void SetCoordinateTransformationSupported(Byte coordinateTransformationSupported) 
        {
            mSize += BYTE_SIZE;
            mPresenceVector &= PresenceVector::CoordinateTransformationSupported;
            mCoordinateTransformationSupported = coordinateTransformationSupported;
        }
        virtual void PrintSensorFields() const;
        virtual int ReadMessageBody(const Packet& packet);
        virtual int WriteMessageBody(Packet& packet) const;
    private:
        UShort mPresenceVector;         ///<  Presence Vector. Required; Required
        UShort mSensorId;               ///<  SensorID value of "0" is invalid; Required
        std::string mSensorName;        ///<  Variable Length String; Count=byte
        Byte mSupportedStates;           ///<  Optional; Supported state; Bit Mask: 0=Active;  1=Standby; 2=Off;
        UInt mMinimumHorizontalFieldOfViewStartAngle;  ///<  Optional; Radians scaled between -pi and pi about the sensor's fixed +Z axis
        UInt mMaximumHorizontalFieldOfViewStopAngle;   ///<  Optional; Radians scaled between -pi and pi about the sensor's fixed +Z axis
        UInt mMinimumVerticalFieldOfViewStartAngle;    ///<  Optional; Radians scaled between -pi and pi about the sensor's fixed +Y axis
        UInt mMaximumVerticalFieldOfViewStopAngle;     ///<  Optional; Radians scaled between -pi and pi about the sensor's fixed +Y axis
        UShort mMinimumUpdateRate;      ///<  Optional; Sacled between 0 and 1,000
        UShort mMaximumUpdateRate;      ///<  Optional; Sacled between 0 and 1,000
        UShort mMinimumRange;           ///<  Optional; Sacled between 0 and 1,000,000
        UShort mMaximumRange;           ///<  Optional; Sacled between 0 and 1,000,000
        Byte  mSupportedCompression;    ///<  Optional; Bit0=No Compression; Bit1=DEFLATE; Bit2=bzip2; Bit3=LZMA;
        Byte  mCoordinateTransformationSupported;       ///<  Optional; Boolean value 0=False; 1=True
        UInt mSize;                     ///<The size in bytes of this record
    };
}

#endif
/*  End of File */
