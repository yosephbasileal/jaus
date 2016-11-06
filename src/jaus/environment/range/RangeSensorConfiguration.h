////////////////////////////////////////////////////////////////////////////////////
///
///  \file RangeSensorConfiguration.h
///  \brief Data structure representing a Range Sensor Capabilities Data Record.
///
///  <br>Author(s): Daniel Barber, Jonathan Harris
///  <br>Created: 18 March 2013
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
#ifndef __JAUS_ENVIRONMENT_SENSING_RANGE_SENSOR__CONFIGURATION__H
#define __JAUS_ENVIRONMENT_SENSING_RANGE_SENSOR__CONFIGURATION__H

#include "jaus/environment/range/Range.h"
#include "jaus/core/Types.h"


namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class RangeSensorConfiguration
    ///   \brief Data structure representing a single range sensor configuration
    ///   use within Report Range Sensor Configuration message.
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_ENVIRONMENT_DLL RangeSensorConfiguration
    {
    public:
        class JAUS_ENVIRONMENT_DLL PresenceVector : public JAUS::PresenceVector
        {
        public:
            static const UShort HorizontalFieldOfViewStartAngle         = 0x0001;
            static const UShort HorizontalFieldOfViewStopAngle          = 0x0002;
            static const UShort VerticalFieldOfViewStartAngle           = 0x0004;
            static const UShort VerticalFieldOfViewStopAngle            = 0x0008;
            static const UShort UpdateRate                              = 0x0010;
            static const UShort MinimumRange                            = 0x0020;
            static const UShort MaximumRange                            = 0x0040;
            static const UShort SensorState                             = 0x0080;
        };
        enum JAUS_ENVIRONMENT_DLL SensorState
        {
            Active = 0, Standby = 1, Off =2
        };
        RangeSensorConfiguration(UShort presenceVector = 0 , UShort sensorId = -1);
        ~RangeSensorConfiguration() {}
        virtual UInt GetSize() const;
        virtual UShort GetPresenceVector() const { return mPresenceVector; }
        virtual UInt GetPresenceVectorSize() const { return USHORT_SIZE; }
        virtual UShort GetSensorID() const { return mSensorId; }
        virtual UInt GetHorizontalFieldOfViewStartAngle() const  
        {
            return mHorizontalFieldOfViewStartAngle;
        }
        virtual UInt GetHorizontalFieldOfViewStopAngle() const  
        {
            return mHorizontalFieldOfViewStopAngle;
        }
        virtual UInt GetVerticalFieldOfViewStartAngle() const  
        {
            return mVerticalFieldOfViewStartAngle;
        }
        virtual UInt GetVerticalFieldOfViewStopAngle() const  
        {
            return mVerticalFieldOfViewStopAngle;
        }
        virtual UShort GetUpdateRate() const { return mUpdateRate; }        
        virtual UInt GetMinimumRange() const { return mMinimumRange; }
        virtual UInt GetMaximumRange() const { return mMaximumRange; }
        virtual Byte GetSensorState( ) const { return mSensorState; }
        virtual void SetSensorId(UShort sensorId) 
        {
            mSensorId = sensorId; 
        }
        virtual void SetHorizontalFieldOfViewStartAngle(UInt horizontalFieldOfViewStartAngle)
        {
            mHorizontalFieldOfViewStartAngle = horizontalFieldOfViewStartAngle; 
        }
        virtual void SetHorizontalFieldOfViewStopAngle(UInt horizontalFieldOfViewStopAngle)
        {
            mHorizontalFieldOfViewStopAngle = horizontalFieldOfViewStopAngle; 
        }
        virtual void SetVerticalFieldOfViewStartAngle(UInt verticalFieldOfViewStartAngle)
        {
            mVerticalFieldOfViewStartAngle = verticalFieldOfViewStartAngle; 
        }
        virtual void SetVerticalFieldOfViewStopAngle(UInt verticalFieldOfViewStopAngle)
        {
            mVerticalFieldOfViewStopAngle = verticalFieldOfViewStopAngle; 
        }
        virtual void SetUpdateRate(UShort updateRate)
        {
            mUpdateRate = updateRate;
        }
        virtual void SetMinimumRange(UInt minimumRange)
        {
            mMinimumRange = minimumRange; 
        }
        virtual void SetMaximumRange(UInt maximumRange)
        {
            mMaximumRange = mMaximumRange; 
        }
        virtual void SetSensorState(SensorState sensorState)
        {
            mSensorState = (Byte) sensorState; 
        }
        virtual void PrintSensorFields() const;
        virtual int ReadMessageBody(const Packet& packet);
        virtual int WriteMessageBody(Packet& packet) const;
    private:
        UShort mPresenceVector;                 ///<  Presence Vector. Required; Required
        UShort mSensorId;                       ///<  SensorID value of "0" is invalid; Required
        UInt mHorizontalFieldOfViewStartAngle;  ///<  Optional; Radians; Scaled between -pi and pi
        UInt mHorizontalFieldOfViewStopAngle;   ///<  Optional; Radians; Scaled between -pi and pi
        UInt mVerticalFieldOfViewStartAngle;    ///<  Optional; Radians; Scaled between -pi and pi
        UInt mVerticalFieldOfViewStopAngle;     ///<  Optional; Radians; Scaled between -pi and pi
        UShort mUpdateRate;                     ///<  Optional; data collection rate of the sensor in hertz; value between 0 and 1,000
        UInt mMinimumRange;                     ///<  Optional; minimum range of data points reported in meters; value between 0 and 1,000,000
        UInt mMaximumRange;                     ///<  Optional; maximum range of data points reported in meters; value between 0 and 1,000,000
        Byte mSensorState;                      ///<  Optional; 0 = Active; 1 Standby; 2 = Off;
    };
}

#endif
/*  End of File */
