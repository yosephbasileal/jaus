////////////////////////////////////////////////////////////////////////////////////
///
///  \file Encoders.h
///  \brief Simple data structure for doing math using encoders on a vehicles
///         motors to calculate speed and rotations.
///   
///  <br>Author(s): Daniel Barber
///  <br>Created: 3/9/2013
///  <br>Copyright (c) 2013
///  <br>Applied Cognition and Training in Immersive Virtual Environments
///  <br>(ACTIVE) Laboratory
///  <br>Institute for Simulation and Training (IST)
///  <br>University of Central Florida (UCF)
///  <br>All rights reserved.
///  <br>Email: dbarber@ist.ucf.edu
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
#ifndef __CXUTILS_ENCODERS_MATH__H
#define __CXUTILS_ENCODERS_MATH__H

#include <map>
#include "cxutils/math/CxMath.h"

namespace CxUtils
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class Motor
    ///   \brief Data structure for storing Quadrature encoder information used in
    ///          closed loop control systems (e.g. motor control).
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class CX_UTILS_DLL Encoders
    {
    public:
        /** Stores actual encoder values for a motor. */
        class CX_UTILS_DLL Motor
        {
        public:
            typedef std::map<unsigned int, Motor> Map;
            Motor();
            Motor(const Motor& encoder);
            virtual ~Motor();
            virtual void Clear();
            double ToMeters() const { return ToMeters(mShaftRadius, mValuesPerRevolution); }
            double ToMeters(const double radius,                                         // Radius/dist from shaft 
                            const long long int ticksPerRevolution) const;               // Motor count for 1 full revolution of motor shaft.
            double ToLinearVelocity() const { return ToLinearVelocity(mShaftRadius, mValuesPerRevolution); }
            double ToLinearVelocity(const double radius,                                 // Radius/dist from shaft 
                                    const long long int ticksPerRevolution) const;       // Motor count for 1 full revolution of motor shaft.
            double ToRotationalVelocity() const { return ToRotationalVelocity(mValuesPerRevolution); }
            double ToRotationalVelocity(const long long int ticksPerRevolution) const;   // Motor count for 1 full revolution of motor shaft.
            Motor operator+(const Motor& encoder) const;
            Motor& operator+=(const Motor& encoder);
            Motor& operator=(const Motor& encoder);
            bool mRelativeFlag;             ///<  True if relative encoder values, false for absolute position (default is true).
            int mCount;                     ///<  Motor ticks/count.
            int mValuesPerRevolution;       ///<  Number of encoder values in 1 revolution of the motor shaft.
            double mShaftRadius;            ///<  Motor shaft radius (or wheel radius).
            Time::Stamp mSamplePeriodMs;    ///<  Time change in milliseconds representing sampling period (for speed calculations).
            Time mTimeStamp;                ///<  Time of reading (UTC).
        };
        Encoders();
        Encoders(const Encoders& encoder);
        virtual ~Encoders();
        // Returns true if data has been added, but Update hasn't been called.
        bool IsDirty() const { return mDirtyFlag; }
        // Update relative encoder values for left side of platform.
        void UpdateLeftEncoder(const unsigned int encoderID,
            const Motor& encoderData);
        // Update relative encoder values for right side of platform.
        void UpdateRightEncoder(const unsigned int encoderID,
            const Motor& encoderData);
        // Set platform width.
        void SetWidth(const double width) { mWidth = fabs(width); }
        // Clears all data.
        void Reset();
        // Update calculations using current data.
        bool Update();
        // Get width value being used.
        double GetWidth() const { return mWidth; }
        // Gets the linear velocity (m/s) based previous call to Update method.
        double GetLinearVelocity() const;
        // Gets the rotational velocity (rad/s) based previous call to Update method.
        double GetRotationalVelocity() const;
        // Gets the change in meters for left and right side of vehicle.
        void GetPositionChange(double& left, double& right) { left = mLeftSideDist; right = mRightSideDist; }
        // Gets the number of times Update has been called.
        unsigned int GetUpdateCount() const { return mUpdateCounter; }
        // Gets the number of encoders added to system.
        unsigned int GetEncoderCount() const { return (unsigned int)(mLeftSide.size() + mRightSide.size()); }
        // Sets equal to.
        Encoders& operator=(const Encoders& encoder);
    protected:
        bool mDirtyFlag;                ///<  If true, new data has been added but not used in calculations yet.
        Motor::Map mLeftSide;           ///<  Encoders on the left side of the body.
        Motor::Map mRightSide;          ///<  Encoders on the right side of the body.
        double mWidth;                  ///<  Width of rigid body in meters (between left and right side).
        unsigned int mUpdateCounter;    ///<  Keeps track of number of calls to update method.
        double mLeftSideDist;           ///<  Left side distance change in meters.
        double mRightSideDist;          ///<  Right side distance change in meters.
        double mLeftSideVelocity;       ///<  Left side velocity in m/s.
        double mRightSideVelocity;      ///<  Right side velocity in m/s.
        Time mUpdateTime;               ///<  The last time update was called.
        double mSamplePeriodSeconds;    ///<  Sampling period in ms.
    };
};

#endif  //__CXUTILS_ENCODERS_MATH__H

/* End of File */

