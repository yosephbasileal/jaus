////////////////////////////////////////////////////////////////////////////////////
///
///  \file Range.h
///  \brief Data structure representing a Range Sensor Data Record.
///
///  <br>Author(s): Daniel Barber
///  <br>Created: 13 February 2012
///  <br>Copyright (c) 2012
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
#ifndef __JAUS_ENVIRONMENT_SENSING_RANGE_DATA_RECORD__H
#define __JAUS_ENVIRONMENT_SENSING_RANGE_DATA_RECORD__H

#include "jaus/core/Message.h"
#include "jaus/core/ScaledInteger.h"
#include "jaus/environment/EnvironmentCodes.h"

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class Range
    ///   \brief Data structure representing a single range data point record for
    ///   use within messages.  
    ///
    ///   Each Range object uses a Point3D to represent the range, with X 
    ///   equal to the range in meters, y the inclination in radians [-pi, pi], and
    ///   z the bearing [-pi, pi].
    ///
    ///   Data points are in the local coordinate frame, relative to the robot or
    ///   the sensor.
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_ENVIRONMENT_DLL Range
    {
    public:
        static const Byte NativeCoordinate = 0;    ///< Range point is in the native coordinate system of sensor (Default).
        static const Byte VehicleCoordinate = 1;   ///< Range point is in the coordinate system of the vehicle.
        typedef std::vector<Range> List;    ///<  List of range data points.
        ////////////////////////////////////////////////////////////////////////////////////
        ///
        ///   \class PresenceVector
        ///   \brief This class contains bit masks for bitwise operations on the
        ///          presence vector for this message.
        ///
        ////////////////////////////////////////////////////////////////////////////////////
        class JAUS_ENVIRONMENT_DLL PresenceVector : public JAUS::PresenceVector
        {
        public:
            static const UShort PointID                 = 0x0001;
            static const UShort RangeValidity           = 0x0002;
            static const UShort RangeErrorRMS           = 0x0004;
            static const UShort BearingValidity         = 0x0008;
            static const UShort BearingErrorRMS         = 0x0010;
            static const UShort InclinationValidity     = 0x0020;
            static const UShort InclinationErrorRMS     = 0x0040;
        };
        ////////////////////////////////////////////////////////////////////////////////////
        ///
        ///   \class Limits
        ///   \brief Contains constants for limit values of data members of class.
        ///
        ////////////////////////////////////////////////////////////////////////////////////
        class JAUS_ENVIRONMENT_DLL Limits : public JAUS::Limits
        {
        public:
            const static double MinRange;       ///<  Minium range 0, in meters.
            const static double MaxRange;       ///<  Maximum range 1,000,000 in meters.
            const static double MinRangeRMS;    ///<  Minimum range RMS 0 meters.
            const static double MaxRangeRMS;    ///<  Maximum range RMS 100,000 meters.
            const static double MinAngleRMS;    ///<  Minimum range RMS 0 radians.
            const static double MaxAngleRMS;    ///<  Maximum range RMS pi, radians.
            const static double MinAngle;       ///<  Min angle for bearing or inclination (radians).
            const static double MaxAngle;       ///<  Maximum angle for bearing or inclination (radians).
        };
        Range() : mID(0),
                  mPresenceVector(0),
                  mRangeValidFlag(true),
                  mBearingValidFlag(true),
                  mInclinationValidFlag(true),
                  mBearingErrorRMS(0.0),
                  mRangeErrorRMS(0.0),
                  mInclinationErrorRMS(0.0)
        {
        }
        Range(const Point3D& point) : mID(0),
                  mPresenceVector(0),
                  mRangeValidFlag(true),
                  mBearingValidFlag(true),
                  mInclinationValidFlag(true),
                  mBearingErrorRMS(0.0),
                  mRangeErrorRMS(0.0),
                  mInclinationErrorRMS(0.0),
                  mPoint(point)
        {
        }
        Range(const Range& data) { *this = data; }
        ~Range() {}
        /** Sets value and updates presence vector. */
        inline void SetID(const UInt id) 
        {
            mID = id; mPresenceVector |= PresenceVector::PointID;
        }
        /** Sets value and updates presence vector. */
        inline void SetRangeErrorRMS(const double rms)
        {
            mRangeErrorRMS = rms; mPresenceVector |= PresenceVector::RangeErrorRMS;
        }
        /** Sets value and updates presence vector. */
        inline void SetBearingErrorRMS(const double rms)
        {
            mBearingErrorRMS = rms; mPresenceVector |= PresenceVector::BearingErrorRMS;
        }
        /** Sets value and updates presence vector. */
        inline void SetInclinationErrorRMS(const double rms)
        {
            mInclinationErrorRMS = rms; mPresenceVector |= PresenceVector::InclinationErrorRMS;
        }
        /** Sets value and updates presence vector. */
        inline void SetRangeValidity(const bool valid = true)
        {
            mRangeValidFlag = valid; mPresenceVector |= PresenceVector::RangeValidity;
        }
        /** Sets value and updates presence vector. */
        inline void SetBearingValidity(const bool valid = true)
        {
            mBearingValidFlag = valid; mPresenceVector |= PresenceVector::BearingValidity;
        }
        /** Sets value and updates presence vector. */
        inline void SetInclinationValidity(const bool valid = true)
        {
            mInclinationValidFlag = valid; mPresenceVector |= PresenceVector::InclinationValidity;
        }
        /** Method to modify presence vector fields present. */
        inline void SetPresenceVector(const UShort pv) { mPresenceVector = pv; }
        inline void SetRange(const double x) { mPoint.mX = x; }
        inline void SetBearing(const double z) { mPoint.mZ = z; }
        inline void SetInclination(const double y) { mPoint.mY = y; }
        inline UInt GetID() const { return mID; }
        inline double GetRange() const { return mPoint.mX; }
        inline double GetInclination() const { return mPoint.mY; }
        inline double GetBearing() const { return mPoint.mZ; }
        inline bool IsRangeValid() const { return mRangeValidFlag; }
        inline bool IsInclinationValid() const { return mInclinationValidFlag; }
        inline bool IsBearingValid() const { return mBearingValidFlag; }
        inline Point3D GetRangePoint() const { return mPoint; }
        inline double GetRangeErrorRMS() const { return mRangeErrorRMS; }
        inline double GetBearingErrorRMS() const { return mBearingErrorRMS; }
        inline double GetInclinationErrorRMS() const { return mInclinationErrorRMS; }
        inline UShort GetPresenceVector() const { return mPresenceVector; }
        /** Convert native polar coordinates to vehicle polar coordinates given
            a sensor orientation (x = roll, y = pitch, z = yaw degrees) an
            sensor location (vehicle coordinates x = front, y = right, z positive down). */
        static Point3D ConvertToVehicleCoordinates(const Point3D& point,
                                      const Point3D& sensorOrientation,
                                      const Point3D& sensorLocation);
        static Point3D ConvertToSpherical(const Point3D& cartesian);
        static Point3D ConvertToCartesian(const Point3D& spherical);
        /** Writes JAUS formatted record data to packet. 
            \return Bytes written, -1 on error. */
        int Write(Packet& packet) const
        {
            int sum = 0;
            int expected = USHORT_SIZE;

            sum += packet.Write(mPresenceVector);
            if((mPresenceVector & PresenceVector::PointID) > 0)
            {
                sum += packet.Write(mID); expected += UINT_SIZE;
            }

            sum += ScaledInteger::Write(packet, mPoint.mX, Limits::MaxRange, Limits::MinRange, ScaledInteger::UInt);
            expected += UINT_SIZE;

            if((mPresenceVector & PresenceVector::RangeValidity) > 0)
            {
                sum += packet.Write(mRangeValidFlag); expected += BYTE_SIZE;
            }

            if((mPresenceVector & PresenceVector::RangeErrorRMS) > 0)
            {
                sum += ScaledInteger::Write(packet, mRangeErrorRMS, Limits::MaxRangeRMS, Limits::MinRangeRMS, ScaledInteger::UInt);
                expected += UINT_SIZE;
            }

            sum += ScaledInteger::Write(packet, mPoint.mZ, Limits::MaxAngle, Limits::MinAngle, ScaledInteger::UInt);
            expected += UINT_SIZE;

            if((mPresenceVector & PresenceVector::BearingValidity) > 0)
            {
                sum += packet.Write(mBearingValidFlag); expected += BYTE_SIZE;
            }

            if((mPresenceVector & PresenceVector::BearingErrorRMS) > 0)
            {
                sum += ScaledInteger::Write(packet, mBearingErrorRMS, Limits::MaxAngleRMS, Limits::MinAngleRMS, ScaledInteger::UInt);
                expected += UINT_SIZE;
            }

            sum += ScaledInteger::Write(packet, mPoint.mY, Limits::MaxAngle, Limits::MinAngle, ScaledInteger::UInt);
            expected += UINT_SIZE;

            if((mPresenceVector & PresenceVector::InclinationValidity) > 0)
            {
                sum += packet.Write(mInclinationValidFlag); expected += BYTE_SIZE;
            }

            if((mPresenceVector & PresenceVector::InclinationErrorRMS) > 0)
            {
                sum += ScaledInteger::Write(packet, mInclinationErrorRMS, Limits::MaxAngleRMS, Limits::MinAngleRMS, ScaledInteger::UInt);
                expected += UINT_SIZE;
            }

            return expected == sum ? sum : -1;
        }
        /** Reads JAUS formmated record data from packet. 
            \return Bytes read, -1 on error. */
        int Read(const Packet& packet)
        {
            int sum = 0;
            int expected = USHORT_SIZE;

            sum += packet.Read(mPresenceVector);
            if((mPresenceVector & PresenceVector::PointID) > 0)
            {
                sum += packet.Read(mID); expected += UINT_SIZE;
            }

            sum += ScaledInteger::Read(packet, mPoint.mX, Limits::MaxRange, Limits::MinRange, ScaledInteger::UInt);
            expected += UINT_SIZE;

            if((mPresenceVector & PresenceVector::RangeValidity) > 0)
            {
                sum += packet.Read((unsigned char &)mRangeValidFlag); expected += BYTE_SIZE;
            }

            if((mPresenceVector & PresenceVector::RangeErrorRMS) > 0)
            {
                sum += ScaledInteger::Read(packet, mRangeErrorRMS, Limits::MaxRangeRMS, Limits::MinRangeRMS, ScaledInteger::UInt);
                expected += UINT_SIZE;
            }

            sum += ScaledInteger::Read(packet, mPoint.mZ, Limits::MaxAngle, Limits::MinAngle, ScaledInteger::UInt);
            expected += UINT_SIZE;

            if((mPresenceVector & PresenceVector::BearingValidity) > 0)
            {
                sum += packet.Read((unsigned char &)mBearingValidFlag); expected += BYTE_SIZE;
            }

            if((mPresenceVector & PresenceVector::BearingErrorRMS) > 0)
            {
                sum += ScaledInteger::Read(packet, mBearingErrorRMS, Limits::MaxAngleRMS, Limits::MinAngleRMS, ScaledInteger::UInt);
                expected += UINT_SIZE;
            }

            sum += ScaledInteger::Read(packet, mPoint.mY, Limits::MaxAngle, Limits::MinAngle, ScaledInteger::UInt);
            expected += UINT_SIZE;

            if((mPresenceVector & PresenceVector::InclinationValidity) > 0)
            {
                sum += packet.Read((unsigned char &)mInclinationValidFlag); expected += BYTE_SIZE;
            }

            if((mPresenceVector & PresenceVector::InclinationErrorRMS) > 0)
            {
                sum += ScaledInteger::Read(packet, mInclinationErrorRMS, Limits::MaxAngleRMS, Limits::MinAngleRMS, ScaledInteger::UInt);
                expected += UINT_SIZE;
            }

            return expected == sum ? sum : -1;
        }
        /** Clears all values. */
        void Clear()
        {
            mPresenceVector = 0;
            mID = 0;
            mPoint.Clear();
            mRangeValidFlag = mBearingValidFlag = mInclinationValidFlag = true;
            mRangeErrorRMS = mBearingErrorRMS = mInclinationErrorRMS = 0.0;
        }
        /** Overload of the parenthesis operators to set range point info. */
        Range& operator()(const double range, 
                          const double bearing, 
                          const double inclination = 0)
        {
            mPoint(range, inclination, bearing);
            return *this;
        }
        Range& operator=(const Range& range)
        {
            if(this != &range)
            {
                mPresenceVector = range.mPresenceVector;
                mID = range.mID;
                mPoint = range.mPoint;
                mRangeValidFlag = range.mRangeValidFlag;
                mBearingValidFlag = range.mBearingValidFlag;
                mInclinationValidFlag = range.mInclinationValidFlag;
                mRangeErrorRMS = range.mRangeErrorRMS;
                mBearingErrorRMS = range.mBearingErrorRMS;
                mInclinationErrorRMS = range.mInclinationErrorRMS;
            }
            return *this;
        }
    protected:
        UShort mPresenceVector;         ///<  Presence vector indicating presence of optional fields.
        UInt mID;                       ///<  ID value, default is 0 when not set by presence vector. 
        Point3D mPoint;                 ///<  Range data point (mX = range meters, mY = inclination [-pi,pi], mZ = bearing [-pi, pi]).
        bool mRangeValidFlag;           ///<  Range validity flag, if presence vector bit not set, default value is true (valid).
        bool mBearingValidFlag;         ///<  Bearing validity flag, if presence vector bit not set, default is true (valid).
        bool mInclinationValidFlag;     ///<  Inclination validity flag, if presence vector bit not set, default is true (valid).
        double mRangeErrorRMS;          ///<  Range error RMS in meters [0, 100,000].
        double mBearingErrorRMS;        ///<  Range error RMS in meters [0, 100,000].
        double mInclinationErrorRMS;    ///<  Range error RMS in meters [0, 100,000].
    };
}

#endif
/*  End of File */
