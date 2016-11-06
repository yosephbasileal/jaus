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
#include "cxutils/math/Encoders.h"

using namespace CxUtils;


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////////
Encoders::Motor::Motor() : mRelativeFlag(true), mCount(0), mValuesPerRevolution(1), mShaftRadius(.01), mSamplePeriodMs(0)
{
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Copy constructor.
///
////////////////////////////////////////////////////////////////////////////////////
Encoders::Motor::Motor(const Motor& encoder) : mRelativeFlag(true), mCount(0), mValuesPerRevolution(1), mShaftRadius(.01), mSamplePeriodMs(0)
{
    *this = encoder;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
Encoders::Motor::~Motor()
{
    
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Clears all values.
///
////////////////////////////////////////////////////////////////////////////////////
void Encoders::Motor::Clear()
{
    mRelativeFlag = true;
    mCount = 0;
    mSamplePeriodMs = 0;
    mTimeStamp.Clear();
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the encoder values to meters at a specific radius
///          from the motor shaft (like the end of a wheel).
///
///   \param[in] radius Shaft/Wheel radius.
///   \param[in] ticksPerRevolution Number of encoder values in 1 revolution of
///                                 the motor shaft.
///
////////////////////////////////////////////////////////////////////////////////////
double Encoders::Motor::ToMeters(const double radius,
                                 const long long int ticksPerRevolution) const
{
    return (mCount*radius*CxUtils::CX_TWO_PI)/ticksPerRevolution;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the encoder data to a linear velocity in m/s.  This would
///          equate to the velocity on the ground based on wheel movement.
///
///   For rotational velocity, see ToRotationalVelocity which is in rad/s.
///
///   \param[in] radius Shaft/Wheel radius.
///   \param[in] ticksPerRevolution Number of encoder values in 1 revolution of
///                                 the motor shaft.
///
////////////////////////////////////////////////////////////////////////////////////
double Encoders::Motor::ToLinearVelocity(const double radius,
                                 const long long int ticksPerRevolution) const
{
    return (mCount*radius*CxUtils::CX_TWO_PI)/(ticksPerRevolution*mSamplePeriodMs/1000.0);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the encoder data to a rotational velocity in radians/s.
///
///   \param[in] ticksPerRevolution Number of encoder values in 1 revolution of
///                                 the motor shaft.
///
////////////////////////////////////////////////////////////////////////////////////
double Encoders::Motor::ToRotationalVelocity(const long long int ticksPerRevolution) const
{
    return (mCount*CxUtils::CX_TWO_PI)/(ticksPerRevolution*mSamplePeriodMs/1000.0);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Adds encoder data together. TimeStamp is set to the less recent
///          time of the two to mark the beginning of samples.
///
////////////////////////////////////////////////////////////////////////////////////
Encoders::Motor Encoders::Motor::operator + (const Encoders::Motor& encoder) const
{
    Motor result;
    result.mShaftRadius = this->mShaftRadius;
    result.mValuesPerRevolution = this->mValuesPerRevolution;
    result.mRelativeFlag = this->mRelativeFlag;
    if(result.mRelativeFlag)
    {
        result.mCount = mCount + encoder.mCount;
    }
    else
    {
        result.mCount = encoder.mCount;
    }
    result.mSamplePeriodMs = mSamplePeriodMs + encoder.mSamplePeriodMs;
    result.mTimeStamp = mTimeStamp < encoder.mTimeStamp ? mTimeStamp : encoder.mTimeStamp;
    return result;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Adds encoder data together. TimeStamp is set to the more recent
///          time of the two.
///
////////////////////////////////////////////////////////////////////////////////////
Encoders::Motor& Encoders::Motor::operator += (const Encoders::Motor& encoder)
{
    return *this = *this + encoder;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets equal to.
///
////////////////////////////////////////////////////////////////////////////////////
Encoders::Motor& Encoders::Motor::operator=(const Encoders::Motor& encoder)
{
    if(this != &encoder)
    {
        mRelativeFlag = encoder.mRelativeFlag;
        mCount = encoder.mCount;
        mValuesPerRevolution = encoder.mValuesPerRevolution;
        mShaftRadius = encoder.mShaftRadius;
        mSamplePeriodMs = encoder.mSamplePeriodMs;
        mTimeStamp = encoder.mTimeStamp;
    }

    return *this;
}



////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////////
Encoders::Encoders()
{
    mDirtyFlag = false;
    mWidth = 1.0;
    mUpdateCounter = 0;
    mLeftSideDist = mRightSideDist = 0.0;
    mLeftSideVelocity = mRightSideVelocity = 0.0;
    mSamplePeriodSeconds = 1.0;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Copy constructor.
///
////////////////////////////////////////////////////////////////////////////////////
Encoders::Encoders(const Encoders& body)
{
    *this = body;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
Encoders::~Encoders()
{
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Update encoder data for a side of the rigid body.
///
///   \param[in] encoderID Motor ID number.
///   \param[in] encoderData Motor values.
///
////////////////////////////////////////////////////////////////////////////////////
void Encoders::UpdateLeftEncoder(const unsigned int encoderID,
                                  const Motor& encoderData)
{
    if(mLeftSide.find(encoderID) == mLeftSide.end())
    {
        mLeftSide[encoderID] = encoderData;
    }
    else
    {
        mLeftSide[encoderID] += encoderData;
    }
    mDirtyFlag = true;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Update encoder data for a side of the rigid body.
///
///   \param[in] encoderID Motor ID number.
///   \param[in] encoderData Motor values.
///
////////////////////////////////////////////////////////////////////////////////////
void Encoders::UpdateRightEncoder(const unsigned int encoderID,
                                   const Motor& encoderData)
{
    if(mRightSide.find(encoderID) == mRightSide.end())
    {
        mRightSide[encoderID] = encoderData;
    }
    else
    {
        mRightSide[encoderID] += encoderData;
    }
    mDirtyFlag = true;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Resets all values but width.
///
////////////////////////////////////////////////////////////////////////////////////
void Encoders::Reset()
{
    mDirtyFlag = false;
    mLeftSide.clear();
    mRightSide.clear();
    mUpdateCounter = 0;
    mLeftSideDist = mRightSideDist = 0.0;
    mLeftSideVelocity = mRightSideVelocity = 0.0;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Updates internal values using current encoder data values.
///
///   \return True on success (enough values) false otherwise.
///
////////////////////////////////////////////////////////////////////////////////////
bool Encoders::Update()
{
    CxUtils::Time currentTime(true);
    mLeftSideDist = mRightSideDist = 0.0;
    mLeftSideVelocity = mRightSideVelocity = 0.0;
    int leftValues = 0, rightValues = 0;
    Motor::Map::iterator e;
    double longestPeriod = 0.0;

    for(e = mLeftSide.begin(); e != mLeftSide.end(); e++)
    {
        if(e->second.mSamplePeriodMs > longestPeriod)
        {
            longestPeriod = (double)e->second.mSamplePeriodMs;
        }
        Motor copy = e->second;
        mLeftSideDist += e->second.ToMeters();
        mLeftSideVelocity += mLeftSideDist/(e->second.mSamplePeriodMs/1000.0);
        leftValues++;
    }

    for(e = mRightSide.begin(); e != mRightSide.end(); e++)
    {
        if(e->second.mSamplePeriodMs > longestPeriod)
        {
            longestPeriod = (double)e->second.mSamplePeriodMs;
        }
        Motor copy = e->second;
        mRightSideDist += e->second.ToMeters();
        mRightSideVelocity += mRightSideDist/(e->second.mSamplePeriodMs/1000.0);
        rightValues++;
    }

    if(leftValues > 0 && rightValues > 0)
    {
        mLeftSideDist /= leftValues;
        mRightSideDist /= rightValues;
        mLeftSideVelocity /= leftValues;
        mRightSideVelocity /= rightValues;
        mLeftSide.clear();
        mRightSide.clear();
        mUpdateCounter++;
        // If this is the first time called...
        if(mUpdateTime.ToMs() == 0)
        {
            mSamplePeriodSeconds = longestPeriod/1000.0;
        }
        else
        {
            mSamplePeriodSeconds = currentTime - mUpdateTime;
        }
        mUpdateTime = currentTime;
        return true;
    }

    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Gets the linear velocity data calculated using the Update
///          method.
///
////////////////////////////////////////////////////////////////////////////////////
double Encoders::GetLinearVelocity() const
{
    if(mUpdateCounter > 0)
    {
        return (mLeftSideDist + mRightSideDist)/(mSamplePeriodSeconds*2.0);
    }
    else
    {
        return 0.0;
    }
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Gets the rotational velocity data calculated using the Update
///          method.
///
////////////////////////////////////////////////////////////////////////////////////
double Encoders::GetRotationalVelocity() const
{
    if(mUpdateCounter > 0)
    {
        return (mLeftSideVelocity - mRightSideVelocity)/mWidth;
        /*
        double v = (mLeftSideDist - mRightSideDist)/mWidth;
        if(v < -1 || v > 1.0)
        {
            // ERROR
            return 0.0;
        }
        return asin(v)/(mSamplePeriodSeconds);
        */
    }
    else
    {
        return 0.0;
    }
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets equal to.
///
////////////////////////////////////////////////////////////////////////////////////
Encoders& Encoders::operator =(const Encoders& body)
{
    if(this != &body)
    {
        mDirtyFlag = body.mDirtyFlag;
        mLeftSide = body.mLeftSide;
        mRightSide = body.mRightSide;
        mWidth = body.mWidth;
        mUpdateCounter = body.mUpdateCounter;
        mLeftSideDist = body.mLeftSideDist;
        mRightSideDist = body.mRightSideDist;
        mLeftSideVelocity = body.mLeftSideVelocity;
        mRightSideVelocity = body.mRightSideVelocity;
        mSamplePeriodSeconds = body.mSamplePeriodSeconds;
    }
    return *this;
}


/* End of File */
