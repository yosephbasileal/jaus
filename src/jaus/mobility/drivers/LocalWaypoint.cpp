////////////////////////////////////////////////////////////////////////////////////
///
///  \file LocalWaypoint.cpp
///  \brief This file contains the implementation of a JAUS message.
///
///  <br>Author(s): Bo Sun
///  <br>Created: 30 November 2009
///  <br>Copyright (c) 2009
///  <br>Applied Cognition and Training in Immersive Virtual Environments
///  <br>(ACTIVE) Laboratory
///  <br>Institute for Simulation and Training (IST)
///  <br>University of Central Florida (UCF)
///  <br>All rights reserved.
///  <br>Email: bsun@ist.ucf.edu
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
#include "jaus/mobility/drivers/LocalWaypoint.h"
#include "jaus/core/ScaledInteger.h"
#include "jaus/mobility/drivers/SetLocalWaypoint.h"
#include "jaus/mobility/drivers/ReportLocalWaypoint.h"
#include <cxutils/math/CxMath.h>
#include <iomanip>

const double JAUS::LocalWaypoint::Limits::MinPoint = -100000;
const double JAUS::LocalWaypoint::Limits::MaxPoint = 100000;
const double JAUS::LocalWaypoint::Limits::MinAngle = -CxUtils::CX_PI;
const double JAUS::LocalWaypoint::Limits::MaxAngle = CxUtils::CX_PI;
const double JAUS::LocalWaypoint::Limits::MinWaypointTolerance = 0.0;
const double JAUS::LocalWaypoint::Limits::MaxWaypointTolerance = 100.0;
const double JAUS::LocalWaypoint::Limits::MinPathTolerance = 0.0;
const double JAUS::LocalWaypoint::Limits::MaxPathTolerance = 100000.0;

using namespace JAUS;


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor, initializes default values.
///
////////////////////////////////////////////////////////////////////////////////////
LocalWaypoint::LocalWaypoint()
{
    mPresenceVector = 0;
    mX = 0;
    mY = 0;
    mZ = 0;
    mRoll = 0;
    mPitch = 0;
    mYaw = 0;
    mWaypointTolerance = 0;
    mPathTolerance = 0;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Copy constructor.
///
////////////////////////////////////////////////////////////////////////////////////
LocalWaypoint::LocalWaypoint(const LocalWaypoint& wp)
{
    *this = wp;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
LocalWaypoint::~LocalWaypoint()
{
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the latitude and updates the presence vector for the wp.
///
///   \param[in] value Desired X in meters [-100000, 100000].
///
///   \return true on success, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
bool LocalWaypoint::SetX(const double value)
{
    if(value >= Limits::MinPoint && value <= Limits::MaxPoint)
    {
        mX = value;
        return true;
    }
    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the longitude and updates the presence vector for the wp.
///
///   \param[in] value Desired Y in meters [-100000, 100000].
///
///   \return true on success, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
bool LocalWaypoint::SetY(const double value)
{
    if(value >= Limits::MinPoint && value <= Limits::MaxPoint)
    {
        mY = value;
        return true;
    }
    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the altitude and updates the presence vector for the
///          wp.
///
///   \param[in] value Desired Z in meters [-100000, 100000].
///
///   \return true on success, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
bool LocalWaypoint::SetZ(const double value)
{
    if(value >= Limits::MinPoint && value <= Limits::MaxPoint)
    {
        mZ = value;
        mPresenceVector |= PresenceVector::Z;
        return true;
    }
    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the roll and updates the presence vector for the wp.
///
///   \param[in] radians Desired roll in radians[-PI, PI].
///
///   \return true on success, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
bool LocalWaypoint::SetRoll(const double radians)
{
    if(radians >= Limits::MinAngle && radians <= Limits::MaxAngle)
    {
        mRoll = radians;
        mPresenceVector |= PresenceVector::Roll;
        return true;
    }
    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the pitch value and updates the presence vector for the wp.
///
///   \param[in] radians Desired pitch in radians[-PI, PI].
///
///   \return true on success, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
bool LocalWaypoint::SetPitch(const double radians)
{
    if(radians >= Limits::MinAngle && radians <= Limits::MaxAngle)
    {
        mPitch = radians;
        mPresenceVector |= PresenceVector::Pitch;
        return true;
    }
    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the yaw value and updates the presence vector for the wp.
///
///   \param[in] radians Desired pitch in radians[-PI, PI].
///
///   \return true on success, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
bool LocalWaypoint::SetYaw(const double radians)
{
    if(radians >= Limits::MinAngle && radians <= Limits::MaxAngle)
    {
        mYaw = radians;
        mPresenceVector |= PresenceVector::Yaw;
        return true;
    }
    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the waypoint tolerance value and updates the presence vector for
///          the wp.
///
///   \param[in] value Desired waypoint tolerance in meters[0, 100].
///
///   \return true on success, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
bool LocalWaypoint::SetWaypointTolerance(const double value)
{
    if(value >= Limits::MinWaypointTolerance && value <= Limits::MaxWaypointTolerance)
    {
        mWaypointTolerance= value;
        mPresenceVector |= PresenceVector::WaypointTolerance;
        return true;
    }
    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the path tolerance value and updates the presence vector for
///          the wp.
///
///   \param[in] value Desired path tolerance in meters[0, 100000].
///
///   \return true on success, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
bool LocalWaypoint::SetPathTolerance(const double value)
{
    if(value >= Limits::MinPathTolerance && value <= Limits::MaxPathTolerance)
    {
        mPathTolerance= value;
        mPresenceVector |= PresenceVector::PathTolerance;
        return true;
    }
    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Clears wp payload data.
///
////////////////////////////////////////////////////////////////////////////////////
void LocalWaypoint::Clear()
{
    mActiveFlag = mFinishedFlag = false;
    mPresenceVector = 0;
    mX = 0;
    mY = 0;
    mZ = 0;
    mRoll = 0;
    mPitch = 0;
    mYaw = 0;
    mWaypointTolerance = 0;
    mPathTolerance = 0;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Method to check if the waypoints are the same by comparing distance
///          between locations only.
///
///   \param[in] waypoint Waypoint to compare with.
///   \param[in] errorInMeters Tolerance for distance check in meters.
///
///   \return True if the same, false otherwise.
///
////////////////////////////////////////////////////////////////////////////////////
bool LocalWaypoint::IsSameAs(const Waypoint* waypoint, const double errorInMeters) const
{
    const LocalWaypoint* lp = dynamic_cast<const LocalWaypoint*>(waypoint);
    if(lp)
    {
        Point3D a(mX, mY, mZ), b(lp->mX, lp->mY, lp->mZ);
        if(Point3D::Distance(a, b) <= errorInMeters)
            return true;
    }
    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets equal to.
///
////////////////////////////////////////////////////////////////////////////////////
LocalWaypoint& LocalWaypoint::operator=(const LocalWaypoint& wp)
{
    if(this != &wp)
    {
        mActiveFlag = wp.mActiveFlag;
        mFinishedFlag = wp.mFinishedFlag;
        mPresenceVector = wp.mPresenceVector;
        mX = wp.mX;
        mY = wp.mY;
        mZ = wp.mZ;
        mRoll = wp.mRoll;
        mPitch = wp.mPitch;
        mYaw = wp.mYaw;
        mWaypointTolerance = wp.mWaypointTolerance;
        mPathTolerance = wp.mPathTolerance;
    }
    return *this;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets equal to.
///
////////////////////////////////////////////////////////////////////////////////////
LocalWaypoint& LocalWaypoint::operator=(const SetLocalWaypoint& wp)
{
    *this = *((LocalWaypoint*)(&wp));
    return *this;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets equal to.
///
////////////////////////////////////////////////////////////////////////////////////
LocalWaypoint& LocalWaypoint::operator=(const ReportLocalWaypoint& wp)
{
    *this = *((LocalWaypoint*)(&wp));
    return *this;
}


/*  End of File */
