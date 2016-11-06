////////////////////////////////////////////////////////////////////////////////////
///
///  \file Range.cpp
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
#include "jaus/environment/range/Range.h"
#include <cxutils/math/CxMath.h>

const double JAUS::Range::Limits::MinRange = 0;
const double JAUS::Range::Limits::MaxRange = 1000000;
const double JAUS::Range::Limits::MinRangeRMS = 0;
const double JAUS::Range::Limits::MaxRangeRMS = 100000;
const double JAUS::Range::Limits::MinAngleRMS = 0;
const double JAUS::Range::Limits::MaxAngleRMS = CxUtils::CX_PI;
const double JAUS::Range::Limits::MinAngle = -CxUtils::CX_PI;
const double JAUS::Range::Limits::MaxAngle = CxUtils::CX_PI;

using namespace JAUS;

Point3D Range::ConvertToVehicleCoordinates(const Point3D& nativePoint,
                              const Point3D& sensorOrientation,
                              const Point3D& sensorLocation)
{
    CxUtils::Point3D cartesian;

    cartesian.mX = nativePoint.mX;
    cartesian = cartesian.Rotate(nativePoint.mY, CxUtils::Point3D::Y, false);
    cartesian = cartesian.Rotate(nativePoint.mZ, CxUtils::Point3D::Z, false);

    // Now rotate based on sensor orientation on vehicle.
    cartesian = cartesian.Rotate(sensorOrientation.mX, CxUtils::Point3D::X, false);
    cartesian = cartesian.Rotate(sensorOrientation.mY, CxUtils::Point3D::Y, false);
    cartesian = cartesian.Rotate(sensorOrientation.mZ, CxUtils::Point3D::Z, false);

    // Now translate based on sensor location on vehicle
    cartesian += sensorLocation;

    // Now convert to polar and save result.

    CxUtils::Point3D sphericalLocation;
    sphericalLocation.mX = cartesian.Magnitude();

    sphericalLocation.mY = atan2(sqrt(cartesian.mX*cartesian.mX + 
        cartesian.mY*cartesian.mY),
        cartesian.mZ) - CxUtils::CX_HALF_PI;
    sphericalLocation.mZ = atan2(cartesian.mY, cartesian.mX);

    return sphericalLocation;
}

Point3D Range::ConvertToSpherical(const Point3D& cartesian)
{
    CxUtils::Point3D sphericalLocation;
    sphericalLocation.mX = cartesian.Magnitude();

    sphericalLocation.mY = atan2(sqrt(cartesian.mX*cartesian.mX + 
        cartesian.mY*cartesian.mY),
        cartesian.mZ) - CxUtils::CX_HALF_PI;
    sphericalLocation.mZ = atan2(cartesian.mY, cartesian.mX);

    return sphericalLocation;
}

Point3D Range::ConvertToCartesian(const Point3D& spherical)
{
    CxUtils::Point3D cartesian;

    cartesian.mX = spherical.mX;
    cartesian = cartesian.Rotate(spherical.mY, CxUtils::Point3D::Y, false);
    cartesian = cartesian.Rotate(spherical.mZ, CxUtils::Point3D::Z, false);

    return cartesian;
}

/** End of File */
