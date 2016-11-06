////////////////////////////////////////////////////////////////////////////////////
///
///  \file GeometricProperties.h
///  \brief Data structure representing the geometric properties of a sensor.
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
#ifndef __JAUS_ENVIRONMENT_SENSING_GEEOMETRIC_PROPERTIES__H
#define __JAUS_ENVIRONMENT_SENSING_GEEOMETRIC_PROPERTIES__H

#include "jaus/environment/geometric/StaticGeometricProperties.h"
#include "jaus/environment/geometric/ManipulatorGeometricProperties.h"

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class GeometricProperties
    ///   \brief Data structure representing a sensors geometric properties
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_ENVIRONMENT_DLL GeometricProperties
    {
    public:
        GeometricProperties(UShort sensorId = 0, Byte geometricVariant = 0);
        ~GeometricProperties() {}
        virtual UInt GetSize() const;
        virtual UShort GetSensorId() const { return mSensorId; }
        virtual Byte GetGeometricVariant() const { return mGeometricVariant; }
        virtual Byte GetNoGeometricVariant() const { return (Byte) 0; }
        virtual StaticGeometricProperties GetStaticGeometricProperties() const { return mStaticGeometric; }
        virtual ManipulatorGeometricProperties GetManipulatorGeometricProperties() const { return mManipulatorGeometric; }
        virtual void SetStaticGeometricProperties(UInt x, UInt y, UInt z, UInt d, UInt a, UInt b, UInt c)
        {
            mStaticGeometric = StaticGeometricProperties(x,y,z,d,a,b,c);
        }
        virtual void SetStaticGeometricProperties(StaticGeometricProperties geoProperties)
        {
            mStaticGeometric = geoProperties;
        }
        virtual void SetManipulatorGeometricProperties(UShort subsystemId, Byte nodeId, Byte componentId, Byte jointNumber,
            UInt x, UInt y, UInt z, 
            UInt d, UInt a, UInt b, UInt c)
        {
            mManipulatorGeometric = ManipulatorGeometricProperties(subsystemId, nodeId, componentId, jointNumber,x,y,z,d,a,b,c);
        }
        virtual void SetManipulatorGeometricProperties(ManipulatorGeometricProperties geoProperties)
        {
            mManipulatorGeometric = geoProperties;
        }
        virtual void SetSensorId(UShort sensorId) { mSensorId = sensorId; }
        virtual void SetGeometricVariant(Byte geometricVariant) { mGeometricVariant = geometricVariant;}
        virtual void PrintSensorFields() const;
        virtual int ReadMessageBody(const Packet& packet);
        virtual int WriteMessageBody(Packet& packet) const;
    private:
        UInt mSize;
        UShort mSensorId;               ///<  Required; SensorID value of "0" is invalid; 
        Byte mGeometricVariant;         ///<  Required; 0 = No knowledge; 1 = Static with respect to vehicle; 2 = moviable with respect to vehicle
        StaticGeometricProperties mStaticGeometric; 
        ManipulatorGeometricProperties mManipulatorGeometric; 
    };
}

#endif
/*  End of File */
