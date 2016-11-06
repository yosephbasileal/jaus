////////////////////////////////////////////////////////////////////////////////////
///
///  \file StaticGeometricProperties.h
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
#ifndef __JAUS_ENVIRONMENT_SENSING_STATIC_GEEOMETRIC_PROPERTIES__H
#define __JAUS_ENVIRONMENT_SENSING_STATIC_GEEOMETRIC_PROPERTIES__H

#include "jaus/core/Types.h"
#include "jaus/core/Message.h"
#include "jaus/environment/EnvironmentCodes.h"

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class StaticGeometricProperties
    ///   \brief Data structure representing the geometric properties of a staticly mounted sensor
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_ENVIRONMENT_DLL StaticGeometricProperties
    {
    public:
        class JAUS_ENVIRONMENT_DLL SensorPosition{
        public:
            SensorPosition(UInt x = 0, UInt y = 0, UInt z = 0) :
              mX(x), mY(y), mZ(z) {}
              virtual UInt GetX() { return mX; }
              virtual UInt GetY() { return mY; }
              virtual UInt GetZ() { return mZ; }
        private:
            UInt mX, mY, mZ;
        };
        class JAUS_ENVIRONMENT_DLL UnitQuaternion{
        public:
            UnitQuaternion(UInt d = 0, UInt a = 0, UInt b = 0, UInt c = 0) :
              mD(d), mA(a), mB(b), mC(c) {}
              virtual UInt GetA() { return mA; }
              virtual UInt GetB() { return mB; }
              virtual UInt GetC() { return mC; }
              virtual UInt GetD() { return mD; }
        private:
            UInt mA, mB, mC, mD;
        };
        StaticGeometricProperties(UInt x = 0, UInt y = 0, UInt z = 0, UInt d = 0, UInt a = 0, UInt b = 0, UInt c = 0) 
        {
            mPosition = SensorPosition(x,y,z);
            mQuaternion = UnitQuaternion(d,a,b,c);
        }
        ~StaticGeometricProperties() {}
        virtual UInt GetSize() { return UINT_SIZE * 7; }
        virtual SensorPosition GetSensorPosition() const { return mPosition;}
        virtual UnitQuaternion GetUnitQuaternion() const { return mQuaternion;}
        virtual void SetSensorPosition(UInt x, UInt y, UInt z)
        {
            mPosition = SensorPosition(x,y,z);
        }
        virtual void SetUnitQuaternion(UInt d, UInt a, UInt b, UInt c)
        {
            mQuaternion = UnitQuaternion(d,a,b,c);
        }
        virtual void PrintSensorFields() const;
        virtual int ReadMessageBody(const Packet& packet);
        virtual int WriteMessageBody(Packet& packet) const;
    private:
        SensorPosition mPosition;   ///<  Required;
        UnitQuaternion mQuaternion; ///<  Required;
    };
}

#endif
/*  End of File */
