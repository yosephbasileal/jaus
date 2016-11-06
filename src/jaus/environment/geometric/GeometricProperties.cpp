////////////////////////////////////////////////////////////////////////////////////
///
///  \file GeometricProperties.cpp
///  \brief Data structure representing the geometric properties of a sensor.
///
///  <br>Author(s): Daniel Barber, Jonathan Harris
///  <br>Created: 20 March 2013
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
#include <string>
#include "jaus/environment/geometric/GeometricProperties.h"

using namespace JAUS;

GeometricProperties::GeometricProperties(UShort sensorId, Byte geometricVariant) :    
mSensorId(sensorId), mGeometricVariant(geometricVariant) {}

UInt GeometricProperties::GetSize() const
{
    UInt size = 0;
    switch(mGeometricVariant)
    {
    case 0:
        size = USHORT_SIZE + BYTE_SIZE;
        break;
    case 1:
        size = USHORT_SIZE + UINT_SIZE * 7;
        break;
    case 2:
        size = (BYTE_SIZE * 3) + (USHORT_SIZE * 2) + (UINT_SIZE * 7);
        break;
    }
    return size;
}

int GeometricProperties::ReadMessageBody(const Packet& packet) 
{
    UInt startPos = packet.GetReadPos();
    packet.Read(mSensorId);
    packet.Read(mGeometricVariant);
    if(mGeometricVariant == 0)
    {
        //Do nothing because the sensor doesn't know anything abou it's geometric properties
    }
    if(mGeometricVariant == 1) //Set the static property
    {
        mStaticGeometric.ReadMessageBody(packet);
    }
    if(mGeometricVariant == 2) //Set the Manipulatible property
    {
        mManipulatorGeometric.ReadMessageBody(packet);
    }
    return packet.GetReadPos() - startPos;
}

int GeometricProperties::WriteMessageBody(Packet& packet) const
{ 
    UInt startPos = packet.GetWritePos();
    packet.Write(mSensorId);
    packet.Write(mGeometricVariant);
    if(mGeometricVariant == 0) //No Knowledge of geometric properties
    {
        packet.Write((Byte) 0);
    }
    if(mGeometricVariant == 1) //Staticly connected to the vehicle
    {
        mStaticGeometric.WriteMessageBody(packet);
    }
    if(mGeometricVariant == 2) //connected to a manipulator on the vehicle
    {
        mManipulatorGeometric.WriteMessageBody(packet);
    }
    return packet.GetWritePos() - startPos;
}

void GeometricProperties::PrintSensorFields() const
{
    std::cout << "Size: " << (UInt) GetSize() << std::endl 
              << "SensorId:  " << (UInt) mSensorId  << std::endl
              << "Geometric Variant: " << (UInt) mGeometricVariant << std::endl
              << "StaticGeometricPropertiess: ";
    mStaticGeometric.PrintSensorFields();
    std::cout << std::endl;
    std::cout << "Manipulator Geometric Properties: ";
    mManipulatorGeometric.PrintSensorFields();
    std::cout << std::endl;
}

/** End of File */
