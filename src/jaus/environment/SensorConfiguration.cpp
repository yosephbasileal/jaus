////////////////////////////////////////////////////////////////////////////////////
///
///  \file SensorConfiguration.cpp
///  \brief Data structure representing a Sensor Data configuration.
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
#include "jaus/environment/SensorConfiguration.h"

using namespace JAUS;

int SensorConfiguration::ReadMessageBody(const Packet& packet)
{
    UInt startPos = packet.GetReadPos();
    packet.Read(mVariant);
    packet.Read(mSensorId);
    if(mVariant > 0) // we had an error
    {
        Byte errorCode = 0;
        Byte errorMessageLength = 0;
        std::string message;
        packet.Read(errorCode);
        packet.Read(errorMessageLength);
        packet.Read(message,errorMessageLength);
        SetError(mVariant, errorCode, message);
    }
    return packet.GetReadPos() - startPos;
}

int SensorConfiguration::WriteMessageBody(Packet& packet) const
{ 
    UInt startPos = packet.GetWritePos();
    packet.Write(mVariant);
    packet.Write(mSensorId);
    if(mVariant == 0)
    {
        //Write nothing because the set was sucessful
    }
    else // write the error
    {
        SensorConfiguration::SensorError err = mSensorError;
        packet.Write(mSensorError.GetSensorErrorCode());
        packet.Write(mSensorError.GetMessageLength());
        packet.Write(mSensorError.GetMessage());
    }
    return packet.GetWritePos() - startPos;
}

void SensorConfiguration::PrintSensorFields() const
{
    std::cout << "Size: " << GetSize() << std::endl 
              << "Sensor Id:  " << (UInt) mSensorId  << std::endl
              << "Variant:  " << (UInt) mVariant << std::endl
              << "Sensor Error Code: " << mSensorError.GetSensorErrorCode() << std::endl
              << "Sensor Error message: " << mSensorError.GetMessage() << std::endl;
}

/** End of File */
