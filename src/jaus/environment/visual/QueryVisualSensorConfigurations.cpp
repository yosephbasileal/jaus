////////////////////////////////////////////////////////////////////////////////////
///
///  \file QueryVisualSensorConfigurations.cpp
///  \brief This file contains the implementation of a JAUS message.
///
///  <br>Author(s): Daniel Barber, Jonathan Harris
///  <br>Created: 21 March 2013
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
#include "jaus/environment/visual/QueryVisualSensorConfigurations.h"

using namespace JAUS;

QueryVisualSensorConfigurations::QueryVisualSensorConfigurations(const Address& dest, const Address& src) : 
Message(QUERY_VISUAL_SENSOR_CONFIGURATION, dest, src)
{
    mSensorList = List();
}

QueryVisualSensorConfigurations::QueryVisualSensorConfigurations(const QueryVisualSensorConfigurations& message) : 
Message(QUERY_VISUAL_SENSOR_CONFIGURATION)
{
    *this = message;
}

QueryVisualSensorConfigurations::~QueryVisualSensorConfigurations()
{
    mSensorList.clear();
}

int QueryVisualSensorConfigurations::WriteMessageBody(Packet& packet) const
{
    UInt startPos = packet.GetWritePos();
    UShort count = (UShort)mSensorList.size();
    List::const_iterator visualSensorConfiguration = mSensorList.begin();
    packet.Write(count);
    while(visualSensorConfiguration != mSensorList.end())
    {
        packet.Write(visualSensorConfiguration->GetSensorID());           //required
        packet.Write(visualSensorConfiguration->GetPresenceVector());     //required
        visualSensorConfiguration++;
    }

    return packet.GetWritePos() - startPos;
}

int QueryVisualSensorConfigurations::ReadMessageBody(const Packet& packet)
{
    UInt startPos = packet.GetReadPos();
    UShort count = 0;
    packet.Read(count);
    for(int i = 0; i < (int)count; i++)
    {
        UShort sensorID = 0;
        UShort presenceVector = 0;
        packet.Read(sensorID);
        packet.Read(presenceVector);
        VisualSensorConfiguration vsc = VisualSensorConfiguration(presenceVector, sensorID);
        mSensorList.push_back(vsc);
    }
    return packet.GetReadPos() - startPos;
}

bool QueryVisualSensorConfigurations::IsLargeDataSet(const UInt maxPayloadSize) const
{
    UInt dataSetSize = ((USHORT_SIZE * 2) * mSensorList.size());
    return dataSetSize > maxPayloadSize;
}

void QueryVisualSensorConfigurations::PrintMessageBody() const
{
    std::cout << "Sensor Count: " << mSensorList.size() << std::endl;
    UShort count = (UShort)mSensorList.size();
    List::const_iterator sensor = mSensorList.begin();
    while(sensor != mSensorList.end())
    {
        std::cout << "<Sensor>" <<  std::endl;
        sensor->PrintSensorFields();
        std::cout << "</Sensor>" <<  std::endl;
        sensor++;
    }
}

/*  End of File */
