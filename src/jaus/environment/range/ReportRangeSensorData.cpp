////////////////////////////////////////////////////////////////////////////////////
///
///  \file ReportRangeSensorData.cpp
///  \brief This file contains the implementation of a JAUS message.
///
///  <br>Author(s): Daniel Barber
///  <br>Created: 2013
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
#include "jaus/environment/range/ReportRangeSensorData.h"

using namespace JAUS;

ReportRangeSensorData::ReportRangeSensorData(const Address& dest,
                                             const Address& src) 
                                             : Message(REPORT_RANGE_SENSOR_DATA, dest, src)
{

}


ReportRangeSensorData::ReportRangeSensorData(const ReportRangeSensorData& msg)
    : Message(REPORT_RANGE_SENSOR_DATA, msg.GetDestinationID(), msg.GetSourceID())
{
    *this = msg;
}


ReportRangeSensorData::~ReportRangeSensorData()
{
}


// Writes message payload data to the packet at the current write position.
int ReportRangeSensorData::WriteMessageBody(Packet& packet) const
{
    unsigned int startPos = packet.GetWritePos();

    // Write count field
    UShort count = (UShort)mRangeSensorData.size();
    packet.Write(count);
    RangeScan::Map::const_iterator scan;
    for(scan = mRangeSensorData.begin();
        scan != mRangeSensorData.end();
        scan++)
    {
        scan->second.Write(packet);
    }

    return packet.GetWritePos() - startPos;
}

// Reads message payload data from the packets current read position.
int ReportRangeSensorData::ReadMessageBody(const Packet& packet)
{
    unsigned int startPos = packet.GetReadPos();

    // Write count field
    UShort count = 0;
    packet.Read(count);
    UShort id;
    RangeScan::Map::const_iterator scan;
    for(UShort s = 0;
        s < count;
        s++)
    {
        // Make sure we use the map to read the data so
        // we don't have to make a copy of the data
        // after reading.

        // Get the id
        unsigned int startReadPos = packet.GetReadPos();
        packet.Read(id);
        // Reset and read the whole thing now
        packet.SetReadPos(startReadPos);
        mRangeSensorData[id].Read(packet);
    }

    return packet.GetReadPos() - startPos;
}

/** Prints contents to console. */
void ReportRangeSensorData::PrintMessageBody() const
{
    RangeScan::Map::const_iterator scan;
    for(scan = mRangeSensorData.begin();
        scan != mRangeSensorData.end();
        scan++)
    {
        std::cout << "Sensor: " << scan->first 
            << " Points: " << scan->second.GetRangeDataList().size() << std::endl;
    }
}


bool ReportRangeSensorData::IsLargeDataSet(const unsigned int maxPayloadSize) const
{
    unsigned int minSize = 12;
    RangeScan::Map::const_iterator scan;
    for(scan = mRangeSensorData.begin();
        scan != mRangeSensorData.end();
        scan++)
    {
        minSize += scan->second.GetErrorMessage().size();
        unsigned int pointSize = 0;
        if(scan->second.GetRangeDataList().size() > 0)
        {
            // Only need to look at first point.
            Range point = *scan->second.GetRangeDataList().begin();
            UShort pv = point.GetPresenceVector();
            unsigned int pointSize = 14; // Base size
            if((pv & Range::PresenceVector::PointID) > 0)
            {
                pointSize += UINT_SIZE;
            }

            if((pv & Range::PresenceVector::RangeValidity) > 0)
            {
                pointSize += BYTE_SIZE;
            }

            if((pv & Range::PresenceVector::RangeErrorRMS) > 0)
            {
                pointSize += UINT_SIZE;
            }

            if((pv & Range::PresenceVector::BearingValidity) > 0)
            {
                pointSize += BYTE_SIZE;
            }

            if((pv & Range::PresenceVector::BearingErrorRMS) > 0)
            {
                pointSize += UINT_SIZE;
            }

            if((pv & Range::PresenceVector::InclinationValidity) > 0)
            {
                pointSize += BYTE_SIZE;
            }

            if((pv & Range::PresenceVector::InclinationErrorRMS) > 0)
            {
                pointSize += UINT_SIZE;
            }
        }
        minSize += scan->second.GetRangeDataList().size()*pointSize;
    }

    if( minSize > maxPayloadSize )
    {
        return true;
    }
    return false;
}

/** End of File */
