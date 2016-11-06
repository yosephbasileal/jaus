////////////////////////////////////////////////////////////////////////////////////
///
///  \file QueryRangeSensorData.h
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
#ifndef __JAUS_ENVIRONMENT_SENSING_QUERY_RANGE_SENSOR_DATA__H
#define __JAUS_ENVIRONMENT_SENSING_QUERY_RANGE_SENSOR_DATA__H

#include "jaus/core/Message.h"
#include "jaus/environment/EnvironmentCodes.h"
#include "jaus/environment/range/RangeScan.h"

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class QueryRangeSensorData
    ///   \brief This method is used to report the range information from a sensor.
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_ENVIRONMENT_DLL QueryRangeSensorData : public Message
    {
    public:
        /** QueryRangeSensorData record.
        */
        class JAUS_ENVIRONMENT_DLL Sensor
        {
        public:
            typedef std::vector<Sensor> List;
            Sensor(const UShort sensorID = 0,
                   const Byte coordinateFrame = Range::NativeCoordinate,
                   const UShort presenceVector = 0,
                   const Byte compression = 0) :
            mSensorID(sensorID), 
                mCoordinateSystem(coordinateFrame), 
                mPresenceVector(presenceVector),
                mCompressionType(compression)
            {
            }
            Sensor(const Sensor& sensor)
            {
                *this = sensor;
            }
            Sensor& operator=(const Sensor& sensor)
            {
                mSensorID = sensor.mSensorID;
                mPresenceVector = sensor.mPresenceVector;
                mCoordinateSystem = sensor.mCoordinateSystem;
                mCompressionType = sensor.mCompressionType;
                return *this;
            }
            UShort mSensorID;       ///< Range sensor to request, 0 is all.
            UShort mPresenceVector; ///< Presence Vector for range data to report.
            Byte mCoordinateSystem; ///< Coordinate system to report data in.
            Byte mCompressionType;  ///< Compression type.
        };
        QueryRangeSensorData(const Address& dest = Address(), 
                   const Address& src = Address()) : Message(QUERY_RANGE_SENSOR_DATA, dest, src)
        {
        }
        QueryRangeSensorData(const QueryRangeSensorData& message)
             : Message(QUERY_RANGE_SENSOR_DATA, message.GetDestinationID(), message.GetSourceID())
        {
            *this = message;
        }
        ~QueryRangeSensorData() {}
        inline void AddSensor(const Sensor& sensor) { mSensors.push_back(sensor); }
        inline Sensor::List& GetSensors() { return mSensors; }
        inline const Sensor::List& GetSensors() const { return mSensors; }
        // Return true if a command message, otherwise false.
        virtual bool IsCommand() const { return false; }
        // Writes message payload data to the packet at the current write position.
        virtual int WriteMessageBody(Packet& packet) const
        {
            unsigned int startPos = packet.GetWritePos();
            UShort count = (UShort)mSensors.size();
            packet.Write(count);
            Sensor::List::const_iterator s;
            for(s = mSensors.begin();
                s != mSensors.end();
                s++)
            {
                packet.Write(s->mSensorID);
                packet.Write(s->mCoordinateSystem);
                packet.Write(s->mPresenceVector);
            }
            return packet.GetWritePos() - startPos;
        }
        // Reads message payload data from the packets current read position.
        virtual int ReadMessageBody(const Packet& packet)
        {
            unsigned int startPos = packet.GetReadPos();
            UShort count = 0;
            packet.Read(count);
            for(int i = 0; i < (int)count; i++)
            {
                UShort sensorID = 0;
                Byte coordinateFrame = 0;
                UShort presenceVector = 0;
                packet.Read(sensorID);
                packet.Read(coordinateFrame);
                packet.Read(presenceVector);
                mSensors.push_back(Sensor(sensorID, coordinateFrame, presenceVector));
            }
            return packet.GetReadPos() - startPos;
        }
        // Make a copy of the message and returns pointer to it.
        virtual Message* Clone() const { return new QueryRangeSensorData(*this); }
        // Gets the presence vector data for the message.
        virtual UInt GetPresenceVector() const { return 0; }
        // Get the size of the message presence vector. 0 value indicates no presence vector.
        virtual UInt GetPresenceVectorSize() const { return 0; }
        // Get the mask associated with presence vector.  Indicates what bits are used.
        virtual UInt GetPresenceVectorMask() const { return 0x00; }
        // Gets the response type to the associated message, 0 if message is a response type.
        virtual UShort GetMessageCodeOfResponse() const { return REPORT_RANGE_SENSOR_DATA; }
        // Gets the name of the message in human readable format (for logging, etc.)
        virtual std::string GetMessageName() const { return "Query Range Sensor Data"; }
        // Clears only message body information.
        virtual void ClearMessageBody() 
        { 
            mSensors.clear();
        }
        // Return true if payload is greater than maxPaylodSize (payload == message data only).
        virtual bool IsLargeDataSet(const unsigned int maxPayloadSize) const 
        { 
            unsigned int totalSize = USHORT_SIZE + 
                mSensors.size()*(USHORT_SIZE*2 + BYTE_SIZE);
            return totalSize < maxPayloadSize;
        }
        // Prints only message body information.
        virtual void PrintMessageBody() const {}
        QueryRangeSensorData& operator=(const QueryRangeSensorData& message)
        {
            CopyHeaderData(&message);
            mSensors = message.mSensors;
            return *this;
        }
    protected:
        Sensor::List mSensors;  ///< Sensors to get data from.
    };
}

#endif
/*  End of File */
