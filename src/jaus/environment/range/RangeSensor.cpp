////////////////////////////////////////////////////////////////////////////////////
///
///  \file rangesensor.h
///  \brief Contains the Range Sensor Service implementation.
///
///  <br>Author(s): Daniel Barber
///  Created: 2013
///  Copyright (c) 2013
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
#include "jaus/environment/range/RangeSensor.h"
#include <cxutils/math/CxMath.h>

using namespace JAUS;


const std::string RangeSensor::Name = "urn:jaus:jss:environmentSensing:RangeSensor";


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////////
RangeSensor::RangeSensor() : Events::Child(Service::ID(RangeSensor::Name), Service::ID(Events::Name))
{
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
RangeSensor::~RangeSensor()
{
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Set the current scan collected/received from the device.
///
///   \param[in] deviceID Device ID number.
///   \param[in] deviceLocation Location of sensor on vehicle relative to
///                             vehicle origin when the scan was performed (meters).
///                             Values are in the platform coordinate frame (e.g.
///                             positive x is in front of vehicle).
///   \param[in] deviceOrientation Orientation of the device (x = roll, y = pitch,
///                                z = yaw), when the scan was performed.
///   \param[in] scan The scan/range data collected.  The units are in polar
///                    coordinates, with X = range, Y = inclination (positive being
///                    above sensor), and Z the bearing (positive to the right).
///                    Range is in meters, and bearing and inclination are in
///                    radians.
///   \param[in] timestamp Time when the data was captured (UTC).
///
////////////////////////////////////////////////////////////////////////////////////
void RangeSensor::SetRangeScan(const UShort deviceID, 
                               const Point3D &deviceLocation, 
                               const Point3D &deviceOrientation, 
                               const Point3D::List& scan,
                               const Time& timestamp)
{
    // MUTEX
    {
        WriteLock lock(mRangeSensorMutex);

        mSensorPose[deviceID].first = deviceLocation;
        mSensorPose[deviceID].second = deviceOrientation;

        mNativeRangeScans[deviceID].SetSensorID(deviceID);
        mNativeRangeScans[deviceID].SetCoordinateSystem(Range::NativeCoordinate);
        mVehicleRangeScans[deviceID].SetCoordinateSystem(Range::VehicleCoordinate);
        mVehicleRangeScans[deviceID].SetSensorID(deviceID);

        mNativeRangeScans[deviceID].SetTimeStamp(timestamp);
        mVehicleRangeScans[deviceID].SetTimeStamp(timestamp);

        Range::List* native = mNativeRangeScans[deviceID].GetRangeDataListPtr();
        Range::List* vehicle = mVehicleRangeScans[deviceID].GetRangeDataListPtr();

        native->clear();
        vehicle->clear();

        Point3D::List::const_iterator point;
        for(point = scan.begin();
            point != scan.end();
            point++)
        {
            // Native points
            Point3D nativePoint(*point);
            native->push_back(Range(nativePoint));
            vehicle->push_back(Range( Range::ConvertToVehicleCoordinates(*point, 
                            deviceOrientation, 
                            deviceLocation)));
        }

        // For testing/debugging
        //double startTimeSeconds;
        //double endTimeSeconds;
        //double zLibTimeSeconds, bzipLibTimeSeconds, lzmaLibTimeSeconds;
        //unsigned int zLibSizeBytes;
        //unsigned int bzLibSizeBytes;
        //unsigned int lzmaLibSizeBytes;
        //CxUtils::Packet zlibTest;
        //CxUtils::Packet bz2libTest;
        //CxUtils::Packet lzmaLibTest;
        //zlibTest.Reserve(100000);
        //bz2libTest.Reserve(100000);
        //lzmaLibTest.Reserve(100000);

        //startTimeSeconds = CxUtils::Timer::GetTimeSeconds();
        //mNativeRangeScans[deviceID].Write(zlibTest, RangeScan::DEFLATE);
        //mNativeRangeScans[deviceID].Clear();
        //mNativeRangeScans[deviceID].Read(zlibTest, true);
        //endTimeSeconds = CxUtils::Timer::GetTimeSeconds();
        //zLibTimeSeconds = endTimeSeconds - startTimeSeconds;
        //zLibSizeBytes = zlibTest.Length();

        //startTimeSeconds = CxUtils::Timer::GetTimeSeconds();
        //mNativeRangeScans[deviceID].Write(bz2libTest, RangeScan::bzip2);
        //mNativeRangeScans[deviceID].Clear();
        //mNativeRangeScans[deviceID].Read(bz2libTest, true);
        //endTimeSeconds = CxUtils::Timer::GetTimeSeconds();
        //bzipLibTimeSeconds = endTimeSeconds - startTimeSeconds;
        //bzLibSizeBytes = bz2libTest.Length();

        //startTimeSeconds = CxUtils::Timer::GetTimeSeconds();
        //mNativeRangeScans[deviceID].Write(lzmaLibTest, RangeScan::LZMA);
        //mNativeRangeScans[deviceID].Clear();
        //mNativeRangeScans[deviceID].Read(lzmaLibTest, true);
        //endTimeSeconds = CxUtils::Timer::GetTimeSeconds();
        //lzmaLibTimeSeconds = endTimeSeconds - startTimeSeconds;
        //lzmaLibSizeBytes = lzmaLibTest.Length();

        mUpdatedSensors.insert(deviceID);
    } 
    // MUTEX

    SignalEvent(REPORT_RANGE_SENSOR_DATA);
    SignalEvent(REPORT_RANGE_SENSOR_COMPRESSED_DATA);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Set the current scan collected/received from the device.
///
///   \param[in] deviceID Device ID number.
///   \param[in] deviceLocation Location of sensor on vehicle relative to
///                             vehicle origin when the scan was performed (meters).
///                             Values are in the platform coordinate frame (e.g.
///                             positive x is in front of vehicle).
///   \param[in] deviceOrientation Orientation of the device (x = roll, y = pitch,
///                                z = yaw), when the scan was performed.
///   \param[in] scan The scan/range data collected.  The units are Cartesian
///                   and in meters using local coordinate frame. In this frame
///                   X is positive in front of sensor, Y positive right, Z
///                   positive down.
///   \param[in] timestamp Time when the data was captured (UTC).
///
////////////////////////////////////////////////////////////////////////////////////
void RangeSensor::SetCartesianRangeScan(const UShort deviceID,
                               const Point3D &deviceLocation,
                               const Point3D &deviceOrientation,
                               const Point3D::List& scan,
                               const Time& timestamp)
{
    // MUTEX
    {
        WriteLock lock(mRangeSensorMutex);

        mSensorPose[deviceID].first = deviceLocation;
        mSensorPose[deviceID].second = deviceOrientation;

        mNativeRangeScans[deviceID].SetSensorID(deviceID);
        mNativeRangeScans[deviceID].SetCoordinateSystem(Range::NativeCoordinate);
        mVehicleRangeScans[deviceID].SetCoordinateSystem(Range::VehicleCoordinate);
        mVehicleRangeScans[deviceID].SetSensorID(deviceID);

        mNativeRangeScans[deviceID].SetTimeStamp(timestamp);
        mVehicleRangeScans[deviceID].SetTimeStamp(timestamp);

        Range::List* native = mNativeRangeScans[deviceID].GetRangeDataListPtr();
        Range::List* vehicle = mVehicleRangeScans[deviceID].GetRangeDataListPtr();

        native->clear();
        vehicle->clear();

        Point3D::List::const_iterator point;
        for(point = scan.begin();
            point != scan.end();
            point++)
        {
            // Native points
            Point3D nativePoint(Range::ConvertToSpherical(*point));
            native->push_back(Range(nativePoint));
            vehicle->push_back(Range( Range::ConvertToVehicleCoordinates(nativePoint,
                            deviceOrientation,
                            deviceLocation)));
        }

        mUpdatedSensors.insert(deviceID);
    }
    // MUTEX

    SignalEvent(REPORT_RANGE_SENSOR_DATA);
    SignalEvent(REPORT_RANGE_SENSOR_COMPRESSED_DATA);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Set the current scan collected/received from the device.
///
///   \param[in] scan Range Sensor data for a sensor. Make sure sensor ID is 
///                   set to non-zero.
///
////////////////////////////////////////////////////////////////////////////////////
void RangeSensor::SetRangeScan(const RangeScan& scan)
{
    
    // MUTEX
    {
        WriteLock lock(mRangeSensorMutex);
        if(scan.GetCoordinateSystem() == Range::VehicleCoordinate)
        {
            mVehicleRangeScans[scan.GetSensorID()] = scan;
        }
        else
        {
            mNativeRangeScans[scan.GetSensorID()] = scan;
        }
        mUpdatedSensors.insert(scan.GetSensorID());
    } 
    // MUTEX

    SignalEvent(REPORT_RANGE_SENSOR_DATA);
    SignalEvent(REPORT_RANGE_SENSOR_COMPRESSED_DATA);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Set the current scan collected/received from the device.
///
///   \param[in] native Range Sensor data for a sensor. Make sure sensor ID is 
///                   set to non-zero. This data should be relative to the sensor.
///   \param[in] native Range Sensor data for a sensor. Make sure sensor ID is 
///                   set to non-zero. This data should be relative to the vehicle.
///
////////////////////////////////////////////////////////////////////////////////////
void RangeSensor::SetRangeScans(const RangeScan& native, const RangeScan& vehicle)
{
    
    // MUTEX
    {
        WriteLock lock(mRangeSensorMutex);
        mVehicleRangeScans[vehicle.GetSensorID()] = vehicle;
        mNativeRangeScans[native.GetSensorID()] = native;
        mVehicleRangeScans[vehicle.GetSensorID()].SetCoordinateSystem(Range::VehicleCoordinate);
        mNativeRangeScans[vehicle.GetSensorID()].SetCoordinateSystem(Range::NativeCoordinate);
        mUpdatedSensors.insert(native.GetSensorID());
        mUpdatedSensors.insert(vehicle.GetSensorID());
    } 
    // MUTEX

    SignalEvent(REPORT_RANGE_SENSOR_DATA);
    SignalEvent(REPORT_RANGE_SENSOR_COMPRESSED_DATA);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Generates an event for the given information.
///
///   \param[in] info The event information (ID, Sequence #, etc.) for generation.
///
///   \return True if event generated, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
bool RangeSensor::GenerateEvent(const Events::Subscription& info) const
{
    if(info.mpQueryMessage->GetMessageCode() == QUERY_RANGE_SENSOR_DATA ||
        info.mpQueryMessage->GetMessageCode() == QUERY_RANGE_SENSOR_COMPRESSED_DATA)
    {
        ReportRangeSensorData report;
        ReportRangeSensorCompressedData compressed;
        ReportRangeSensorCompressedData* compressedReport = 
            (ReportRangeSensorCompressedData*)&compressed;
        bool haveData = false;
        {
            WriteLock lock(*((SharedMutex *)&mRangeSensorMutex));

            QueryRangeSensorData* query = NULL;
            QueryRangeSensorCompressedData* compressedQuery = NULL;

            QueryRangeSensorData::Sensor::List sensors;
            QueryRangeSensorData::Sensor::List::const_iterator sensor;

            if(info.mpQueryMessage->GetMessageCode() == QUERY_RANGE_SENSOR_DATA)
            {
                query = (QueryRangeSensorData*)info.mpQueryMessage;
                sensors = query->GetSensors();
            }
            else 
            {
                compressedQuery = (QueryRangeSensorCompressedData*)info.mpQueryMessage;
                sensors = compressedQuery->GetSensors();
            }

            for(sensor = sensors.begin();
                sensor != sensors.end();
                sensor++)
            {
                std::map<UShort, RangeScan>::const_iterator scan;

                std::set<UShort>::const_iterator update;
                for(update = mUpdatedSensors.begin();
                    update != mUpdatedSensors.end();
                    update++)
                {
                    if(sensor->mSensorID == 0 || sensor->mSensorID == *update)
                    {
                        bool haveScan = false;
                        if(sensor->mCoordinateSystem == Range::NativeCoordinate)
                        {
                            scan = mNativeRangeScans.find(*update);
                            if(scan != mNativeRangeScans.end())
                            {
                                haveScan = true;
                            }
                        }
                        else
                        {
                            scan = mVehicleRangeScans.find(*update);
                            if(scan != mVehicleRangeScans.end())
                            {
                                haveScan = true;
                            }
                            else
                            {
                                scan = mNativeRangeScans.find(*update);
                                if(scan != mNativeRangeScans.end())
                                {
                                    haveScan = true;
                                }
                            }
                        }
                        // If I have scan data, add it to the message for
                        // transmission.
                        if(haveScan)
                        {
                            haveData = true;
                            if(query)
                            {
                                report.GetRangeSensorData()[*update] = scan->second;
                            }
                            else
                            {
                                compressedReport->GetRangeSensorData()[*update] = scan->second;
                                compressedReport->SetCompressionType(sensor->mCompressionType);
                            }
                            if(sensor->mPresenceVector > 0)
                            {
                                // Remove unwanted data.
                                Range::List::iterator point;
                                Range::List* scan;
                                if(query)
                                {
                                    scan = report.GetRangeSensorData()[*update].GetRangeDataListPtr();
                                }
                                else
                                {
                                    scan = compressedReport->GetRangeSensorData()[*update].GetRangeDataListPtr();
                                }
                                report.GetRangeSensorData()[*update].GetRangeDataListPtr();
                                for(point = scan->begin();
                                    point != scan->end();
                                    point++)
                                {
                                    point->SetPresenceVector((point->GetPresenceVector() & query->GetPresenceVector()));
                                }
                            }
                        }
                    }
                }
            }
            ((std::set<UShort>*)&mUpdatedSensors)->clear();
        }
        // Send event.
        if(haveData)
        {
            if(info.mpQueryMessage->GetMessageCode() == QUERY_RANGE_SENSOR_DATA)
            {
                SendEvent(info, &report);
            }
            else
            {
                SendEvent(info, compressedReport);
            }
            return true;
        }
    }
    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Checks if the event is supported by the Service.
///
///   \param[in] type The event type (Periodic/EveryChange).
///   \param[in] requestedPeriodicRate If type == Periodic, then this is the
///                                    desired update rate.
///   \param[in] queryMessage The query message associated with the event.
///   \param[out] confirmedPeriodicRate This is the confirmed periodic rate 
///                                     supported by the Service.
///   \param[out] errorMessage If not supported, this is an optional error message.
///
///   \return True if event supported, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
bool RangeSensor::IsEventSupported(const Events::Type type,
                                   const double requestedPeriodicRate,
                                   const Message* queryMessage,
                                   double& confirmedPeriodicRate,
                                   std::string& errorMessage) const
{
    if(queryMessage->GetMessageCode() == QUERY_RANGE_SENSOR_DATA ||
       queryMessage->GetMessageCode() == QUERY_RANGE_SENSOR_COMPRESSED_DATA)
    {
        confirmedPeriodicRate = requestedPeriodicRate;
        return true;
    }

    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Processes message received by the Service.  If not supported, then
///          message is passed to inheriting services depending on what
///          type of control has been established for the component.
///
///   This Service supports PrimitiveDriver related messages only.
///
///   \param[in] message Message data to process.
///
////////////////////////////////////////////////////////////////////////////////////
void RangeSensor::Receive(const JAUS::Message *message)
{
    switch(message->GetMessageCode())
    {
    case QUERY_RANGE_SENSOR_DATA:
    case QUERY_RANGE_SENSOR_COMPRESSED_DATA:
        {
            ReportRangeSensorData report;
            ReportRangeSensorCompressedData compressed;
            ReportRangeSensorCompressedData* compressedReport = 
                (ReportRangeSensorCompressedData*)&compressed;
            bool haveData = false;

            QueryRangeSensorData* query = NULL;
            QueryRangeSensorCompressedData* compressedQuery = NULL;

            QueryRangeSensorData::Sensor::List sensors;
            QueryRangeSensorData::Sensor::List::const_iterator sensor;

            if(message->GetMessageCode() == QUERY_RANGE_SENSOR_DATA)
            {
                query = (QueryRangeSensorData*)message;
                sensors = query->GetSensors();
            }
            else 
            {
                compressedQuery = (QueryRangeSensorCompressedData*)message;
                sensors = compressedQuery->GetSensors();
            }

            for(sensor = sensors.begin();
                sensor != sensors.end();
                sensor++)
            {
                std::map<UShort, RangeScan>::const_iterator scan;

                bool haveScan = false;
                if(sensor->mCoordinateSystem == Range::NativeCoordinate)
                {
                    scan = mNativeRangeScans.find(sensor->mSensorID);
                    if(scan != mNativeRangeScans.end())
                    {
                        haveScan = true;
                    }
                }
                else
                {
                    scan = mVehicleRangeScans.find(sensor->mSensorID);
                    if(scan != mVehicleRangeScans.end())
                    {
                        haveScan = true;
                    }
                    else
                    {
                        // Don't have it transformed, so default to
                        // relative to sensor.
                        scan = mNativeRangeScans.find(sensor->mSensorID);
                        if(scan != mNativeRangeScans.end())
                        {
                            haveScan = true;
                        }
                    }
                }
                // If I have scan data, add it to the message for
                // transmission.
                if(haveScan)
                {
                    haveData = true;
                    if(query)
                    {
                        report.GetRangeSensorData()[sensor->mSensorID] = scan->second;
                    }
                    else
                    {
                        compressedReport->GetRangeSensorData()[sensor->mSensorID] = scan->second;
                        compressedReport->SetCompressionType(sensor->mCompressionType);
                    }
                    if(sensor->mPresenceVector > 0)
                    {
                        // Remove unwanted data.
                        Range::List::iterator point;
                        Range::List* scan;
                        if(query)
                        {
                            scan = report.GetRangeSensorData()[sensor->mSensorID].GetRangeDataListPtr();
                        }
                        else
                        {
                            scan = compressedReport->GetRangeSensorData()[sensor->mSensorID].GetRangeDataListPtr();
                        }
                        report.GetRangeSensorData()[sensor->mSensorID].GetRangeDataListPtr();
                        for(point = scan->begin();
                            point != scan->end();
                            point++)
                        {
                            point->SetPresenceVector((point->GetPresenceVector() & query->GetPresenceVector()));
                        }
                    }
                }
            }

            // Send event.
            if(haveData)
            {
                if(message->GetMessageCode() == QUERY_RANGE_SENSOR_DATA)
                {
                    Send(&report);
                }
                else
                {
                    Send(compressedReport);
                }
            }
        }
        break;
    case QUERY_RANGE_SENSOR_CONFIGURATION:
        {
            // Deprecated code ported from JAUS EXTRAS
            //ReportRangeSensorConfiguration report(message->GetSourceID(), GetComponentID());
            //Mutex::ScopedLock lock(&mRangeSensorMutex);
            //RangeSensorConfig::Map::iterator config;
            //for(config = mRangeSensors.begin();
            //    config != mRangeSensors.end();
            //    config++)
            //{
            //    report.GetConfiguration()->push_back(config->second);
            //}
            //Send(&report);
        }
        break;
    default:
        break;
    }
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Attempts to create the message desired.  Only message supported
///          by this Service can be created by this Service.
///
///   \param[in] messageCode Message to create.
///
///   \return Pointer to newly allocated Message data, NULL if message is not
///           supported by the Service.
///
////////////////////////////////////////////////////////////////////////////////////
Message* RangeSensor::CreateMessage(const UShort messageCode) const
{
    Message* message = NULL;
    switch(messageCode)
    {
    case QUERY_RANGE_SENSOR_DATA:
        message = new QueryRangeSensorData();
        break;
    case QUERY_RANGE_SENSOR_CONFIGURATION:
        //message = new QueryRangeSensorConfiguration();
        break;
    case QUERY_RANGE_SENSOR_CAPABILITIES:
        break;
    case QUERY_RANGE_SENSOR_COMPRESSED_DATA:
        message = new QueryRangeSensorCompressedData();
        break;
    case QUERY_SENSOR_GEOMETRIC_PROPERTIES:
        break;
    case REPORT_RANGE_SENSOR_DATA:
        message = new ReportRangeSensorData();
        break;
    case REPORT_RANGE_SENSOR_CONFIGURATION:
        //message = new QueryRangeSensorConfiguration();
        break;
    case REPORT_RANGE_SENSOR_CAPABILITIES:
        break;
    case REPORT_RANGE_SENSOR_COMPRESSED_DATA:
        message = new ReportRangeSensorCompressedData();
        break;
    case REPORT_SENSOR_GEOMETRIC_PROPERTIES:
        break;
    default:
        message = NULL;
        break;
    };
    return message;
}

/** Prints status of sensor. */
void RangeSensor::PrintStatus() const
{
    ReadLock rLock(*((SharedMutex*)&mRangeSensorMutex));
    std::map<UShort, RangeScan>::const_iterator scan;
    for(scan = mNativeRangeScans.begin();
        scan != mNativeRangeScans.end();
        scan++)
    {
        std::cout << "\tRange Sensor "
                  << scan->first
                  << " Data Count: "
                  << scan->second.GetRangeDataList().size()
                  << std::endl;
    }
}


bool RangeSensor::GetRangeScans(std::map<UShort, RangeScan>& scanData, 
                                const Byte coordinateFrame) const
{
    scanData.clear();
    {
        ReadLock rLock(*((SharedMutex*)&mRangeSensorMutex));
        if(coordinateFrame == Range::VehicleCoordinate)
        {
            scanData = mVehicleRangeScans;
        }
        else
        {
            scanData = mNativeRangeScans;
        }
    }
    return scanData.size() > 0 ? true : false;
}


/*  End of File */
