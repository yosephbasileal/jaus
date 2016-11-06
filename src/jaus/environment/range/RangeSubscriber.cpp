////////////////////////////////////////////////////////////////////////////////////
///
///  \file rangesensor.h
///  \brief Contains the Range Subscriber service.
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
#include "jaus/environment/range/RangeSubscriber.h"
#include "jaus/core/Component.h"
#include <cxutils/math/CxMath.h>
#include <cxutils/xml/XmlConfig.h>

using namespace JAUS;

const std::string RangeSubscriber::Name = "urn:jaus:jss:jpp:customService:RangeSubscriber";


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////////
RangeSubscriber::RangeSubscriber() : Management::Child(Service::ID(RangeSubscriber::Name),
                                                       Service::ID(Management::Name))
{
    mDefaultCompression = (Byte)RangeScan::bzip2;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
RangeSubscriber::~RangeSubscriber()
{
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Method called on Shutdown.
///
////////////////////////////////////////////////////////////////////////////////////
void RangeSubscriber::Shutdown()
{
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Loads UDP configuration values from an XML file.  Only loads values
///          if transport has not been initialized.
///
///   \param[in] filename File containing Service settings data.
///
///   \return True on success, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool RangeSubscriber::LoadSettings(const std::string& filename)
{
    CxUtils::XmlConfig xml;
    if(xml.LoadFile(filename))
    {
        std::string compression = "bzip2";
        xml.GetValue(this->GetServiceID().ToString() + ".CompressionType", compression);
        mDefaultCompression = FromStringToCompression(compression);
        return true;
    }

    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Create a subscription to range data. Requests the default
///          compression type.
///
///   \param[in] id The component ID to get video data from.
///   \param[in] sensorID The sensor/source on the component.
///
///   \return True on success, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool RangeSubscriber::CreateRangeSubscription(const Address& id, 
                                              const UShort sensorID)
{
    return CreateRangeSubscription(id,
                                   mDefaultCompression,
                                   sensorID,
                                   Service::DefaultWaitMs);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Create a subscription to range data.
///
///   \param[in] id The component ID to get video data from.
///   \param[in] sensorID The sensor/source on the component.
///   \param[in] compression Compression types (as of 1/13 only none or DEFLATE
///                          supported). 
///   \param[in] waitTimeMs How long to wait in ms before timeout on request.
///
///   \return True on success, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool RangeSubscriber::CreateRangeSubscription(const Address& id, 
                                              const Byte compression,
                                              const UShort sensorID,
                                              const unsigned int waitTimeMs)
{
    if(compression > 0)
    {
        QueryRangeSensorCompressedData queryEvent;
        queryEvent.AddSensor(QueryRangeSensorData::Sensor(sensorID, Range::VehicleCoordinate, 0, compression));
        return EventsService()->RequestEveryChangeEvent(id, &queryEvent, waitTimeMs);
    }
    else
    {
        QueryRangeSensorData queryEvent;
        queryEvent.AddSensor(QueryRangeSensorData::Sensor(sensorID, Range::VehicleCoordinate, 0));
        return EventsService()->RequestEveryChangeEvent(id, &queryEvent, waitTimeMs);
    }
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Method to check if a video subscription is present.
///
///   \param[in] id ID of the source of the subscription.
///   \param[in] sensorID ID of the sensor, set to -1 for any sensor.
///
///   \return True if a range subscription exists, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
bool RangeSubscriber::HaveRangeSubscription(const Address& id,
                                            const UShort sensorID) const
{
    bool result = false;
    Events::Subscription::List list = GetComponent()->EventsService()->GetSubscriptions(id, REPORT_RANGE_SENSOR_DATA);
    Events::Subscription::List::iterator s;
    for(s = list.begin();
        s != list.end();
        s++)
    {
        if(sensorID == 0)
        {
            result = true;
            break;
        }
        else
        {
            QueryRangeSensorData* query 
                = static_cast<QueryRangeSensorData *>(s->mpQueryMessage);

            if(query)
            {
                QueryRangeSensorData::Sensor::List::iterator s;
                for(s = query->GetSensors().begin();
                    s != query->GetSensors().end();
                    s++)
                {
                    if(s->mSensorID == (Byte)sensorID)
                    {
                        result = true;
                        break;
                    }
                }
            }

            QueryRangeSensorCompressedData* cquery 
                = static_cast<QueryRangeSensorCompressedData *>(s->mpQueryMessage);

            if(cquery)
            {
                QueryRangeSensorData::Sensor::List::iterator s;
                for(s = cquery->GetSensors().begin();
                    s != cquery->GetSensors().end();
                    s++)
                {
                    if(s->mSensorID == (Byte)sensorID)
                    {
                        result = true;
                        break;
                    }
                }
            }
        }
    }
 
    return result;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Method to check cancel a range subscription.
///
///   \param[in] id ID of the source of the subscription.
///   \param[in] sensorID ID of the sensor, set to -1 for any sensor.
///
///   \return True if a range subscription exists, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
bool RangeSubscriber::CancelRangeSubscription(const Address& id,
                                              const UShort sensorID)
{
    bool result = false;

    Events::Subscription::List list = GetComponent()->EventsService()->GetSubscriptions(id, REPORT_RANGE_SENSOR_DATA);
    Events::Subscription::List compressed = GetComponent()->EventsService()->GetSubscriptions(id, REPORT_RANGE_SENSOR_COMPRESSED_DATA);
    list.insert(list.end(), compressed.begin(), compressed.end());
    Events::Subscription::List::iterator s;
    for(s = list.begin();
        s != list.end();
        s++)
    {
        UShort reportType = REPORT_RANGE_SENSOR_DATA;
        if(s->mpQueryMessage->GetMessageCode() == QUERY_RANGE_SENSOR_COMPRESSED_DATA)
        {
            reportType = REPORT_RANGE_SENSOR_COMPRESSED_DATA;
        }

        if(sensorID == 0)
        {
            if(GetComponent()->EventsService()->CancelSubscription(s->mProducer,
                                                                   reportType,
                                                                   s->mID,
                                                                   Service::DefaultWaitMs))
            {
                result = true;
            }
        }
        else
        {
            QueryRangeSensorData* query = static_cast<QueryRangeSensorData *>(s->mpQueryMessage);
            QueryRangeSensorCompressedData* cquery = static_cast<QueryRangeSensorCompressedData *>(s->mpQueryMessage);
            QueryRangeSensorData::Sensor::List sensors;
            if(query)
            {
                sensors = query->GetSensors();
            }
            else if(cquery)
            {
                sensors = cquery->GetSensors();
            }
            QueryRangeSensorData::Sensor::List::iterator sensor;
            for(sensor = sensors.begin();
                sensor != sensors.end();
                sensor++)
            {
                if(sensor->mSensorID == (Byte)sensorID)
                {
                    if(GetComponent()->EventsService()->CancelSubscription(s->mProducer,
                                                                       reportType,
                                                                       s->mID,
                                                                       Service::DefaultWaitMs))
                    {
                        result = true;
                        break;
                    }
                }
            }
            if(result)
            {
                break;
            }
        }
    }

    return result;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Method to register a callback to receive range data as it arrives.
///
///   \param[in] callback Pointer to callback to add/remove.
///   \param[in] add If true, callback is added, if false, it is removed.
///
////////////////////////////////////////////////////////////////////////////////////
void RangeSubscriber::RegisterCallback(CartesianCallback* callback, const bool add)
{
    WriteLock lock(mRangeCallbacksMutex);
    if(add)
    {
        mRangeCallbacks.insert(callback);
    }
    else
    {
        CartesianCallback::Set::iterator cb;

        cb = mRangeCallbacks.find(callback);
        if(cb != mRangeCallbacks.end())
        {
            mRangeCallbacks.erase(cb);
        }
    }
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Processes message received by the Service.  If not supported, then
///          message is passed to inheriting services.
///
///   This Service supports the following message: Report Image
///
///   \param[in] message Message data to process.
///
////////////////////////////////////////////////////////////////////////////////////
void RangeSubscriber::Receive(const Message* message)
{
    switch(message->GetMessageCode())
    {
    case REPORT_RANGE_SENSOR_COMPRESSED_DATA:
    case REPORT_RANGE_SENSOR_DATA:
        {
            const ReportRangeSensorData* report = 
                    static_cast<const ReportRangeSensorData*>(message);
            if(report)
            {
                bool haveCartesianCallbacks = false;
                {
                    ReadLock lock(mRangeCallbacksMutex);
                    haveCartesianCallbacks = mRangeCallbacks.size() > 0 ? true : false;
                }

                if(haveCartesianCallbacks)
                {
                    Point3D::List cartesian;
                    RangeScan::Map* sensors = ((ReportRangeSensorData*)report)->GetRangeSensorDataPtr();
                    RangeScan::Map::iterator sensor;
                    for(sensor = sensors->begin();
                        sensor != sensors->end();
                        sensor++)
                    {
                        if(sensor->second.GetCoordinateSystem() != Range::VehicleCoordinate)
                        {
                            continue;
                        }
                        cartesian.clear();
                        Range::List* scan = sensor->second.GetRangeDataListPtr();
                        Range::List::iterator point;
                        for(point = scan->begin();
                            point != scan->end();
                            point++)
                        {
                            cartesian.push_back( Range::ConvertToCartesian( point->GetRangePoint() ) );
                        }

                        ReadLock cbLock(mRangeCallbacksMutex);
                        CartesianCallback::Set::iterator cb;
                        for(cb = mRangeCallbacks.begin();
                            cb != mRangeCallbacks.end();
                            cb++)
                        {
                            (*cb)->ProcessCartesianRangeScan(cartesian,
                                                             report->GetSourceID(),
                                                             sensor->second.GetSensorID(),
                                                             sensor->second.GetTimeStamp());
                        }
                    }
                }
            }
        }
        break;
    case REPORT_RANGE_SENSOR_CONFIGURATION:
        {
            /*const ReportRangeSensorConfiguration* report = 
                dynamic_cast<const ReportRangeSensorConfiguration*>(message);
            if(report)
            {
                Mutex::ScopedLock lock(&mRangeSensorMutex);
                RangeSensorConfig::List::const_iterator c;
                for(c = report->GetConfiguration()->begin();
                    c != report->GetConfiguration()->end();
                    c++)
                {
                    mRangeSensors[report->GetSourceID()][c->mID] = *c;
                }
            }*/
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
Message* RangeSubscriber::CreateMessage(const UShort messageCode) const
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


/*  End of File */
