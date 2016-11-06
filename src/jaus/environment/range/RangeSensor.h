////////////////////////////////////////////////////////////////////////////////////
///
///  \file RangeSensor.h
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
#ifndef __JAUS_ENVIRONMENT_SENSING_RANGE_SENSOR__H
#define __JAUS_ENVIRONMENT_SENSING_RANGE_SENSOR__H

#include "jaus/core/events/Events.h"
#include "jaus/environment/range/QueryRangeSensorData.h"
#include "jaus/environment/range/QueryRangeSensorCompressedData.h"
#include "jaus/environment/range/ReportRangeSensorData.h"
#include "jaus/environment/range/ReportRangeSensorCompressedData.h"

#include <string>

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class RangeSensor
    ///   \brief The RangeSensor service allows you to share range scan information
    ///          from multiple devices with different capabilities.  Example
    ///          devices include LIDAR, or sonar, etc.
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_ENVIRONMENT_DLL RangeSensor : public Events::Child
    {
    public:
        static const std::string Name;   ///<  String name of the Service.
        // Constructor.
        RangeSensor();
        // Destructor.
        ~RangeSensor();
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
        void SetRangeScan(const UShort sensorID,
                          const Point3D& deviceLocation,
                          const Point3D& deviceOrientation,
                          const Point3D::List& scan,
                          const Time& timestamp = Time());
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
        ///   \param[in] scan The scan/range data collected.  The units are cartesian
        ///                   and in meters using local coordinate frame. In this frame
        ///                   X is positive in front of sensor, Y positive right, Z
        ///                   positive down.
        ///   \param[in] timestamp Time when the data was captured (UTC).
        ///
        ////////////////////////////////////////////////////////////////////////////////////
        void SetCartesianRangeScan(const UShort sensorID,
                                   const Point3D& deviceLocation,
                                   const Point3D& deviceOrientation,
                                   const Point3D::List& scan,
                                   const Time& timestamp = Time());
        // Set the current scan data from a device.
        void SetRangeScan(const RangeScan& scan);
        // Set both native and vehicle coordinate frame data for sensor.
        void SetRangeScans(const RangeScan& native, const RangeScan& vehicle);
        // Method called when an Event has been signaled, generates an Event message.
        virtual bool GenerateEvent(const Events::Subscription& info) const;
        // Method called to determine if an Event is supported by the service.
        virtual bool IsEventSupported(const Events::Type type,
                                      const double requestedPeriodicRate,
                                      const Message* queryMessage,
                                      double& confirmedPeriodicRate,
                                      std::string& errorMessage) const;
        // VisualSensor is always discoverable.
        virtual bool IsDiscoverable() const { return true; }
        // Processes messages associated with the VisualSensor Services.
        virtual void Receive(const Message* message);
        // Creates messages associated with the VisualSensor Service.
        virtual Message* CreateMessage(const UShort messageCode) const;
        // Prints range sensor status.
        virtual void PrintStatus() const;
        // Gets a copy of range sensor data.
        bool GetRangeScans(std::map<UShort, RangeScan>& scanData, 
            const Byte coordinateFrame = Range::VehicleCoordinate) const;
    protected:
        SharedMutex mRangeSensorMutex;                              ///<  Mutex for thread protection of data.
        std::map<UShort, std::pair<Point3D, Point3D> > mSensorPose; ///<  Pose of the sensors (if available).
        std::map<UShort, RangeScan> mNativeRangeScans;  ///<  Current range scan data for sensors, coordinates native to sensor.
        std::map<UShort, RangeScan> mVehicleRangeScans; ///<  Current range scan data for sensors, coordinates relative to vehicle.
        std::set<UShort> mUpdatedSensors;               ///<  Set of sensors that updated requiring event generation.
        ReportRangeSensorCompressedData mCompressedData;///<  Compressed range sensor data.
    };
}

#endif

/*  End of File */
