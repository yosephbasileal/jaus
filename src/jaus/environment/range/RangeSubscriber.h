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
#ifndef __JAUS_ENVIRONMENT_SENSING_RANGE_SUBSCRIBER__H
#define __JAUS_ENVIRONMENT_SENSING_RANGE_SUBSCRIBER__H

#include "jaus/core/management/Management.h"
#include "jaus/environment/range/QueryRangeSensorData.h"
#include "jaus/environment/range/QueryRangeSensorCompressedData.h"
#include "jaus/environment/range/ReportRangeSensorData.h"
#include "jaus/environment/range/ReportRangeSensorCompressedData.h"

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class RangeSubscriber
    ///   \brief This service is used to subscribe to range data from components
    ///          with the Range Sensor service.
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_ENVIRONMENT_DLL RangeSubscriber : public Management::Child
    {
    public:
        ////////////////////////////////////////////////////////////////////////////////////
        ///
        ///   \class Callback
        ///   \brief Callback class used to receive image/range data as it arrives.
        ///
        ////////////////////////////////////////////////////////////////////////////////////
        class JAUS_ENVIRONMENT_DLL CartesianCallback : public AccessControl::Callback
        {
        public:
            typedef std::set<CartesianCallback* > Set;
            CartesianCallback() {}
            virtual ~CartesianCallback() {}
            virtual void ProcessCartesianRangeScan(const JAUS::Point3D::List& scan, 
                                                   const JAUS::Address& sourceID, 
                                                   const JAUS::UShort sensorID,
                                                   const JAUS::Time& timestamp) {};
        };
        const static std::string Name; ///< String name of the Service.
        // Constructor.
        RangeSubscriber();
        // Destructor.
        ~RangeSubscriber();     
        // Shuts down the service.
        virtual void Shutdown();
        // Loads settings from XML for service.
        virtual bool LoadSettings(const std::string& xml);
        // Create a range subscription.
        bool CreateRangeSubscription(const Address& id, 
                                     const UShort sensorID);
        // Create a range subscription.
        bool CreateRangeSubscription(const Address& id, 
                                     const Byte compression,
                                     const UShort sensorID,
                                     const unsigned int waitTimeMs = Service::DefaultWaitMs);
        // Check to see if you have a range subscription.
        bool HaveRangeSubscription(const Address& id, const UShort sensorID = 0) const;
        // Cancel a range subscription.
        bool CancelRangeSubscription(const Address& id = Address(), const UShort sensorID = 0);
        // Register to receive updates of subsystems (add or removes callback).
        void RegisterCallback(CartesianCallback* callback, const bool add = true);
        // Method called when an Event has been signaled, generates an Event message.
        virtual bool GenerateEvent(const Events::Subscription& info) const { return false; }
        // Method called to determine if an Event is supported by the service.
        virtual bool IsEventSupported(const Events::Type type,
                                      const double requestedPeriodicRate,
                                      const Message* queryMessage,
                                      double& confirmedPeriodicRate,
                                      std::string& errorMessage) const { return false; }
        // RangeSubscriber doesn't need to be discovered.
        virtual bool IsDiscoverable() const { return true; }
        // Processes messages associated with the RangeSubscriber Services.
        virtual void Receive(const Message* message);
        // Creates messages associated with the RangeSubscriber Service.
        virtual Message* CreateMessage(const UShort messageCode) const;
        // Method called when transitioning to a ready state.
        virtual bool Resume() { return true; }
        // Method called to transition due to reset.
        virtual bool Reset() { return true; }
        // Method called when transitioning to a standby state.
        virtual bool Standby() { return true; }
        // Method called when transitioning to an emergency state.
        virtual bool SetEmergency() { return true; }
        // Method called when leaving the emergency state.
        virtual bool ClearEmergency() { return true; }
        // Method called when control is released.
        virtual bool ReleaseControl() { return true; }
        // Get the preferred default compression request type.
        Byte GetDefaultCompressionType() const { return mDefaultCompression; }
        // Sets the preferred default compression type.
        void SetDefaultCompressionType(const RangeScan::Compression type)
        {
            mDefaultCompression = (Byte)type;
        }
    private:
        Byte mDefaultCompression; ///< Default compression type.
        SharedMutex mRangeCallbacksMutex;       ///<  Mutex for thread protection of callback data.
        CartesianCallback::Set mRangeCallbacks; ///<  Range sensor callbacks.
    };
}

#endif
/*  End of File */
