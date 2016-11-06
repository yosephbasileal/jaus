////////////////////////////////////////////////////////////////////////////////////
///
///  \file LocalVectorDriver.h
///  \brief This file contains the definition of the LocalVectorDriver class, used
///         as an interface for platform mobility.
///
///  <br>Author(s): Daniel Barber
///  <br>Created: March 14 2013
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
#ifndef __JAUS_MOBILITY_LOCAL_VECTOR_DRIVER__H
#define __JAUS_MOBILITY_LOCAL_VECTOR_DRIVER__H

#include "jaus/core/management/Management.h"
#include "jaus/mobility/drivers/SetLocalVector.h"
#include "jaus/mobility/drivers/QueryLocalVector.h"
#include "jaus/mobility/drivers/ReportLocalVector.h"
#include "jaus/mobility/sensors/VelocityStateSensor.h"
#include "jaus/mobility/sensors/LocalPoseSensor.h"

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class LocalVectorDriver
    ///   \brief The Local Vector Driver service performs closed loop control
    ///   as a function of desired heading, pitch, roll, and speed of the
    ///   platform. It gets data from both the Local Pose Sensor and Velocity State
    ///   Sensor.
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_MOBILITY_DLL LocalVectorDriver : public Management::Child
    {
    public:
        const static std::string Name; ///< String name of the Service.
        LocalVectorDriver();
        virtual ~LocalVectorDriver();
        // Method called when Resume command received, overload for additional behavior.
        virtual bool Resume() { return true; }
        // Method called when Reset command received, overload for additional behavior.
        virtual bool Reset()
        { 
            WriteLock wLock(mLocalVectorDriverMutex); 
            mLocalVectorCommand.ClearMessage(); 
            mLocalVectorCommandTime.SetCurrentTime();
            return true; 
        }
        // Method called when Standby command received, overload for additional behavior.
        virtual bool Standby() { return Reset(); }
        // Method called when Set Emergency command received, overload for additional behavior.
        virtual bool SetEmergency() { return Reset(); }
        // Method called when Clear Emergency command received, overload for additional behavior.
        virtual bool ClearEmergency() { return true; }
        // Method called when Release Control command received, overload for additional behavior.
        virtual bool ReleaseControl() { return Reset(); }
        ////////////////////////////////////////////////////////////////////////////////////
        ///
        ///   \brief Method called whenever a SetLocalVector is received.
        ///
        ///   Overload this method to be notified when a new command is received. Drive
        ///   commands are saved internally regardless and can be accessed using the
        ///   GetCurrentDriveCommand method.
        ///
        ///   \param[in] command Drive command to implement.
        ///
        ////////////////////////////////////////////////////////////////////////////////////
        virtual bool SetDriveCommand(const JAUS::SetLocalVector* command) {  return true; };
        JAUS::SetLocalVector GetCurrentDriveCommand() const;
        // Gets the time when the last drive command was received.
        virtual Time GetDriveCommandTime() const;
        // Generates service specific events.
        virtual bool GenerateEvent(const Events::Subscription& info) const;
        // Adds support for service specific events.
        virtual bool IsEventSupported(const Events::Type type,
                                      const double requestedPeriodicRate,
                                      const Message* queryMessage,
                                      double& confirmedPeriodicRate,
                                      std::string& errorMessage) const;
        // By default, this is discoverable to other components (overload to hide).
        virtual bool IsDiscoverable() const { return true; }
        // Method called whenever a message is received.
        virtual void Receive(const Message* message);
        // Creates messages associated with this service.
        virtual Message* CreateMessage(const UShort messageCode) const;  
        // Prints information about the service.
        virtual void PrintStatus() const;
    protected:
        SharedMutex mLocalVectorDriverMutex;          ///<  Mutex for thread protection of data.
    private:
        void CreateReportFromQuery(const QueryLocalVector* query, 
                                   ReportLocalVector& report) const;
        Time mLocalVectorCommandTime;                   ///<  Time when the last drive command was received.
        JAUS::SetLocalVector mLocalVectorCommand;          ///<  The last drive command received.
    };
}

#endif
/*  End of File */
