////////////////////////////////////////////////////////////////////////////////////
///
///  \file GlobalVectorDriver.cpp
///  \brief This file contains the implementation of the GlobalVectorDriver class.
///
///  <br>Author(s): Daniel Barber
///  <br>Created: March 23 2011
///  <br>Copyright (c) 2011
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
#include "jaus/mobility/drivers/GlobalVectorDriver.h"
#include "jaus/core/Component.h"

using namespace JAUS;

const std::string GlobalVectorDriver::Name = "urn:jaus:jss:mobility:GlobalVectorDriver";

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////////
GlobalVectorDriver::GlobalVectorDriver() : Management::Child(Service::ID(GlobalVectorDriver::Name), 
                                                           Service::ID(Management::Name))
{
    mGlobalVectorCommand.SetSpeed(0);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
GlobalVectorDriver::~GlobalVectorDriver()
{
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Returns the current drive command received.
///
///   If the component is not in ready state, than the value of the default
///   velocity command is returned, otherwise the current command received
///   by controlling component is taken.
///
///   \return The current drive command last received.
///
////////////////////////////////////////////////////////////////////////////////////
SetGlobalVector GlobalVectorDriver::GetCurrentDriveCommand() const
{
    ReadLock rLock( *( (SharedMutex *)&mGlobalVectorDriverMutex) );
    return mGlobalVectorCommand;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \return The time (UTC seconds) that a drive command was received.
///
////////////////////////////////////////////////////////////////////////////////////
Time GlobalVectorDriver::GetDriveCommandTime() const
{
    ReadLock rLock( *( (SharedMutex *)&mGlobalVectorDriverMutex) );
    return mGlobalVectorCommandTime;
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
bool GlobalVectorDriver::GenerateEvent(const Events::Subscription& info) const
{
    if(info.mpQueryMessage->GetMessageCode() == QUERY_GLOBAL_VECTOR)
    {
        const QueryGlobalVector* query = static_cast<const QueryGlobalVector*>(info.mpQueryMessage);
        ReportGlobalVector report;
        {
            ReadLock rLock( *( (SharedMutex *)&mGlobalVectorDriverMutex) );
            CreateReportFromQuery(query, report);
        }
        SendEvent(info, &report);

        return true;
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
bool GlobalVectorDriver::IsEventSupported(const Events::Type type,
                                           const double requestedPeriodicRate,
                                           const Message* queryMessage,
                                           double& confirmedPeriodicRate,
                                           std::string& errorMessage) const
{
    if(queryMessage->GetMessageCode() == QUERY_GLOBAL_VECTOR)
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
///   This Service supports GlobalVectorDriver related messages only.
///
///   \param[in] message Message data to process.
///
////////////////////////////////////////////////////////////////////////////////////
void GlobalVectorDriver::Receive(const Message *message)
{
    switch(message->GetMessageCode())
    {
    case QUERY_GLOBAL_VECTOR:
        {
            const JAUS::QueryGlobalVector* query = static_cast<const JAUS::QueryGlobalVector*>(message);
            ReadLock rLock( *( (SharedMutex *)&mGlobalVectorDriverMutex) );
            ReportGlobalVector report;
            CreateReportFromQuery(query, report);
            Send(&report);
        }
        break;
    case SET_GLOBAL_VECTOR:
        {
            const JAUS::SetGlobalVector* command = static_cast<const JAUS::SetGlobalVector*>(message);
            {
                WriteLock wLock(mGlobalVectorDriverMutex);
                mGlobalVectorCommand = *command;
                mGlobalVectorCommandTime.SetCurrentTime();
            }
            this->SetDriveCommand(command);
            SignalEvent(REPORT_VELOCITY_COMMAND);
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
Message* GlobalVectorDriver::CreateMessage(const UShort messageCode) const
{
    Message* message = NULL;
    switch(messageCode)
    {
    case QUERY_GLOBAL_VECTOR:
        message = new JAUS::QueryGlobalVector();
        break;
    case SET_GLOBAL_VECTOR:
        message = new JAUS::SetGlobalVector();
        break;
    case REPORT_GLOBAL_VECTOR:
        message = new JAUS::ReportGlobalVector();
        break;
    default:
        message = NULL;
        break;
    };
    return message;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Prints the status of the Primitive Driver.
///
////////////////////////////////////////////////////////////////////////////////////
void GlobalVectorDriver::PrintStatus() const
{
    std::cout << "[" << GetServiceID().ToString() << "] - Current Velocity Command:\n";
    JAUS::SetGlobalVector command;
    
    if(GetStatus() != Management::Status::Ready)
    {
        std::cout << "Standby, no command.\n";
    }
    else
    {
        {
            ReadLock rLock( *( (SharedMutex *)&mGlobalVectorDriverMutex) );
            command = mGlobalVectorCommand;
        }
        command.PrintMessageBody();
    }
    
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Creates a report message based on the query.
///
////////////////////////////////////////////////////////////////////////////////////
void GlobalVectorDriver::CreateReportFromQuery(const QueryGlobalVector* query,
                                                ReportGlobalVector& report) const
{
    report.ClearMessage();
    report.SetDestinationID(query->GetSourceID());
    report.SetSourceID(GetComponentID());

    UInt pv1 = query->GetPresenceVector();
    UInt pv2 = mGlobalVectorCommand.GetPresenceVector();
    // Check bit field requested from pv1, then see if we have data for
    // that field in pv2, if so, set the data to report message.
    if( (pv2 & (pv1 & QueryGlobalVector::PresenceVector::Speed)) > 0) { report.SetSpeed(mGlobalVectorCommand.GetSpeed()); }
    if( (pv2 & (pv1 & QueryGlobalVector::PresenceVector::Altitude)) > 0) { report.SetAltitude(mGlobalVectorCommand.GetAltitude()); }
    if( (pv2 & (pv1 & QueryGlobalVector::PresenceVector::Heading)) > 0) { report.SetHeading(mGlobalVectorCommand.GetHeading()); }
    if( (pv2 & (pv1 & QueryGlobalVector::PresenceVector::Roll)) > 0) { report.SetRoll(mGlobalVectorCommand.GetRoll()); }
    if( (pv2 & (pv1 & QueryGlobalVector::PresenceVector::Pitch)) > 0) { report.SetPitch(mGlobalVectorCommand.GetPitch()); }
}


/*  End of File */
