////////////////////////////////////////////////////////////////////////////////////
///
///  \file LocalVectorDriver.cpp
///  \brief This file contains the implementation of the LocalVectorDriver class.
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
#include "jaus/mobility/drivers/LocalVectorDriver.h"
#include "jaus/core/Component.h"

using namespace JAUS;

const std::string LocalVectorDriver::Name = "urn:jaus:jss:mobility:LocalVectorDriver";

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////////
LocalVectorDriver::LocalVectorDriver() : Management::Child(Service::ID(LocalVectorDriver::Name), 
                                                           Service::ID(Management::Name))
{
    mLocalVectorCommand.SetSpeed(0);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
LocalVectorDriver::~LocalVectorDriver()
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
SetLocalVector LocalVectorDriver::GetCurrentDriveCommand() const
{
    ReadLock rLock( *( (SharedMutex *)&mLocalVectorDriverMutex) );
    return mLocalVectorCommand;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \return The time (UTC seconds) that a drive command was received.
///
////////////////////////////////////////////////////////////////////////////////////
Time LocalVectorDriver::GetDriveCommandTime() const
{
    ReadLock rLock( *( (SharedMutex *)&mLocalVectorDriverMutex) );
    return mLocalVectorCommandTime;
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
bool LocalVectorDriver::GenerateEvent(const Events::Subscription& info) const
{
    if(info.mpQueryMessage->GetMessageCode() == QUERY_LOCAL_VECTOR)
    {
        const QueryLocalVector* query = static_cast<const QueryLocalVector*>(info.mpQueryMessage);
        ReportLocalVector report;
        {
            ReadLock rLock( *( (SharedMutex *)&mLocalVectorDriverMutex) );
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
bool LocalVectorDriver::IsEventSupported(const Events::Type type,
                                           const double requestedPeriodicRate,
                                           const Message* queryMessage,
                                           double& confirmedPeriodicRate,
                                           std::string& errorMessage) const
{
    if(queryMessage->GetMessageCode() == QUERY_LOCAL_VECTOR)
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
///   This Service supports LocalVectorDriver related messages only.
///
///   \param[in] message Message data to process.
///
////////////////////////////////////////////////////////////////////////////////////
void LocalVectorDriver::Receive(const Message *message)
{
    switch(message->GetMessageCode())
    {
    case QUERY_LOCAL_VECTOR:
        {
            const JAUS::QueryLocalVector* query = static_cast<const JAUS::QueryLocalVector*>(message);
            ReadLock rLock( *( (SharedMutex *)&mLocalVectorDriverMutex) );
            ReportLocalVector report;
            CreateReportFromQuery(query, report);
            Send(&report);
        }
        break;
    case SET_LOCAL_VECTOR:
        {
            const JAUS::SetLocalVector* command = static_cast<const JAUS::SetLocalVector*>(message);
            {
                WriteLock wLock(mLocalVectorDriverMutex);
                mLocalVectorCommand = *command;
                mLocalVectorCommandTime.SetCurrentTime();
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
Message* LocalVectorDriver::CreateMessage(const UShort messageCode) const
{
    Message* message = NULL;
    switch(messageCode)
    {
    case QUERY_LOCAL_VECTOR:
        message = new JAUS::QueryLocalVector();
        break;
    case SET_LOCAL_VECTOR:
        message = new JAUS::SetLocalVector();
        break;
    case REPORT_LOCAL_VECTOR:
        message = new JAUS::ReportLocalVector();
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
void LocalVectorDriver::PrintStatus() const
{
    std::cout << "[" << GetServiceID().ToString() << "] - Current Velocity Command:\n";
    JAUS::SetLocalVector command;
    
    if(GetStatus() != Management::Status::Ready)
    {
        std::cout << "Standby, no command.\n";
    }
    else
    {
        {
            ReadLock rLock( *( (SharedMutex *)&mLocalVectorDriverMutex) );
            command = mLocalVectorCommand;
        }
        command.PrintMessageBody();
    }
    
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Creates a report message based on the query.
///
////////////////////////////////////////////////////////////////////////////////////
void LocalVectorDriver::CreateReportFromQuery(const QueryLocalVector* query,
                                                ReportLocalVector& report) const
{
    report.ClearMessage();
    report.SetDestinationID(query->GetSourceID());
    report.SetSourceID(GetComponentID());

    UInt pv1 = query->GetPresenceVector();
    UInt pv2 = mLocalVectorCommand.GetPresenceVector();
    // Check bit field requested from pv1, then see if we have data for
    // that field in pv2, if so, set the data to report message.
    if( (pv2 & (pv1 & QueryLocalVector::PresenceVector::Speed)) > 0) { report.SetSpeed(mLocalVectorCommand.GetSpeed()); }
    if( (pv2 & (pv1 & QueryLocalVector::PresenceVector::Z)) > 0) { report.SetZ(mLocalVectorCommand.GetZ()); }
    if( (pv2 & (pv1 & QueryLocalVector::PresenceVector::Heading)) > 0) { report.SetHeading(mLocalVectorCommand.GetHeading()); }
    if( (pv2 & (pv1 & QueryLocalVector::PresenceVector::Roll)) > 0) { report.SetRoll(mLocalVectorCommand.GetRoll()); }
    if( (pv2 & (pv1 & QueryLocalVector::PresenceVector::Pitch)) > 0) { report.SetPitch(mLocalVectorCommand.GetPitch()); }
}


/*  End of File */
