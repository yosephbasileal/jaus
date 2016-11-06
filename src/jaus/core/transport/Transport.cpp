////////////////////////////////////////////////////////////////////////////////////
///
///  \file Transport.cpp
///  \brief This file contains the implementation for creating Transport Services
///  in JAUS++.
///
///  <br>Author(s): Daniel Barber
///  <br>Created: 11 October 2009
///  <br>Copyright (c) 2009
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
#include "jaus/core/transport/Transport.h"
#include "jaus/core/events/Event.h"
#include "jaus/core/discovery/QueryIdentification.h"
#include "jaus/core/time/ReportTime.h"

#include <queue>
#include <map>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time.hpp>

#include <cxutils/FileIO.h>
#include <cxutils/networking/UDPClient.h>
#include <cxutils/CircularArray.h>

namespace JAUS
{
    
}

//#define USE_MESSAGE_QUEUE

using namespace JAUS;

static const unsigned int PACKET_QUEUE_SIZE = 1024;

const std::string Transport::Name = "urn:jaus:jss:core:Transport";


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor, initializes default values.
///
////////////////////////////////////////////////////////////////////////////////////
Transport::Receipt::Receipt()
{
    mPendingFlag = true;
    mpMessage = NULL;
    mpResponse = NULL;
    mpResponses = NULL;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
Transport::Receipt::~Receipt()
{
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor, initializes default values.
///
////////////////////////////////////////////////////////////////////////////////////
Transport::Transport() 
    : Service(Service::ID(Transport::Name, 1.0), Service::ID()),
      mMessageQueue(PACKET_QUEUE_SIZE),
      mStopMessageProcessingFlag(true),
      mSequenceNumber(0),
      mStartTransportServicesFlag(false),
      mDisconnectTimeMs(10000)
                         
{
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
Transport::~Transport()
{
    Shutdown();

    // Delete memory
    std::map<UShort, Message*>::iterator msg;
    {
        WriteLock wLock(mMessageTemplatesMutex);
        mMessageTemplates.clear();
    }
    {
        WriteLock wLock(mMessageTemplatesMutex);
        mMessageCache.clear();
    }
}


/** Transport service is not discoverable. */
bool Transport::IsDiscoverable() const { return false; }


/** Performs basic transport services initialization. */
bool Transport::Initialize(const Address& componentID)
{
    try
    {
        Shutdown();
        mComponentID = componentID;

        mStopMessageProcessingFlag = false;
        mMessageProcessingThread = 
            boost::thread(boost::bind(&Transport::MessageProcessingThread, this));
        SetThreadName(mMessageProcessingThread,
            componentID.ToString() + ": Message Processing");
        return true;
    }
    catch(...)
    {
        return false;
    }
}


/** Shuts down the transport service. */
void Transport::Shutdown()
{
    mStopMessageProcessingFlag = true;
    mMessageAvailableCondition.notify_all();
    mMessageProcessingThread.join();
    mMessageProcessingThread.detach();
    mStartTransportServicesFlag = false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Tries to create/add a new fixed/permanent connection to a component
///          ID.
///
///   If component/transport is initialized, and method returns false, then
///   the connection attempt failed.  If not initialized, method will always
///   return false, however when initialization takes place, connections will
///   be created then.
///
///   \param[in] id ID of JAUS component.
///   \param[in] destinationIP Connection type information.
///   \param[in] port The destination port.
///
///   \return True on success, false on failure.  See detailed function notes.
///
////////////////////////////////////////////////////////////////////////////////////
bool Transport::AddNetworkConnection(const Address& id,
                                     const std::string& destinationIP,
                                     const unsigned short port)
{
    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Method called when data is received by the service.  This method
///   shares it will all Services on on the component.
///
///   \param[in] message Message to process.
///
////////////////////////////////////////////////////////////////////////////////////
void Transport::Receive(const Message* message)
{
    PushMessageToChildren(message);
}


/** Updates all processes within the Transport service. */
void Transport::UpdateServiceEvent()
{

}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Given a message code, the Service attempts to create a Message object
///          that can be used for de-serialization of JAUS packets.
///
///   \param[in] messageCode JAUS Message to try create.
///
///   \return Pointer to created message, NULL if message not supported by
///           any Services attached to the Transport.
///
////////////////////////////////////////////////////////////////////////////////////
Message* Transport::CreateMessage(const UShort messageCode) const
{
    // Try to create the message recursively.
    Message* message = CreateMessageFromService(messageCode, this);
    return message;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sends a JAUS message.
///
///   \param[in] message JAUS Message to send.
///   \param[in] broadcastFlags Values to use to signify if message should be
///                             sent using any broadcast options (e.g.
///                             multicast). 0 = no options, 1 = local broadcast,
///                             2 = global broadcast.
///
///   \return True on success, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool Transport::Send(const Message* message,
                     const int broadcastFlags) const
{
    Packet sendPacket;
    Packet::List stream;
    Header::List streamHeaders;
    Packet::List::iterator packet;
    Header::List::iterator header;
    Header jausHeader;

    if(message->GetDestinationID() == mComponentID)
    {
        return false;
    }
    // Automatically set the source ID.
    ( (Message * ) message )->SetSourceID(mComponentID);
    unsigned int bestPacketSize = 
        this->GetMaximumPacketSizeBytes();

    SharedMutex* seqMutex = (SharedMutex*)&mSequenceNumberMutex;
    UShort sequenceNumber = 0;
    
    if(message->IsLargeDataSet(bestPacketSize))
    {
        if(SerializeMessage(message, stream, streamHeaders, 0, (Byte)broadcastFlags) == false)
        {
            return false;
        }
        // Get sequence number
        {
            WriteLock wLock(*seqMutex);
            sequenceNumber = mSequenceNumber;
            (*((UShort *)(&mSequenceNumber))) += (UShort)stream.size();
        }
        for(packet = stream.begin(), header = streamHeaders.begin();
            packet != stream.end() && header != streamHeaders.end();
            packet++, header++)
        {
            header->mSequenceNumber = sequenceNumber++;
            // Sequence number goes at the end of the packet (this is retarded).
            packet->Write(sequenceNumber, packet->Length() - USHORT_SIZE);
            if(SendPacket(*packet, *header) == false)
            {
                return false;
            }
        }
        return true;
    }
    // Small message (single packet).
    {
        // Get sequence number
        WriteLock wLock(*seqMutex);
        sequenceNumber = mSequenceNumber;
        (*((UShort *)(&mSequenceNumber)))++;
    }
    
    if(message->Write(*((Packet *)&sendPacket), jausHeader, &(GetTransportHeader()), true, sequenceNumber, (Byte)broadcastFlags))
    {
        bool result =  SendPacket(sendPacket, jausHeader);

        return result;
    }

    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sends a JAUS message to multiple destinations, reducing the 
///   number of times the data must be re-serialized.
///
///   This method is good for when you needs to send a large chunk of the same
///   data to multiple places (like range data or video).
///
///   \param[in] destinations All destinations the message must go to.
///   \param[in] message JAUS Message to send.
///   \param[in] broadcastFlags Values to use to signify if message should be
///                             sent using any broadcast options (e.g.
///                             multicast). 0 = no options, 1 = local broadcast,
///                             2 = global broadcast.
///
///   \return True on success, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool Transport::SendToList(const Address::Set& destinations,
                           const Message* message,
                           const int broadcastFlags) const
{
    Packet sendPacket;
    Packet::List stream;
    Header::List streamHeaders;
    Packet::List::iterator packet;
    Header::List::iterator header;
    Header jausHeader;

    if(message->GetDestinationID() == mComponentID)
    {
        return false;
    }
    // Automatically set the source ID.
    ( (Message * ) message )->SetSourceID(mComponentID);
    unsigned int bestPacketSize = 
        GetMaximumPacketSizeBytes();

    Address::Set::const_iterator dest;
    SharedMutex* seqMutex = (SharedMutex*)&mSequenceNumberMutex;
    unsigned int transportHeaderSize = GetTransportHeader().Length();

    if(message->IsLargeDataSet(bestPacketSize))
    {
        if(SerializeMessage(message, stream, streamHeaders, 0, (Byte)broadcastFlags) == false)
        {
            return false;
        }
        for(dest = destinations.begin();
            dest != destinations.end();
            dest++)
        {
            // Get sequence numbers for this message sequence.
            UShort sequenceNumber = 0;
            {
                WriteLock wsLock(*seqMutex);
                sequenceNumber = mSequenceNumber;
                (*((UShort *)(&mSequenceNumber))) += (UShort)stream.size();;
            }
            for(packet = stream.begin(), header = streamHeaders.begin();
                packet != stream.end() && header != streamHeaders.end();
                packet++, header++)
            {
                header->mDestinationID = *dest;
                header->mSequenceNumber = sequenceNumber++;
                packet->SetWritePos(transportHeaderSize);
                // Update header
                header->Write(*packet);
                // Send the data.
                if(SendPacket(*packet, *header) == false)
                {
                    return false;
                }
            }
        }
        return true;
    }
    else
    {
        bool result = false;
        
        message->Write(sendPacket, jausHeader, &(GetTransportHeader()), true, 0, (Byte)broadcastFlags);
        for(dest = destinations.begin();
            dest != destinations.end() && sendPacket.Length() > 0;
            dest++)
        {
            // Update JAUS Header data in packet.
            jausHeader.mDestinationID = *dest;
            // Now get a new sequence number
            {
                WriteLock wsLock(*seqMutex);
                jausHeader.mSequenceNumber = mSequenceNumber;
                (*((UShort *)(&mSequenceNumber)))++;
            }
            // Replace sequence number in packet.
            sendPacket.SetWritePos(transportHeaderSize);
            jausHeader.Write(sendPacket);
            // Send the packet
            result = SendPacket(sendPacket, jausHeader);
        }
        return result;
    }

    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sends a JAUS message, and blocks until a response is received.
///
///   \param[in] message JAUS Message to send.
///   \param[out] response Pointer to message that is the response to the
///                        message sent which will read received data.
///   \param[in] waitTimeMs How long to wait in ms for a response to be
///                         received. A value of 0 is INFINITE.
///
///   \return True on success, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool Transport::Send(const Message* message,
                     Message* response,
                     const unsigned int waitTimeMs) const
{
    Message::List possibleResponses;
    possibleResponses.push_back(response);
    return Send(message, possibleResponses, waitTimeMs);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sends a JAUS message, and blocks until a response is received.  You
///          must pass pointers to the response message object by adding them
///          to the list passed.
///
///   On a succesful return, the possibleResponses list is modified to
///   contain only the message containing the response.  Make sure you manage
///   the memory of the messages put in this list to prevent memory leaks.
///
///   \param[in] message JAUS Message to send.
///   \param[out] possibleResponses Lists of messages that are possible
///                                 responses to the message sent. For example
///                                 Create Event can be responded with
///                                 Confirm or Reject Event Request.  Method
///                                 does not delete or modify pointers.
///   \param[in] waitTimeMs How long to wait in ms for a response to be
///                         received, 0 is INFINITE.
///
///   \return True on success, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool Transport::Send(const Message* message,
                     Message::List& possibleResponses,
                     const unsigned int waitTimeMs) const
{
    Receipt receipt;
    Receipt::Set::iterator ritr;

    if(message == NULL || possibleResponses.size() == 0 || message->GetDestinationID().IsBroadcast())
    {
        return false;
    }

    receipt.mPendingFlag = true;
    receipt.mpMessage = message;
    if(possibleResponses.size() == 1)
    {
        receipt.mpResponse = possibleResponses.front();
    }
    else
    {
        receipt.mpResponses = &possibleResponses;
    }

    // Insert into the receipts queue so that
    // as messages arrive, they can be checked against
    // pending receipts.
    Receipt::Set* pending = NULL;
    {
        WriteLock wLock(*((SharedMutex *)&mPendingReceiptsMutex));
        pending = ((Receipt::Set *)&mPendingReceipts);
        pending->insert(&receipt);
    }

    // Send the messages
    if(Send(message))
    {
        boost::mutex::scoped_lock lock(receipt.mConditionMutex);
        if(receipt.mPendingFlag == true)
        {
            // Wait for notification of response received.
            if(waitTimeMs == 0)
            {
                receipt.mWaitCondition.wait(lock);
            }
            else
            {
                receipt.mWaitCondition.timed_wait(lock,
                    boost::posix_time::milliseconds(waitTimeMs));
            }
        }
    }
    
    // Remove the receipt from the queue of
    // pending receipts so it is
    // no longer checked
    {
        WriteLock wLock(*((SharedMutex *)&mPendingReceiptsMutex));
        pending = ((Receipt::Set *)&mPendingReceipts);
        Receipt::Set::iterator toRemove = 
            pending->find(&receipt);
        if(toRemove != pending->end())
        {
            pending->erase(toRemove);
        }
    }
    return !receipt.mPendingFlag;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Allows external programs to get notified when specific types of
///          messages are received by the Transport Service.
///
///   Messages are always passed to the inheriting Services, only copies of those
///   messages are sent to registered callbacks (after Service receives them).
///
///   This method only works if called before component/transport initialization.
///
///   \param[in] messageCode The type of message to register the callback for.
///   \param[in] callback The callback to use.
///
///   \return True on success, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
void Transport::RegisterCallback(const UShort messageCode, Transport::Callback* callback)
{
    // Can only register before initialization
    if(IsInitialized() == false)
    {
        mMessageCallbacks[messageCode].insert(callback);
    }
    else
    {
        std::cout << "Transport::RegisterCallback::ERROR - Must register before initialization!\n";
        assert(0 && "Transport::RegisterCallback::ERROR - Must register before initialization!\n");
    }
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Given a message code, the Service attempts to create a Message object
///          that can be used for de-serialization of JAUS packets.
///
///   \param[in] messageCode JAUS Message to try create.
///   \param[in] service The Service to use to create a message.
///
///   \return Pointer to created message, NULL if message not supported by
///           or attached to the given Service.
///
////////////////////////////////////////////////////////////////////////////////////
Message* Transport::CreateMessageFromService(const UShort messageCode, const Service *service) const
{
    Message* message = NULL;

    if(mStopMessageProcessingFlag)
    {
        return message;
    }

    const Transport* transport = dynamic_cast<const Transport*>(service);
    if(transport)
    {
        message =  GetMessageFromTemplate(messageCode);
    }
    // Make sure we do not do an infinite recursive call on ourselves!
    if(message == NULL && this != service)
    {
        message = service->CreateMessage(messageCode);
    }
    if(message == NULL)
    {
        Service::Map::const_iterator child;
        for(child = service->mJausChildServices.begin();
            child != service->mJausChildServices.end();
            child++)
        {
            if( (message = CreateMessageFromService(messageCode, child->second)) != NULL )
            {
                break;
            }
        }
    }

    return message;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts a packet containing a message type (UShort) and payload to
///          a Message structure if supported by the child services.
///
///   \param[in] packet Packet of payload data with the frist 2 bytes being
///                     the message code.  This packet should not contain
///                     the general transport header.
///
///   \return Pointer to created message, NULL if message not supported by
///           or attached to the given Service.
///
////////////////////////////////////////////////////////////////////////////////////
Message* Transport::CreateMessageFromPacket(const Packet& packet) const
{
    // Read the header data from the packet.
    packet.SetReadPos(0);
    UShort messageCode = 0;
    packet.Read(messageCode);
    Message* message = CreateMessageFromService(messageCode, this);
    if(message)
    {
        Packet::Wrapper wrapper((unsigned char *)(packet.Ptr() + USHORT_SIZE), packet.Length() - USHORT_SIZE);
        if(message->ReadMessageBody(*wrapper.GetData()) < 0)
        {
            delete message;
            message = NULL;
            if(mDebugMessagesFlag)
            {
                WriteLock printLock(mDebugMessagesMutex);
                std::cout << "[" << GetServiceID().ToString() << "-" << mComponentID.ToString() << "] - Received Malformed Packet [0x" << std::setbase(16) << messageCode << std::setbase(10) << "]\n";
            }
        }
    }
    // Reset packet data.
    packet.SetReadPos(0);
    return message;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Triggers any callbacks registered for the given message type.
///
////////////////////////////////////////////////////////////////////////////////////
void Transport::TriggerMessageCallbacks(const Message* message)
{
    Callback::Map::iterator cb;
    cb = mMessageCallbacks.find(message->GetMessageCode());
    if(cb != mMessageCallbacks.end())
    {
        Callback::Set::iterator cb2;
        for(cb2 = cb->second.begin();
            cb2 != cb->second.end();
            cb2++)
        {
            (*cb2)->ProcessMessage(message);
        }
    }
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Add a message class that the Transport service will save and use
///          to deserialize custom messages no created by any known Service.
///
///   \param[in] message Pointer to message template.  Pointer ownership will
///                      be taken and any memory deleted by service.
///
////////////////////////////////////////////////////////////////////////////////////
void Transport::AddMessageTemplate(Message* message)
{
    WriteLock wLock(mMessageTemplatesMutex);
    mMessageTemplates[message->GetMessageCode()]
        = boost::shared_ptr<Message>(message);
}


unsigned int Transport::GetDisconnectTimeMs() const
{
    return mDisconnectTimeMs;
}


void Transport::SetDisconnectTimeMs(const unsigned int limitMs)
{
    mDisconnectTimeMs = limitMs;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Creates the desired message from templates.
///
///   \param[in] messageCode Message type.
///
///   \return Pointer to message structure (you must delete) on success, NULL
///           if message type not found.
///
////////////////////////////////////////////////////////////////////////////////////
Message*  Transport::GetMessageFromTemplate(const UShort messageCode) const
{
    Message* message = NULL;
    {
        ReadLock rLock(*((SharedMutex*)&mMessageTemplatesMutex));
        std::map<UShort, boost::shared_ptr<Message> >::const_iterator m;
        m = mMessageTemplates.find(messageCode);
        if(m != mMessageTemplates.end())
        {
            message = m->second->Clone();
        }
    }
    return message;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Gets a pointer to a Message structure that has been previoulsy
///          created to reduce constant re-allocation of memory for
///          de-serializing message data.  If the message has not been
///          created previously, it get's made.
///
///   \param[in] messageCode Message type.
///
///   \return Pointer to message structure (you must delete) on success, NULL
///           if message type not found.
///
////////////////////////////////////////////////////////////////////////////////////
Message*  Transport::GetCachedMessage(const UShort messageCode)
{
    Message* message = NULL;
    {
        ReadLock rLock(mMessageCacheMutex);
        std::map<UShort, boost::shared_ptr<Message> >::iterator m;
        m = mMessageCache.find(messageCode);
        if(m == mMessageCache.end())
        {
            message = CreateMessage(messageCode);
            if(message)
            {
                mMessageCache[messageCode] 
                    = boost::shared_ptr<Message>(message);
            }
        }
        else
        {
            message = m->second.get();
        }
        if(message)
        {
            message->ClearMessage();
        }
    }
    return message;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Copies message callbacks and templates from transport service.
///
///   This method is needed to preserve registered events, etc. when the
///   Transport method is changed within a Component.
///
///   \param[in] transport Pointer to transport service to get data from.
///
////////////////////////////////////////////////////////////////////////////////////
void Transport::CopyRegisteredItems(Transport* transport)
{
    {
        std::map<UShort, boost::shared_ptr<Message> >::iterator msg;
        ReadLock lock( transport->mMessageTemplatesMutex);
        for(msg = transport->mMessageTemplates.begin();
            msg != transport->mMessageTemplates.end();
            msg++)
        {
            AddMessageTemplate(msg->second->Clone());
        }
    }
    {
        Callback::Map::iterator cb;
        for(cb = transport->mMessageCallbacks.begin();
            cb != transport->mMessageCallbacks.end();
            cb++)
        {
            Callback::Set::iterator cb2;
            for(cb2 = cb->second.begin();
                cb2 != cb->second.end();
                cb2++)
            {
                RegisterCallback(cb->first, *cb2);
            }
        }
    }
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Helper method to match packets to pending receipts that are
///          from blocking send methods.
///
////////////////////////////////////////////////////////////////////////////////////
bool Transport::CheckPendingReceipts(const Header& header,
                                     const UShort messageCode,
                                     const Packet& packet)
{
    Receipt::Set::iterator receipt;

    ReadLock rLock(mPendingReceiptsMutex);

    for(receipt = mPendingReceipts.begin();
        receipt != mPendingReceipts.end();
        receipt++)
    {
        boost::mutex::scoped_lock lock((*receipt)->mConditionMutex);
        if((*receipt)->mpResponse != NULL)
        {
            if((*receipt)->mpResponse ->GetMessageCode() == messageCode &&
                   (*receipt)->mpMessage->GetDestinationID() == header.mSourceID)
                {
                    if( (*receipt)->mpResponse ->Read(packet) )
                    {
                        // Notify the other thread that
                        // the response was received.
                        (*receipt)->mPendingFlag = false;
                        (*receipt)->mWaitCondition.notify_one();
                        return true;
                    }
                }
        }
        // Handle the case where a 
        // message may have different response types, and if
        // this is the case, see if this packet is a
        // possible response.
        else if((*receipt)->mpResponses != NULL)
        {
            Message::List::iterator responses = (*receipt)->mpResponses->begin();
            while(responses != (*receipt)->mpResponses->end())
            {
                if((*responses)->GetMessageCode() == messageCode &&
                   (*receipt)->mpMessage->GetDestinationID() == header.mSourceID)
                {
                    if( (*responses)->Read(packet) )
                    {
                        Message* final = (*responses);
                        // Remove possible choices so we know which one
                        // has valid data.
                        responses = (*receipt)->mpResponses->begin();
                        while(responses != (*receipt)->mpResponses->end())
                        {
                            if( (*responses) != final )
                            {
                                responses = (*receipt)->mpResponses->erase(responses); 
                                continue;
                            }
                            responses++;
                        }

                        // Notify the other thread that
                        // the response was received.
                        (*receipt)->mPendingFlag = false;
                        (*receipt)->mWaitCondition.notify_one();
                        return true;
                    }
                }
                responses++;
            }
        }
    }

    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Process all data as it arrives and assignes to packet handling
///          queues for processing by other methods.
///
///   \param[in] jausPacket JAUS Packet data without the transport header.
///   \param[in] jausHeader JAUS General Header information for the packet.
///   \param[in] connection Pointer to the connection the data was received by.
///   \param[in] sourceInfo Additional information about the source of the packet
///                         like the address it came in on.
///
////////////////////////////////////////////////////////////////////////////////////
void Transport::ProcessPacket(Packet& jausPacket,
                              Header& jausHeader)
{
    // Ignore message not destined for this component.
    if(Address::DestinationMatch(jausHeader.mDestinationID, mComponentID) == false)
    {
        return;
    }

    UShort messageCode = 0;

    // Read the message code.
    jausPacket.SetReadPos(Header::PayloadOffset);
    jausPacket.Read(messageCode);
    jausPacket.SetReadPos(0);   // Reset the read position.

    // Send Acknowledge if requested.
    if(jausHeader.mAckNackFlag == Header::AckNack::Request)
    {
        Header ackHeader = jausHeader;
        ackHeader.mSourceID = mComponentID;
        ackHeader.mDestinationID = jausHeader.mSourceID;
        ackHeader.mSize = Header::MinSize;
        ackHeader.mAckNackFlag = Header::AckNack::Ack;
        ackHeader.mSequenceNumber = jausHeader.mSequenceNumber;
        Packet ackPacket;
        if(ackHeader.Write(ackPacket))
        {
            SendPacket(ackPacket, ackHeader);
        }
    }

    {
        WriteLock wLock(mMessageQueueMutex);
        mMessageQueue.push_back(jausPacket);
    }
    
    // Notify threads of new message data
    //boost::lock_guard<boost::mutex> lock(mProcessingMutex);
    //mMessageAvailableCondition.notify_all();
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Goes the through the single packets queue and de-serializes the data
///          and passes it to any neededing services.
///
///   \param[in] packet Packet to process, contains jaus header and
///                     message body (no transport header data).
///   \param[in] header Header data for packet.
///   \param[in] messageCode Packet message code.
///
////////////////////////////////////////////////////////////////////////////////////
void Transport::ProcessSinglePacket(Packet& packet, 
                                    Header& header,
                                    UShort messageCode)
{
    bool dataToProcess = false;
    bool multithreaded = false;
    Packet mtPacket;

    // De-serialize the data

    Message* message = NULL;

    bool foundReceipt = false;
    packet.SetReadPos(0);

    // See if there is a receipt already waiting for this packet
    // as a response
    if(CheckPendingReceipts(header, messageCode, packet) == false)
    {
        // If no pending match, then we must de-serialize and
        // share the data.

        // Get a message object to read the data with.
        message = GetCachedMessage(messageCode);

        // If supported, de-serialize data and receive.
        if(message && message->Read(packet) > 0)
        {
            if(mDebugMessagesFlag)
            {
                WriteLock printLock(mDebugMessagesMutex);
                std::cout << "[" << GetServiceID().ToString() << "-" << mComponentID.ToString() << "] - Processing " << message->GetMessageName() << " Message\n";
            }
            PushMessageToChildren(message);
        }
        else if(message == NULL)
        {
            WriteLock printLock(mDebugMessagesMutex);
            std::cout << "[" << GetServiceID().ToString() << "-" << mComponentID.ToString() << "] - Received (Single Packet) Unsupported Type [0x" << std::setbase(16) << messageCode << std::setbase(10) << "]\n";
        }
    }
}


/** Goes the through the multi packets queue and de-serializes the data
    and passes it to any neededing services. */
void Transport::ProcessMultiPacket(Packet& packet,
                                   Header& header,
                                   UShort messageCode)
{
    bool dataToProcess = false;

    // De-serialize the data
    Message* message = NULL;

    // Multi-packet stream handling.
    UInt presenceVector = 0;
    LargeDataSet::Key key(header.mSourceID,
        messageCode,
        presenceVector);

    bool added = false;
    std::vector<LargeDataSet>::iterator ld;

    for(ld = mLargeDataSets.begin();
        ld != mLargeDataSets.end() && added == false;
        ld++)
    {
        if(ld->mCompleteFlag == false &&
            ld->mMessageCode == messageCode &&
            ld->mHeader.mSourceID == header.mSourceID)
        {
            // If the first in the sequence, flush out the old data
            // and start over.
            if(header.mControlFlag == Header::DataControl::First)
            {
                ld->Clear();
                ld->AddPacket(header, messageCode, packet);
            }
            else if(ld->AddPacket(header, messageCode, packet))
            {
                added = true;
            }
            break;
        }
    }

    // Check for pending receipts/and or completed data.
    ld = mLargeDataSets.begin();
    while(ld != mLargeDataSets.end())
    {
        if(ld->mCompleteFlag == true)
        {
            LargeDataSet& stream = *ld;
            Message* message = NULL;

            // Remove from map
            mLargeDataSets.erase(ld);
            ld = mLargeDataSets.begin();

            mLargeDataSetCache.Clear();
            // Try merge the stream into a single packet.
            if(LargeDataSet::MergeLargeDataSet(stream.mHeader,
                stream.mMessageCode,
                mLargeDataSetCache,
                stream.mStream,
                NULL))
            {
                // If we merged the packet, then use the same methods as if
                // this was a single packet.

                // See if there is a receipt already waiting for this packet
                // as a response
                if(CheckPendingReceipts(stream.mHeader, 
                    stream.mMessageCode, 
                    mLargeDataSetCache) == false)
                {
                    // If no pending match, then we must de-serialize and
                    // share the data.

                    // Get message to read data with.
                    message = GetCachedMessage(messageCode);
                }
                else
                {
                    message = CreateMessage(messageCode);
                }
                // If supported, de-serialize data and receive.
                if(message && message->Read(mLargeDataSetCache) > 0)
                {
                    if(mDebugMessagesFlag)
                    {
                        WriteLock printLock(mDebugMessagesMutex);
                        std::cout << "[" << GetServiceID().ToString() << "-" << mComponentID.ToString() << "] - Processing " << message->GetMessageName() << " Message\n";
                    }
                    PushMessageToChildren(message);
                }
                else if(message == NULL)
                {
                    WriteLock printLock(mDebugMessagesMutex);
                    std::cout << "[" << GetServiceID().ToString() << "-" << mComponentID.ToString() << "] - Received (Multi-Packet) Unsupported Type [0x" << std::setbase(16) << stream.mMessageCode << std::setbase(10) << "]\n";
                }
            }

            stream.Clear();
        }
        // If not updated or modified for more than 5 seconds, remove
        // from our queue
        else if(Time::GetUtcTimeMs() - ld->mUpdateTimeMs > 5000)
        {
            ld = mLargeDataSets.erase(ld);
        }
        else
        {
            ld++;
        }
    }
}


/** Thread for processing all data separately from the 
    transports receive mechanism to avoid deadlocks when
    a service is waiting for responses upon receipt of a new message. */
void Transport::MessageProcessingThread()
{
    //boost::unique_lock<boost::mutex> processingLock(mProcessingMutex);
    Packet jausPacket;
    
    int loopCounter = 0;

    while(mStopMessageProcessingFlag == false)
    {
        if( (++loopCounter%2) == 0)
        {
            boost::this_thread::sleep(boost::posix_time::millisec(1));
        }
        // Wait for new message.
        //mMessageAvailableCondition.timed_wait(processingLock,
        //    boost::posix_time::millisec(100));

        // Check for exit condition
        if(mStopMessageProcessingFlag)
        {
            break;
        }

        bool queueFull = true;
        while(mStopMessageProcessingFlag == false)
        {
            {
                jausPacket.Clear();
                ReadLock rLock(mMessageQueueMutex);
                if(mMessageQueue.size() > 0)
                {
                    jausPacket = mMessageQueue[0];
                    mMessageQueue.pop_front();
                }
                else
                {
                    break;
                }
            }

            if(jausPacket.Length() == 0)
            {
                break;
            }

            // Read the header
            Header jausHeader;
            UShort messageCode = 0;
            jausPacket.SetReadPos(0);
            jausHeader.Read(jausPacket);
            jausPacket.Read(messageCode);
            jausPacket.SetReadPos();
            // Process appropriately.
            if(jausHeader.mControlFlag == Header::DataControl::Single)
            {
                ProcessSinglePacket(jausPacket,
                                    jausHeader,
                                    messageCode);
            }
            else
            {
                ProcessMultiPacket(jausPacket,
                                   jausHeader,
                                   messageCode);
            }
        }
    }
}

/*  End of File */
