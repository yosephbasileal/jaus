////////////////////////////////////////////////////////////////////////////////////
///
///  \file Transport.h
///  \brief This file contains the definition for creating Transport Services
///  in JAUS++.
///
///  <br>Author(s): Daniel Barber
///  <br>Created: 30 September 2009
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
#ifndef __JAUS_CORE_TRANSPORT_BASE__H
#define __JAUS_CORE_TRANSPORT_BASE__H

#include "jaus/core/Service.h"
#include "jaus/core/Time.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

#include <set>

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class Transport
    ///   \brief Transport is an interface class for creating Transport Services
    ///          defined by the SAE-JAUS standard.  All Transport Service 
    ///          implementations are based from this class.
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_CORE_DLL Transport : public Service
    {
        friend class Component;
    public:        
        ////////////////////////////////////////////////////////////////////////////////////
        ///
        ///   \class Callback
        ///   \brief Callback class used to register to get a copy of a message
        ///          received by the Transport Service.  Messages received through 
        ///          the Callback are still received by the Component Services.
        ///
        ////////////////////////////////////////////////////////////////////////////////////
        class JAUS_CORE_DLL Callback
        {
        public:
            Callback() {}
            virtual ~Callback() {}
            typedef std::set<Callback* > Set;
            typedef std::map<UShort, Set > Map;
            virtual void ProcessMessage(const JAUS::Message* message) {};
        };
        static const std::string Name;                  ///<  Name of the service.
        Transport();
        virtual ~Transport();
        // Transport Service does not response to any messages.
        virtual bool IsDiscoverable() const;
        // Initializes the transport with a given ID for a component.
        virtual bool Initialize(const Address& componentID);
        // Returns true if transport has been initialized.
        virtual bool IsInitialized() const = 0;
        // Shutdown the transport service.
        virtual void Shutdown();
        // Loads settings
        virtual bool LoadSettings(const std::string& filename) = 0;
        // Send a serialized message.
        virtual bool SendPacket(const Packet& packet, 
                                const Header& header)  const = 0;
        // Serialize the JAUS message (add transport headers, serialize payload, etc.) 
        virtual bool SerializeMessage(const Message* message, 
                                      Packet::List& stream,
                                      Header::List& streamHeaders,
                                      const UShort startingSequenceNumber,
                                      const int broadcastFlags) const = 0;
        // Adds a new connection and tries to connect.
        virtual bool AddNetworkConnection(const Address& id,
                                          const std::string& destinationIP,
                                          const unsigned short port);
        // Sends to all children.
        virtual void Receive(const Message* message);
        // Get the packet size this transport allows.
        virtual unsigned int GetMaximumPacketSizeBytes() const = 0;
        // Method called to update communications and events within the service.
        virtual void UpdateServiceEvent();
        // Searches inheriting Services (child Services) and factories to create a message for processing.
        virtual Message* CreateMessage(const UShort messageCode) const;
        // Sends a message.
        virtual bool Send(const Message* message, 
            const int broadcastFlags = 
                Service::NoBroadcast) const;
        // Sends a message to multiple places.
        virtual bool SendToList(const Address::Set& destinations,
                                const Message* message, 
                                const int broadcastFlags = 
                                        Service::NoBroadcast) const;
        // Sends a message, then waits for a response.
        virtual bool Send(const Message* message, 
                          Message* response, 
                          const unsigned int waitTimeMs = Service::DefaultWaitMs) const;
        // Sends a message and waits for one of multiple responses.
        virtual bool Send(const Message* message,
                          Message::List& possibleResponses,
                          const unsigned int waitTimeMs = Service::DefaultWaitMs) const;
        // Register to receive copies of messages when received by Transport.
        void RegisterCallback(const UShort messageCode, Callback* callback);
        // Reads the payload data which includes a message code, and converts to Message structure.
        Message* CreateMessageFromPacket(const Packet& packet) const;
        // Triggers any callbacks for a message.
        void TriggerMessageCallbacks(const Message* message);
        // Adds new message template.
        void AddMessageTemplate(Message* message);
        /** Gets how long to consider a connection active in ms. */
        unsigned int GetDisconnectTimeMs() const;
        /** Sets how long to consider a connection active in ms. */
        void SetDisconnectTimeMs(const unsigned int limitMs);
        /** Turns on the transport service. */
        void StartTransportServices() { mStartTransportServicesFlag = true; }
    protected:
        virtual const Packet& GetTransportHeader() const = 0;
        // Copies message template and callbacks.
        void CopyRegisteredItems(Transport* transport);
        // Called when data packets arrive from a connection.
        virtual void ProcessPacket(Packet& jausPacket,
                                   Header& jausHeader);
        volatile bool mStartTransportServicesFlag;            ///<  Wait until this is true to begin service.
    private:
        ////////////////////////////////////////////////////////////////////////////////////
        ///
        ///   \class Receipt
        ///   \brief Data structure used to store information about messages waiting
        ///          for incoming responses.
        ///
        ////////////////////////////////////////////////////////////////////////////////////
        class JAUS_CORE_DLL Receipt
        {
        public:
            typedef std::set<Receipt*> Set; ///< List of receipt data.
            Receipt();
            ~Receipt();
            volatile bool mPendingFlag;     ///<  True if pending.
            const Message* mpMessage;       ///<  Message sent.
            Message* mpResponse;            ///<  Message response data (if only 1 possible response).
            Message::List* mpResponses;     ///<  Message response data.
            boost::mutex mConditionMutex;               ///<  Receipt mutex.
            boost::condition_variable mWaitCondition;   ///<  Used to notify when data is ready.
        };
        // Recursively try to create the message.
        Message* CreateMessageFromService(const UShort messageCode, const Service* service) const;
        // Create a message using templates.
        Message* GetMessageFromTemplate(const UShort messageCode) const;
        // Gets a message structure for reading packet data (pre-allocated).
        Message* GetCachedMessage(const UShort messageCode);
        // Processes a single packet.
        virtual void ProcessSinglePacket(Packet& packet,
                                         Header& header,
                                         UShort messageCode);
        // Process multi-packet stream data
        virtual void ProcessMultiPacket(Packet& packet,
                                        Header& header,
                                        UShort messageCode);
        // Check for a thread/procedure call waiting for an incoming message inline.
        bool CheckPendingReceipts(const Header& header, const UShort messageCode, const Packet& packet);
        // Thread function to check for new data to process.
        void MessageProcessingThread();
        boost::mutex mProcessingMutex;                        ///<  Mutex used for message queue.
        boost::condition_variable mMessageAvailableCondition; ///<  Used to notify when data is ready.
        SharedMutex mMessageQueueMutex;                       ///<  Mutex for message queue.
        boost::circular_buffer<Packet> mMessageQueue;         ///<  Message buffer. 
        boost::thread mMessageProcessingThread;               ///<  Message processing thread object.
        volatile bool mStopMessageProcessingFlag;             ///<  Quit all message processing.
        std::string mSettingsFilename;                        ///< Filename for settings data.
        
        SharedMutex mPendingReceiptsMutex;                    ///<  Mutex for thread protection of receipts.
        Receipt::Set mPendingReceipts;                        ///<  List of blocking send calls waiting for responses.

        SharedMutex mMessageTemplatesMutex;                   ///<  Mutex for thread protection.
        std::map<UShort, boost::shared_ptr<Message> > 
            mMessageTemplates;                                ///<  Custom message templates.
        SharedMutex mMessageCacheMutex;                       ///<  Mutex for thread protection of message cache.
        std::map<UShort, boost::shared_ptr<Message> > 
            mMessageCache;                                    ///<  Pre-allocated memory for message decoding.

        Packet mLargeDataSetCache;                            ///<  Cache packet memory for merged large data sets.
        std::vector<LargeDataSet> mLargeDataSets;             ///<  Large data sets.
        Transport::Callback::Map mMessageCallbacks;           ///<  Map of message callbacks.
        SharedMutex mSequenceNumberMutex;                     ///<  Mutex for thread protection of sequence number.
        UShort mSequenceNumber;                               ///<  Message sequence number.
        
        unsigned int mDisconnectTimeMs;                       ///<  Disconnect time ms.
    };
}

#endif
/*  End of File */
