////////////////////////////////////////////////////////////////////////////////////
///
///  \file JUDP.h
///  \brief JUDP interface for transport services.
///
///  <br>Author(s): Daniel Barber
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
#ifndef __JAUS_CORE_TRANSPORT_JUDP_CONNECTION__H
#define __JAUS_CORE_TRANSPORT_JUDP_CONNECTION__H

#include "jaus/core/transport/Transport.h"
#include <boost/thread/thread.hpp>

#ifdef WIN32
#define _WIN32_WINNT 0x0501
#endif

#include <boost/asio.hpp>


namespace JAUS
{
    /**
        \class JUDP
        \brief Implements the JAUS UDP Transport specifications.
    */
    class JAUS_CORE_DLL JUDP : public Transport
    {
    public:
        const static unsigned short Port = 3794;            ///< JAUS UDP/UDP Port Number == "jaus".
        const static unsigned int OverheadSizeBytes = 61;   ///< Total overhead in bytes include JAUS General header and JUDP 
        const static Byte Version = 0x02;                   ///< JUDP Header Version.
        JUDP();
        virtual ~JUDP();
        // Initializes the transport with a given ID for a component.
        virtual bool Initialize(const Address& componentID);
        // Returns true if transport has been initialized.
        virtual bool IsInitialized() const { return mQuitReceiveThreadFlag == false; }
        // Shutdown the transport service.
        virtual void Shutdown();
        // Loads settings
        virtual bool LoadSettings(const std::string& filename);
        // Send a serialized message.
        virtual bool SendPacket(const Packet& packet, 
                                const Header& header)  const;
        // Serialize the JAUS message (add transport headers, serialize payload, etc.) 
        virtual bool SerializeMessage(const Message* message, 
                                      Packet::List& stream,
                                      Header::List& streamHeaders,
                                      const UShort startingSequenceNumber,
                                      const int broadcastFlags) const;
        // Get the size in bytes the transport adds to each packet.
        // Get the packet size this transport allows.
        virtual unsigned int GetMaximumPacketSizeBytes() const;
        // Adds a new connection and tries to connect.
        virtual bool AddNetworkConnection(const Address& id,
                                          const std::string& destinationIP,
                                          const unsigned short port);
    protected:
        virtual const Packet& GetTransportHeader() const { return mTransportHeader; }
        void ReceiveThread();
        void DiscoveryReceiveThread();
        void ExtractMessages(Packet& buffer, 
            boost::asio::ip::udp::endpoint & remoteEndpoint);
        volatile bool 
            mQuitReceiveThreadFlag;     ///< Indicates shutdown signal for threads.
        boost::thread 
            mReceiveThread;             ///< Thread object to receive UDP data.
        boost::thread 
            mDiscoveryReceiveThread;    ///< Thread object to receive UDP data.
        boost::asio::io_service 
            mSocketIO;                  ///< Socket IO interface.
        boost::asio::io_service 
            mDiscoveryIO;               ///< Socket IO interface.
        boost::shared_ptr<boost::asio::ip::udp::socket> 
            mSocket;                    ///< UDP Socket for transport
        boost::shared_ptr<boost::asio::ip::udp::socket> 
            mDiscoverySocket;           ///< Socket to receive multicast messages sent to 3794.
        SharedMutex 
            mDictionaryMutex;           ///< Mutex for destination lookup protection.
        std::map<Address, boost::asio::ip::udp::endpoint> 
            mDictionary;                ///< Stores address of all discovered transports.
        Packet mTransportHeader;        ///< Stores JUDP header
        Packet mDiscoveryBuffer;        ///< Buffer for storing data.
        Packet mReceiveBuffer;          ///< Buffer for storing data.
        int mMaxPacketSizeBytes;        ///< Maximum send size for a packet.
        int mTTL;                       ///< Multicast TTL.
        std::string mMulticastGroup;    ///< Multicast group IP.
        std::string mLocalAddress;      ///< Local IP address to use.
        unsigned short mDefaultPortNumber;  ///< JUDP Port Number.
        bool mUseBroadcastingFlag;          ///< Always use broadcasting.
        boost::asio::ip::udp::endpoint mBroadcastEndpoint;  ///< Broadacst endpoint.
        boost::asio::ip::udp::endpoint mMulticastEndpoint;  ///< Multicast endpoint.
    };
}


#endif
/*  End of File */