////////////////////////////////////////////////////////////////////////////////////
///
///  \file JUDP.cpp
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
#include "jaus/core/transport/JUDP.h"
#include <tinyxml.h>

using namespace JAUS;
using namespace boost::asio;
using namespace boost::asio::ip;

JUDP::JUDP() 
    : mQuitReceiveThreadFlag(true),
    mDefaultPortNumber(Port),
    mMaxPacketSizeBytes(65535),
    mTTL(16),
    mMulticastGroup("239.255.0.1"),
    mLocalAddress("0.0.0.0"),
    mUseBroadcastingFlag(false)
{
    mTransportHeader.Write(Version);
}


JUDP::~JUDP()
{
}


/**
    Initializes transport services for a given JAUS component ID.
    
    \param[in] componentID The component this transport service represents.
*/
bool JUDP::Initialize(const Address& componentID)
{
    try
    {
        Transport::Initialize(componentID);

        // Initialize discovery socket.
        mDiscoverySocket = 
            boost::shared_ptr<ip::udp::socket>(new udp::socket(mDiscoveryIO));
        // Setup the multicast endpoint.
        ip::address multicastAddress = 
            ip::address::from_string(mMulticastGroup);
        mMulticastEndpoint = 
            udp::endpoint(multicastAddress, mDefaultPortNumber);
        mDiscoverySocket->open(mMulticastEndpoint.protocol());
        // Make sure others can listen on this address.
        mDiscoverySocket->set_option(udp::socket::reuse_address(true));
        // Bind to port
        mDiscoverySocket->bind(
            udp::endpoint(ip::address::from_string(mLocalAddress),
                          mDefaultPortNumber));
        // Leave/Force Join the multicast group
        try
        {
            mDiscoverySocket->set_option(ip::multicast::leave_group(multicastAddress));
        }
        catch (...)
        {
        }
        mDiscoverySocket->set_option(ip::multicast::join_group(multicastAddress));
        mDiscoverySocket->set_option(ip::multicast::enable_loopback(true));
        mDiscoverySocket->set_option(ip::multicast::hops(mTTL));
        // Increase max receive size for buffer
        boost::asio::socket_base::receive_buffer_size receiveSize(JAUS_USHORT_MAX);
        mDiscoverySocket->set_option(receiveSize);

        // Initialize our normal traffic socket.
        mSocket = 
            boost::shared_ptr<udp::socket>(new udp::socket(mSocketIO, 
                                            mMulticastEndpoint.protocol()));
        // Bind to any available source port.
        mSocket->bind(udp::endpoint(ip::address::from_string(mLocalAddress),
                      0));
        // Increase max receive size for buffer
        mSocket->set_option(receiveSize);
        mSocket->set_option(ip::multicast::enable_loopback(true));
        mSocket->set_option(socket_base::broadcast(true));
        mBroadcastEndpoint = 
            udp::endpoint(ip::address_v4::broadcast(),
                            mDefaultPortNumber);

        // Now start our receiving threads.
        mQuitReceiveThreadFlag = false;
        mReceiveThread = 
            boost::thread(boost::bind(&JUDP::ReceiveThread, this));
        mDiscoveryReceiveThread = 
            boost::thread(boost::bind(&JUDP::DiscoveryReceiveThread, this));

        // Set thread name for debugging purposes (win32 only).
        SetThreadName(mReceiveThread,
                      mComponentID.ToString() + ":JUDP");
        SetThreadName(mDiscoveryReceiveThread,
                      mComponentID.ToString() +":JUDP Discover");
        return true;
    }
    catch(std::exception& e)
    {
        std::cout << "JAUS::JUDP::Initialize::ERROR - " << e.what() << std::endl;
    }
    return false;
}


/**
    Closes the transport services.
*/
void JUDP::Shutdown()
{
    Transport::Shutdown();
    try
    {
        mQuitReceiveThreadFlag = true;
        if(mSocket != NULL)
        {
            mSocket->shutdown(boost::asio::socket_base::shutdown_both);
            mSocket->close();
        }
        if(mDiscoverySocket != NULL)
        {
            mDiscoverySocket->set_option(ip::multicast::leave_group(ip::address::from_string(mMulticastGroup)));
            mDiscoverySocket->shutdown(boost::asio::socket_base::shutdown_both);
            mDiscoverySocket->close();
        }

        mReceiveThread.join();
        mReceiveThread.detach();
        mDiscoveryReceiveThread.join();
        mDiscoveryReceiveThread.detach();
        mDiscoverySocket.reset();
        mSocket.reset();
    }
    catch(...)
    {
    }
}


/**
    Loads data regarding this transport from an XML file.

    \param[in] filename Name of the file to load from.

    \return True on success, false on failure.
*/
bool JUDP::LoadSettings(const std::string& filename)
{
    TiXmlDocument xml;

    if(xml.LoadFile(filename.c_str()) == false)
    {
        return false;
    }

    TiXmlHandle doc(&xml);

    TiXmlElement* element = doc.FirstChild("JAUS").FirstChild("Transport").FirstChild("DefaultTransport").ToElement();

    TiXmlNode* node;
    node = doc.FirstChild("JAUS").FirstChild("Transport").FirstChild("DefaultPort").FirstChild().ToNode();
    if(node && node->Value() && atoi(node->Value()) >= 0)
    {
        mDefaultPortNumber = (UShort)atoi(node->Value());
    }

    node = doc.FirstChild("JAUS").FirstChild("Transport").FirstChild("MulticastIP").FirstChild().ToNode();
    if(node && node->Value())
    {
        mMulticastGroup = node->Value();
    }

    node = doc.FirstChild("JAUS").FirstChild("Transport").FirstChild("NetAddress").FirstChild().ToNode();
    if(node && node->Value())
    {
        mLocalAddress = node->Value();
        if(mLocalAddress.empty() || mLocalAddress.size() < 1)
        {
            mLocalAddress = "0.0.0.0";
        }
    }

    node = doc.FirstChild("JAUS").FirstChild("Transport").FirstChild("TTL").FirstChild().ToNode();
    if(node && node->Value())
    {
        mTTL = atoi(node->Value());
    }

    element = doc.FirstChild("JAUS").FirstChild("Transport").FirstChild("Connection").ToElement();
    while(element)
    {
        if(element->Attribute("ip") && element->Attribute("id"))
        {
            unsigned short port = 0;
            std::string ipAddress;
            if(element->Attribute("port"))
            {
                port = (unsigned short)atoi(element->Attribute("port"));
            }
            if(element->Attribute("dest_port"))
            {
                port = (unsigned short)atoi(element->Attribute("port"));
            }
            ipAddress = element->Attribute("ip");
            Address id = Address::FromString(element->Attribute("id"));
            
            if(id.IsValid() && id.IsBroadcast() == false &&
                port != 0 && ipAddress.empty() == false)
            {
                mDictionary[id] = udp::endpoint(ip::address::from_string(ipAddress), port);
            }
        }
        element = element->NextSiblingElement("Connection");
    }

    return true;
}


///////////////////////////////////////////////////////////////////////////////
///
///   \brief Sends a serialized JAUS packet that has the correct JUDP header
///          setup.  If the destination is a local broadcast, then it is sent
///          using multicast, if destination is global, then it is sent using
///          a broadcast socket.
///
///   \param[in] packet JUDP Packet to send.
///   \param[in] header JAUS Transport header information.
///
///   \return True on success, false on failure.
///
///////////////////////////////////////////////////////////////////////////////
bool JUDP::SendPacket(const Packet& packet,
                      const Header& header) const
{
    bool result = false;
    if(mSocket == NULL)
        return result;

    try
    {
        if(header.mDestinationID.IsBroadcast())
        {
            if(mUseBroadcastingFlag || 
                header.mBroadcastFlag == Header::Broadcast::Global)
            {
                mSocket->send_to(buffer((char*)packet.Ptr(),
                                 (size_t)packet.Length()),
                             mBroadcastEndpoint);
            }
            else if(header.mBroadcastFlag == Header::Broadcast::Local)
            {
                mSocket->send_to(buffer((char*)packet.Ptr(),
                                 (size_t)packet.Length()),
                                 mMulticastEndpoint);
            }
            // If no broadcast flag is indicated, then we must
            // loop through all remote destinations and send
            // using unicast.
            else
            {
                std::map<Address, udp::endpoint> dictionary;
                {
                    ReadLock rLock(*((SharedMutex*)(&mDictionaryMutex)));
                    dictionary = mDictionary;
                }
                std::map<Address, udp::endpoint>::iterator remoteEndpoint;
                for(remoteEndpoint = dictionary.begin();
                    remoteEndpoint != dictionary.end();
                    remoteEndpoint++)
                {
                    mSocket->send_to(buffer((char*)packet.Ptr(),
                                 (size_t)packet.Length()),
                             remoteEndpoint->second);
                }
            }
        }
        else
        {
            // Lookup destination using dictionary.
            udp::endpoint remoteEndpoint;
            {
                std::map<Address, udp::endpoint>::const_iterator dest;
                ReadLock rLock(*((SharedMutex*)(&mDictionaryMutex)));
                dest = mDictionary.find(header.mDestinationID);
                if(dest == mDictionary.end())
                {
                    PrintDebugMessage("JUDP::SendPacket - Destination does not exist.");
                    return false;
                }
                remoteEndpoint = dest->second;
            }
            // Now that we have a remote endpoint
            // send the packet.
            mSocket->send_to(buffer((char*)packet.Ptr(),
                                 (size_t)packet.Length()),
                             remoteEndpoint);
        }
        result = true;
    }
    catch(std::exception& e)
    {
        std::cout << "JUDP::SendPacket - ERROR - " << e.what();
        result = false;
    }
    return result;
}


///////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the message data into packets for transmission over
///          the protocol in use.
///
///   \param[in] message Message to convert/serialize.
///   \param[out] stream Packets to send.
///   \param[out] streamHeaders Header info for packets.
///   \param[in] startingSequenceNumber Sequence number to use for packets.
///   \param[in] broadcastFlags Values to use to signify if message should be
///                           sent using any broadcast options (e.g.
///                           multicast). 0 = no options, 1 = local broadcast,
///                           2 = global broadcast.
///
///   \return True on success, false on failure.
///
///////////////////////////////////////////////////////////////////////////////
bool JUDP::SerializeMessage(const Message* message,
                            Packet::List& stream,
                            Header::List& streamHeaders,
                            const UShort startingSequenceNumber,
                            const int broadcastFlags) const
{
    Packet packet;
    Header header;

    // Pre-allocate some memory.
    packet.Reserve(256);

    // Clear stream/headers.
    stream.clear();
    streamHeaders.clear();
    // Best Size is the maximum transport size minus the size
    // of the transport overhead and version number field.
    unsigned int bestPacketSize = 
        mMaxPacketSizeBytes - OverheadSizeBytes - USHORT_SIZE;
    // If the message is a large data set, create a multi-packet stream.
    if(message->IsLargeDataSet(bestPacketSize))
    {
        return message->WriteLargeDataSet(stream,
                                          streamHeaders,
                                          bestPacketSize,
                                          &mTransportHeader,
                                          startingSequenceNumber) > 0;
    }
    // Single packet.
    else if(message->Write(packet, 
                            header, 
                            &mTransportHeader, 
                            true, 
                            startingSequenceNumber, 
                            (Byte)broadcastFlags) > 0)
    {
        stream.push_back(packet);
        streamHeaders.push_back(header);
        return true;
    }
    return false;
}


/** Gets the maximum packet size for JUDP based on configuration and
    actual transport. */
unsigned int JUDP::GetMaximumPacketSizeBytes() const
{
    return mMaxPacketSizeBytes - OverheadSizeBytes - USHORT_SIZE;
}


///////////////////////////////////////////////////////////////////////////////
///
///   \brief Tries to create/add a new fixed/permanent connection 
///          to a component ID.
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
///////////////////////////////////////////////////////////////////////////////
bool JUDP::AddNetworkConnection(const Address& id,
                                     const std::string& destinationIP,
                                     const unsigned short port)
{
    udp::endpoint dest = 
        udp::endpoint(ip::address::from_string(destinationIP), port);
    WriteLock wLock(mDictionaryMutex);
    mDictionary[id] = dest;
    return true;
}


/**
    \brief A thread to continuosly receive
           UDP messages.
*/
void JUDP::ReceiveThread()
{
    mReceiveBuffer.Reserve(JAUS_USHORT_MAX);
    while(mQuitReceiveThreadFlag == false)
    {
        if(mStartTransportServicesFlag == false)
        {
            boost::this_thread::sleep(boost::posix_time::millisec(1));
            continue;
        }
        try
        {
            boost::asio::ip::udp::endpoint remoteEndpoint;
            boost::system::error_code error;

            std::size_t bytesReceived = 
                mSocket->receive_from(
                    boost::asio::buffer((char *)mReceiveBuffer.Ptr(),
                                        mReceiveBuffer.Reserved()),
                    remoteEndpoint, 0, error);
            
            if(bytesReceived > 0)
            {
                mReceiveBuffer.SetLength((unsigned int)bytesReceived);
            }
            else
            {
                continue;
            }
            ExtractMessages(mReceiveBuffer, remoteEndpoint);
        }
        catch(...)
        {
        }
    }
}


/**
    \brief A thread to continuosly receive
           UDP messages.
*/
void JUDP::DiscoveryReceiveThread()
{
    mDiscoveryBuffer.Reserve(JAUS_USHORT_MAX);
    while(mQuitReceiveThreadFlag == false)
    {
        if(mStartTransportServicesFlag == false)
        {
            boost::this_thread::sleep(boost::posix_time::millisec(1));
            continue;
        }
        try
        {
            boost::asio::ip::udp::endpoint remoteEndpoint;
            boost::system::error_code error;

            std::size_t bytesReceived = 
                mDiscoverySocket->receive_from(
                    boost::asio::buffer((char *)mDiscoveryBuffer.Ptr(),
                                        mDiscoveryBuffer.Reserved()),
                    remoteEndpoint, 0, error);
            
            if(bytesReceived > 0)
            {
                mDiscoveryBuffer.SetLength((unsigned int)bytesReceived);
            }
            else
            {
                continue;
            }
            ExtractMessages(mDiscoveryBuffer, remoteEndpoint);
        }
        catch(...)
        {
        }
    }
}


/**
    \brief Extracts jaus messages from the buffer (if multiple)
    and updates internal dictionary of other UDP destingations and
    processes the data.

    \param[in] buffer UDP data received that needs to be parsed.
    \param[in] remoteEndpoint The network address where the data
                originated from.
*/
void JUDP::ExtractMessages(Packet& buffer,
                           boost::asio::ip::udp::endpoint & remoteEndpoint)
{
    unsigned char* ptr = buffer.Ptr();
    unsigned int position = 0;
    unsigned int length = buffer.Length();
    while(position < length)
    {
        if( (position + Header::MinSize + BYTE_SIZE) > length ||
            *ptr != Version)
        {
            break;
        }

        position += BYTE_SIZE;
        ptr += BYTE_SIZE;
        JAUS::Header jausHeader;
        Packet::Wrapper subPacket(ptr, length - position);
        jausHeader.Read(*subPacket.GetData());
        if(jausHeader.IsValid(NULL) == false ||
           subPacket->Length() < jausHeader.mSize ||
           jausHeader.mSourceID == mComponentID)
        {
            break;
        }
        // Update dictionary of remote endpoints.
        {
            WriteLock wLock(mDictionaryMutex);
            mDictionary[jausHeader.mSourceID] = remoteEndpoint;
        }
        // Wrap the extracted packet.
        Packet::Wrapper jausPacket(ptr,
                                   (unsigned int)jausHeader.mSize);

        // Advance the pointers
        ptr += jausHeader.mSize;
        position += (unsigned int)jausHeader.mSize;

        // Only process the message if we are it's
        // destingation.
        if(Address::DestinationMatch(jausHeader.mDestinationID,
                                     mComponentID))
        {
            // Process the data...
            ProcessPacket(*jausPacket.GetData(),
                          jausHeader);
        }
    }
}


/** End of File */
