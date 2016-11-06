////////////////////////////////////////////////////////////////////////////////////
///
///  \file SocketData.h
///  \brief The SocketData class is a file internal to CxUtils Networking
///         which includes all necessary header files for socket and
///         other information shared by all sockets.
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
#ifndef __CXUTILS_SOCKET_DATA__H
#define __CXUTILS_SOCKET_DATA__H

#ifdef WIN32
    #include <winsock.h>
#else
    #include <errno.h>
    #include <sys/ioctl.h>
    #include <sys/types.h>
    #include <unistd.h>
    #include <netinet/in.h>
    #include <sys/socket.h>
    #include <netdb.h>
    #include <arpa/inet.h>
    #include <netinet/tcp.h>
    #include <cstring>
#endif

namespace CxUtils
{
    /**
        \class SocketData
        \brief Contains OS/socket library specific files.
        This file is not exported with the rest of the library
        and should only be included within this library to avoid
        conflicts with Windows files, etc.
    */
    class SocketData
    {
    public:
        SocketData()
        {
            mHP = 0;
            memset((void *)&mSendAddr, 0, sizeof(struct sockaddr_in));
            memset((void *)&mRecvAddr, 0, sizeof(struct sockaddr_in));
            memset((void *)&mService, 0, sizeof(struct sockaddr_in));
            mSocket = 0;
        }
        ~SocketData() {}
        struct hostent* mHP;           ///<  Used for converting computer name/ip address to integer value.
        struct sockaddr_in mSendAddr;  ///<  Address to send to on network.
        struct sockaddr_in mRecvAddr;  ///<  Address data received from.
#ifdef WIN32
        SOCKET mSocket;                ///<  Socket file descripter.
#else
        int    mSocket;                ///<  Socket file descripter.
#endif
        sockaddr_in mService;          ///<  Type of socket and service being used.
    };
}


#endif
/*  End of File */
