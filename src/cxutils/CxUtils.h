////////////////////////////////////////////////////////////////////////////////////
///
///  \file CxUtils.h
///  \brief Main include file for the CxUtils (Cross-Platform Utilties) library.
///
///  <br>Author(s): Daniel Barber
///  <br>Created: 24 April 2007
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
#ifndef __CXUTILS_H
#define __CXUTILS_H

#include "cxutils/Time.h"               //  Time information.
#include "cxutils/Thread.h"             //  Thread class.
#include "cxutils/Mutex.h"              //  Mutex class.
#include "cxutils/Packet.h"             //  Packet class for storing buffered data.
#include "cxutils/Serial.h"             //  Serial class interface.

#include "cxutils/FileIO.h"             //  File IO functions.
#include "cxutils/Timer.h"              //  Precise timing operations.
#include "cxutils/Joystick.h"           //  Joystick interface class.
#include "cxutils/Keyboard.h"           //  Keyboard functions.
#include "cxutils/Mouse.h"              //  Mouse input functions.

#include "CircularArray.h"      // Circular array template for fixed memory size, but fast lookup array.

#include "math/CxMath.h"        //  Useful math tools.

#include "images/Image.h"       //  Some useful image compression/decompression routines.

// Networking Tools
#include "networking/Socket.h"             //  Socket base class for networking.
#include "networking/UDPClient.h"          //  UDP client connections.
#include "networking/UDPServer.h"          //  UDP server connections.
#include "networking/TCPListenSocket.h"    //  Create TCP server connections.
#include "networking/TCPServer.h"          //  TCP server connections.
#include "networking/TCPClient.h"          //  TCP client connections.
#include "networking/UDPSharedServer.h"    //  Multi-Process UDP Server.

// Interprocess communication.
#include "ipc/MappedMemory.h"       //  Shared/mapped memory class.
#include "ipc/MappedMessageBox.h"   //  Shared/mapped memory message box.
#include "ipc/MessageServer.h"      //  Shared/mapped memory message server.
#include "ipc/MessageClient.h"      //  Shared/mapped memory message client.

#endif  
/*  End of File */
