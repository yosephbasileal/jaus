////////////////////////////////////////////////////////////////////////////////////
///
///  \file SetLocalWaypoint.h
///  \brief This file contains the implementation of a JAUS message.
///
///  <br>Author(s): Bo Sun
///  <br>Created: 30 November 2009
///  <br>Copyright (c) 2009
///  <br>Applied Cognition and Training in Immersive Virtual Environments
///  <br>(ACTIVE) Laboratory
///  <br>Institute for Simulation and Training (IST)
///  <br>University of Central Florida (UCF)
///  <br>All rights reserved.
///  <br>Email: bsun@ist.ucf.edu
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
#ifndef __JAUS_MOBILITY_SET_LOCAL_WAYPOINT__H
#define __JAUS_MOBILITY_SET_LOCAL_WAYPOINT__H

#include "jaus/core/Message.h"
#include "jaus/mobility/MobilityCodes.h"
#include "jaus/mobility/drivers/LocalWaypoint.h"

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class SetLocalWaypoint
    ///   \brief This message is used to set waypoint data based on the local
    ///          coordinate system.  A local waypoint can have up to six fields to
    ///          describe it completely.  The waypoint is defined in the global
    ///          coordinate system using the X, Y, and Z fields.
    ///          The desired orientation of the platform at the waypoint is defined
    ///          using the roll, pitch, and yaw fields.  Only fields 1-3 (presence
    ///          vector, latitude, and longitude are required for each waypoint.
    ///          The presence vector is used to specify if the remaining
    ///          fields, i.e., Z, roll, pitch, and yaw, are used to further
    ///          describe orientation at that point.
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_MOBILITY_DLL SetLocalWaypoint : public Message,
        public LocalWaypoint
    {
    public:
        SetLocalWaypoint(const Address& dest = Address(), const Address& src = Address());
        SetLocalWaypoint(const SetLocalWaypoint& message);
        ~SetLocalWaypoint();
        virtual bool IsCommand() const { return true; }
        virtual int WriteMessageBody(Packet& packet) const;
        virtual int ReadMessageBody(const Packet& packet);
        virtual Message* Clone() const { return new SetLocalWaypoint(*this); }
        virtual UInt GetPresenceVector() const { return mPresenceVector; }
        virtual UInt GetPresenceVectorSize() const { return BYTE_SIZE; }
        virtual UInt GetPresenceVectorMask() const { return 0x3F; }
        virtual UShort GetMessageCodeOfResponse() const { return 0; }
        virtual std::string GetMessageName() const { return "Set Local Waypoint"; }        
        virtual void ClearMessageBody();
        virtual bool IsLargeDataSet(const unsigned int maxPayloadSize = 1437) const { return false; }
        virtual int RunTestCase() const;
        void PrintMessageBody() const;
        SetLocalWaypoint& operator=(const SetLocalWaypoint& message);
        SetLocalWaypoint& operator=(const LocalWaypoint& message);
    };
}

#endif
/*  End of File */
