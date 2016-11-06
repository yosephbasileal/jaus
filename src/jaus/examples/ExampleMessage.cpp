////////////////////////////////////////////////////////////////////////////////////
///
///  \file ExampleMessage.cpp
///  \brief This file contains a test/example program for the Message class
///         to verify ability to serialize/deserialize message data to a 
///         JAUS compliant format.
///
///  <br>Author(s): Daniel Barber
///  <br>Created: 26 September 2009
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
#include "jaus/core/transport/LargeDataSet.h"
#include "jaus/core/liveness/Liveness.h"
#include "jaus/core/discovery/Discovery.h"
#include "jaus/core/control/AccessControl.h"
#include "jaus/core/management/Management.h"
#include "jaus/core/events/Events.h"
#include "jaus/core/events/CancelEvent.h"
#include "jaus/core/events/ConfirmEventRequest.h"
#include "jaus/core/events/CreateEvent.h"
#include "jaus/core/events/Event.h"
#include "jaus/core/events/QueryEvents.h"
#include "jaus/core/events/RejectEventRequest.h"
#include "jaus/core/events/ReportEvents.h"
#include "jaus/core/events/UpdateEvent.h"
#include "jaus/mobility/list/ListManager.h"
#include "jaus/mobility/sensors/LocalPoseSensor.h"
#include "jaus/mobility/sensors/VelocityStateSensor.h"
#include "jaus/mobility/drivers/LocalWaypointListDriver.h"
#include "jaus/environment/analogvideo/QueryAnalogVideoSensorCapabilities.h"
#include "jaus/environment/analogvideo/QueryAnalogVideoSensorConfigurations.h"
#include "jaus/environment/analogvideo/ReportAnalogVideoSensorCapabilities.h"
#include "jaus/environment/analogvideo/ReportAnalogVideoSensorConfigurations.h"
#include "jaus/environment/analogvideo/SetAnalogVideoSensorConfigurations.h"
#include "jaus/environment/digitalvideo/ControlDigitalVideoSensor.h"
#include "jaus/environment/digitalvideo/QueryDigitalVideoSensorCapabilities.h"
#include "jaus/environment/digitalvideo/QueryDigitalVideoSensorConfigurations.h"
#include "jaus/environment/digitalvideo/ReportDigitalVideoSensorCapabilities.h"
#include "jaus/environment/digitalvideo/ReportDigitalVideoSensorConfigurations.h"
#include "jaus/environment/digitalvideo/SetDigitalVideoSensorConfigurations.h"
#include "jaus/environment/geometric/GeometricProperties.h"
#include "jaus/environment/geometric/ManipulatorGeometricProperties.h"
#include "jaus/environment/geometric/QuerySensorGeometricProperties.h"
#include "jaus/environment/geometric/ReportSensorGeometricProperties.h"
#include "jaus/environment/geometric/StaticGeometricProperties.h"
#include "jaus/environment/range/QueryRangeSensorCapabilities.h"
#include "jaus/environment/range/QueryRangeSensorCompressedData.h"
#include "jaus/environment/range/QueryRangeSensorConfigurations.h"
#include "jaus/environment/range/QueryRangeSensorData.h"
#include "jaus/environment/range/Range.h"
#include "jaus/environment/range/RangeScan.h"
#include "jaus/environment/range/RangeSensor.h"
#include "jaus/environment/range/RangeSensorCapabilities.h"
#include "jaus/environment/range/RangeSensorConfiguration.h"
#include "jaus/environment/range/RangeSubscriber.h"
#include "jaus/environment/range/SetRangeSensorConfigurations.h"
#include "jaus/environment/range/ReportRangeSensorCapabilities.h"
#include "jaus/environment/range/ReportRangeSensorCompressedData.h"
#include "jaus/environment/range/ReportRangeSensorConfigurations.h"
#include "jaus/environment/range/ReportRangeSensorData.h"
#include "jaus/environment/stillimage/QueryStillImageData.h"
#include "jaus/environment/stillimage/QueryStillImageSensorConfigurations.h"
#include "jaus/environment/stillimage/QueryStillImageSensorCapabilities.h"
#include "jaus/environment/stillimage/ReportStillImageData.h"
#include "jaus/environment/stillimage/ReportStillImageSensorCapabilities.h"
#include "jaus/environment/stillimage/ReportStillImageSensorConfigurations.h"
#include "jaus/environment/stillimage/SetStillImageSensorConfigurations.h"
#include "jaus/environment/stillimage/StillImageData.h"
#include "jaus/environment/stillimage/StillImageSensorCapabilities.h"
#include "jaus/environment/stillimage/StillImageSensorConfiguration.h"
#include "jaus/environment/visual/QueryVisualSensorCapabilities.h"
#include "jaus/environment/visual/QueryVisualSensorConfigurations.h"
#include "jaus/environment/visual/ReportVisualSensorCapabilities.h"
#include "jaus/environment/visual/ReportVisualSensorConfigurations.h"
#include "jaus/environment/visual/SetVisualSensorConfigurations.h"
#include "jaus/environment/visual/VisualSensorCapabilities.h"
#include "jaus/environment/visual/VisualSensorConfiguration.h"
#include "jaus/core/Component.h"
#include <cxutils/Keyboard.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>

using namespace std;
using namespace JAUS;

void VerifyAgainstLog();
void TestMessages();
void TestLargeDataSets();
void TestMessage(Message* src, Message* dest);
void PrintMessage(Message* msg, std::string header);
void TestAnalogMessages();
void TestDigitalVideoMessages();
void TestGeometricMessages();
void TestRangeMessages();
void TestStillImageMessages();
void TestVisualMessages();


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Entry point of example_message.
///
////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
    //VerifyAgainstLog();
    //TestMessages();
    //TestLargeDataSets();

    // Run Tests for the Environment Sensing Service Set
    TestAnalogMessages(); 
    TestDigitalVideoMessages();
    TestGeometricMessages();
    TestRangeMessages();
    TestStillImageMessages();
    TestVisualMessages();
    //End of tests for Environment Sensing Service Set

    std::cout << "<<<<<<<<TEST COMPLETE>>>>>>>>>" << std::endl;
    system ("pause");
    return 0;
}


class TestLocalWaypointListDriver : public LocalWaypointListDriver
{
public:
    TestLocalWaypointListDriver() {}
    ~TestLocalWaypointListDriver() {}
    virtual bool IsWaypointAchieved(const LocalPose& currentPose,
                                    const JAUS::SetLocalWaypoint& desiredWaypoint) const { return false; }
    virtual void WaypointAchieved(const JAUS::SetLocalWaypoint& waypoint) { return ; }
};

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Test method to verify ability to read/write message data from
///   an example wireshark log provided for testing systems for the JAUS
///   Interoperability Challenge hosted at several AUVSI competitions.
///
////////////////////////////////////////////////////////////////////////////////////
void VerifyAgainstLog()
{
    Packet::List packets;
    Packet::List::iterator packet;

    Component component;
    component.DiscoveryService()->SetSubsystemIdentification(Subsystem::OCU, "OCP");
    component.AddService(new LocalPoseSensor());
    component.AddService(new VelocityStateSensor());
    component.AddService(new ListManager());    
    component.AddService(new TestLocalWaypointListDriver());

    Packet::LoadWiresharkCapturePacketExport("logs/example-capture.txt", packets);
    Address cop(42, 1, 1);
    Address sim(6000, 1, 1);
    //Packet::LoadWiresharkCapturePacketExport("logs/example_capture-2.txt", packets);

    Message::List messages;
    Message::List::iterator message;
    
    // Delete UDP header data to get to just the JAUS message.
    for(packet = packets.begin(); packet != packets.end();)
    {
        // Delete UDP header data.
        packet->Delete(43);
        // Read message header data.
        Header jausHeader;
        if(jausHeader.Read(*packet))
        {
            UShort messageCode = 0;
            packet->Read(messageCode, Header::PayloadOffset);
            Message* jausMessage = component.TransportService()->CreateMessage(messageCode);
            if(jausMessage)
            {
                packet->SetReadPos(0);
                if(jausMessage->Read(*packet))
                {
                    messages.push_back(jausMessage);
                    if(jausMessage->GetMessageCode() == SET_ELEMENT)
                    {
                        Element::List::iterator m;
                        SetElement* setElement = (SetElement *)jausMessage;
                        for(m = setElement->GetElementList()->begin();
                            m != setElement->GetElementList()->end();
                            m++)
                        {
                            UShort code = 0;
                            m->mPayload.SetReadPos(0);
                            m->mPayload.Read(code);
                            Message* element = component.TransportService()->CreateMessage(code);
                            if(element)
                            {
                                element->CopyHeaderData(jausMessage);
                                if(element->ReadMessageBody(m->mPayload))
                                {
                                    messages.push_back(element);
                                }
                                else
                                {
                                    std::cout << "Failed to Read Message Data [" << std::hex << element->GetMessageCode() << "]:\n";
                                    delete element;
                                }
                            }
                        }
                    }
                }
                else
                {
                    std::cout << "Failed to Read Message Data [" << std::hex << jausMessage->GetMessageCode() << "]:\n";
                }
            }
            else
            {
                std::cout << "Unknown Message Type [" << std::hex << messageCode << "]:\n";
            }
        }
        else
        {
            std::cout << "Bad Header!\n";
        }
        packet++;
    }

    component.Initialize(cop);

    message = messages.begin();
    for(message = messages.begin(); 
        message != messages.end(); 
        message++)
    {
        //(*message)->Print();
        if((*message)->GetSourceID() == cop)
        {
            (*message)->SetDestinationID(sim);
            component.Send((*message));
        }
        CxUtils::SleepMs(1);
        delete (*message);
    }
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Calls RunTestCase for each message in testMessages.
///
////////////////////////////////////////////////////////////////////////////////////
void TestMessages()
{
    // Test Messages.
    std::vector<Message*> testMessages;

    // Message(s) being tested.
    testMessages.push_back(new QueryAnalogVideoSensorCapabilities());
    testMessages.push_back(new DeleteElement());
    testMessages.push_back(new ExecuteList());
    testMessages.push_back(new QueryActiveElement());
    testMessages.push_back(new QueryElement());
    testMessages.push_back(new QueryElementCount());
    testMessages.push_back(new QueryElementList());
    testMessages.push_back(new RejectElementRequest());
    testMessages.push_back(new ReportActiveElement());
    testMessages.push_back(new ReportElement());
    testMessages.push_back(new ReportElementCount());
    testMessages.push_back(new ReportElementList());
    testMessages.push_back(new SetElement());

    int success = 0;

    for(std::vector<Message*>::iterator ii = testMessages.begin(); ii < testMessages.end(); ++ii)
    {
        success += (*ii)->RunTestCase();
    }

    std::cout << success << " out of " << testMessages.size() << " Messages tested successful.\n";

    for(std::vector<Message*>::iterator ii = testMessages.begin(); ii < testMessages.end(); ++ii)
    {
        delete (*ii);
    }
    testMessages.clear();
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Tests large data set creation and merging.
///
////////////////////////////////////////////////////////////////////////////////////
void TestLargeDataSets()
{
    Packet payload;                         // Large data set payload.
    Header header;                          // Message header data.
    UShort messageCode = 50;                // Random message type.
    Packet::List stream;                    // Multi-packet stream sequence.
    Header::List streamHeaders;             // Mutli-packet stream sequence headers.
    unsigned int payloadSize = 60000;       // Payload data size to create.
    
    header.mSourceID(50, 1, 1);
    header.mDestinationID(51, 1, 1);
    for(unsigned int i = 0; i < payloadSize/UINT_SIZE; i++)
    {
        payload.Write(i);
    }
    
    LargeDataSet::CreateLargeDataSet(header, messageCode, payload, stream, streamHeaders, NULL, 1437, 30);
    payload.Destroy();
    messageCode = 0;
    header = Header();
    LargeDataSet::MergeLargeDataSet(header, messageCode, payload, stream, NULL);
    
    unsigned int data;
    for(unsigned int i = 0; i < payloadSize/UINT_SIZE; i++)
    {
        payload.Read(data);
        if(data != i)
        {
            std::cout << "Large Data Sets Error: Data Does Not Match!\n";
            return;
        }
    }

    std::random_shuffle(stream.begin(), stream.end());

    LargeDataSet dataSet;
    for(unsigned int i = 0; i < (unsigned int)stream.size(); i++)
    {
        Header header;
        UShort messageCode;
        stream[i].SetReadPos(0);
        header.Read(stream[i]);
        stream[i].Read(messageCode);
        if(dataSet.AddPacket(header, messageCode, stream[i]) == false)
        {
            std::cout << "Large Data Sets Error: Could Not Collect Stream.\n";
        }
    }
    // Now merge the data.
    LargeDataSet::MergeLargeDataSet(header, messageCode, payload, dataSet.mStream, NULL);
    for(unsigned int i = 0; i < payloadSize/UINT_SIZE; i++)
    {
        payload.Read(data);
        if(data != i)
        {
            std::cout << "Large Data Sets Error: Data Does Not Match!\n";
            return;
        }
    }
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Simple Test for messages.
///
////////////////////////////////////////////////////////////////////////////////////
void TestMessage(Message* src, Message* dest)
{
    PrintMessage(src, "Source Message"); //Print the source Message
    PrintMessage(dest,"Destination Message"); //Print the source Message

    // Test writing and read body methods
    Packet packet;
    src->WriteMessageBody(packet);
    dest->ReadMessageBody(packet);

    PrintMessage(dest, "Destination after reading packet from source.");

    //ClearMessageBody
    dest->ClearMessageBody();
    PrintMessage(dest, "Destination after clear body.");
    

    //test Clone
    dest = src->Clone();
    PrintMessage(dest, "Destination after cloned from source");

    //IsLargeDataSet
    bool isLarge = dest->IsLargeDataSet(100000);
    std::cout << "Is Large Test - should be false: " << isLarge << std::endl;
    isLarge = dest->IsLargeDataSet(1);
    std::cout << "Is Large Test - should be true: " << isLarge << std::endl;
    
    delete src;
    delete dest;
}

void PrintMessage(Message* msg, std::string header)
{
    std::cout << std::endl << header << std::endl
              << "Message Type: " << msg->GetMessageName() << std::endl
              << "Message ID: " << (UInt) msg->GetMessageCodeOfResponse() << std::endl
              << "Is Command? " << (UInt) msg->IsCommand() << std::endl
              << "Presence Vector: " << (UInt) msg->GetPresenceVector()
              << " VectorSize: " << (UInt) msg->GetPresenceVectorSize() 
              << " Vector Mask: " << (UInt) msg->GetPresenceVectorMask() << std::endl;

    msg->PrintMessageBody();
    std::cout << std::endl;
}

void TestAnalogMessages()
{
    AnalogVideoSensorCapabilities sensor1;
    AnalogVideoSensorCapabilities sensor2;
    AnalogVideoSensorConfiguration sensor3;
    AnalogVideoSensorConfiguration sensor4;

    //Test Query Analog Video Sensor Capabilities
    std::cout << "****Test for Query Analog Video Sensor Capabilities****" << std::endl;
    QueryAnalogVideoSensorCapabilities* src1 = new QueryAnalogVideoSensorCapabilities(Address((UInt)1234),Address((UInt)4321));
    Message* dest1 = new QueryAnalogVideoSensorCapabilities(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message; this tests the AddSensor method
    sensor1 = AnalogVideoSensorCapabilities(1234,5);
    sensor2 = AnalogVideoSensorCapabilities(4321,7);
    src1->AddSensor(sensor1);
    src1->AddSensor(sensor2);
    TestMessage(src1, dest1);
    system ("pause");

    //Test Query Analog Video Sensor Configurations
    std::cout << "****Test for Query Analog Video Sensor Configurations****" << std::endl;
    QueryAnalogVideoSensorConfigurations* src2 = new QueryAnalogVideoSensorConfigurations(Address((UInt)1234),Address((UInt)4321));
    Message* dest2 = new QueryAnalogVideoSensorConfigurations(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message; this tests the AddSensor method
    sensor3 = AnalogVideoSensorConfiguration(1234,5);
    sensor4 = AnalogVideoSensorConfiguration(4321,7);
    src2->AddSensor(sensor3);
    src2->AddSensor(sensor4);
    TestMessage(src2, dest2);
    system ("pause");

    //Test Report Analog Video Sensor Capabilities
    std::cout << "****Test for Report Analog Video Sensor Capabilities****" << std::endl;
    ReportAnalogVideoSensorCapabilities* src3 = new ReportAnalogVideoSensorCapabilities(Address((UInt)1234),Address((UInt)4321));
    Message* dest3 = new ReportAnalogVideoSensorCapabilities(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message; this tests the AddSensor method
    sensor1 = AnalogVideoSensorCapabilities(1234,5);
    sensor2 = AnalogVideoSensorCapabilities(4321,7);
    src3->AddSensor(sensor1);
    src3->AddSensor(sensor2);
    TestMessage(src3, dest3);
    system ("pause");

    //Test Report Analog Video Sensor Configurations
    std::cout << "****Test for Report Analog Video Sensor Configurations****" << std::endl;
    ReportAnalogVideoSensorConfigurations* src4 = new ReportAnalogVideoSensorConfigurations(Address((UInt)1234),Address((UInt)4321));
    Message* dest4 = new ReportAnalogVideoSensorConfigurations(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message; this tests the AddSensor method
    sensor3 = AnalogVideoSensorConfiguration(1234,5);
    sensor4 = AnalogVideoSensorConfiguration(4321,7);
    src4->AddSensor(sensor3);
    src4->AddSensor(sensor4);
    TestMessage(src4, dest4);
    system ("pause");

    //Test Report Analog Video Sensor Configurations
    std::cout << "****Set Analog Video Sensor Configurations****" << std::endl;
    SetAnalogVideoSensorConfigurations* src5 = new SetAnalogVideoSensorConfigurations(Address((UInt)1234),Address((UInt)4321));
    Message* dest5 = new SetAnalogVideoSensorConfigurations(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message; this tests the AddSensor method
    sensor3 = AnalogVideoSensorConfiguration(1234,5);
    sensor4 = AnalogVideoSensorConfiguration(4321,7);
    src5->AddSensor(sensor3);
    src5->AddSensor(sensor4);
    TestMessage(src5, dest5);
    system ("pause");

}

void TestDigitalVideoMessages()
{
    DigitalVideoSensor sensor1;
    DigitalVideoSensorCapabilities sensor2, sensor3;
    DigitalVideoSensorConfiguration sensor4, sensor5;

    //Test Control Digital Video Sensor
    std::cout << "****Control Digital Video****" << std::endl;
    Message* src = new ControlDigitalVideoSensor(Address((UInt)1234),Address((UInt)4321));
    Message* dest = new ControlDigitalVideoSensor(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message;
    sensor1 = DigitalVideoSensor(1234,0);
    ((ControlDigitalVideoSensor*)src)->SetSensor(sensor1);
    TestMessage(src, dest);
    system ("pause");

    //Test Query Digital Video Sensor Capabilities
    std::cout << "****Query Digital Video Sensor Capabilities****" << std::endl;
    src = new QueryDigitalVideoSensorCapabilities(Address((UInt)1234),Address((UInt)4321));
    dest = new QueryDigitalVideoSensorCapabilities(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message;
    sensor2 = DigitalVideoSensorCapabilities(0x0,2);
    sensor3 = DigitalVideoSensorCapabilities(0x0,3);
    ((QueryDigitalVideoSensorCapabilities*)src)->AddSensor(sensor2);
    ((QueryDigitalVideoSensorCapabilities*)src)->AddSensor(sensor3);
    TestMessage(src, dest);
    system ("pause");

    //Test Query Digital Video Sensor Configurations
    std::cout << "****Query Digital Video Sensor Configurations****" << std::endl;
    src = new QueryDigitalVideoSensorConfigurations(Address((UInt)1234),Address((UInt)4321));
    dest = new QueryDigitalVideoSensorConfigurations(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message;
    sensor4 = DigitalVideoSensorConfiguration(0x0,2);
    sensor5 = DigitalVideoSensorConfiguration(0x0,3);
    ((QueryDigitalVideoSensorConfigurations*)src)->AddSensor(sensor4);
    ((QueryDigitalVideoSensorConfigurations*)src)->AddSensor(sensor5);
    TestMessage(src, dest);
    system ("pause");

    //Test Report Digital Video Sensor Capabilities
    std::cout << "****Report Digital Video Sensor Capabilities****" << std::endl;
    src = new ReportDigitalVideoSensorCapabilities(Address((UInt)1234),Address((UInt)4321));
    dest = new ReportDigitalVideoSensorCapabilities(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message;
    sensor2 = DigitalVideoSensorCapabilities(0x0,2);
    sensor3 = DigitalVideoSensorCapabilities(0x0,3);
    ((ReportDigitalVideoSensorCapabilities*)src)->AddSensor(sensor2);
    ((ReportDigitalVideoSensorCapabilities*)src)->AddSensor(sensor3);
    TestMessage(src, dest);
    system ("pause");

    //Test Report Digital Video Sensor Configurations
    std::cout << "****Report Digital Video Sensor Configurations****" << std::endl;
    src = new ReportDigitalVideoSensorConfigurations(Address((UInt)1234),Address((UInt)4321));
    dest = new ReportDigitalVideoSensorConfigurations(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message;
    sensor4 = DigitalVideoSensorConfiguration(0x0,2);
    sensor5 = DigitalVideoSensorConfiguration(0x0,3);
    ((ReportDigitalVideoSensorConfigurations*)src)->AddSensor(sensor4);
    ((ReportDigitalVideoSensorConfigurations*)src)->AddSensor(sensor5);
    TestMessage(src, dest);
    system ("pause");

    //Test Set Digital Video Sensor Configurations
    std::cout << "****Set Digital Video Sensor Configurations****" << std::endl;
    src = new SetDigitalVideoSensorConfigurations(Address((UInt)1234),Address((UInt)4321));
    dest = new SetDigitalVideoSensorConfigurations(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message;
    sensor4 = DigitalVideoSensorConfiguration(0x0,2);
    sensor5 = DigitalVideoSensorConfiguration(0x0,3);
    ((SetDigitalVideoSensorConfigurations*)src)->AddSensor(sensor4);
    ((SetDigitalVideoSensorConfigurations*)src)->AddSensor(sensor5);
    TestMessage(src, dest);
    system ("pause");
}

void TestGeometricMessages()
{
    GeometricProperties prop1, prop4;
    ManipulatorGeometricProperties prop2, prop5;
    StaticGeometricProperties prop3, prop6;

    //Test Query SensorGeometric Properties
    std::cout << "****Query SensorGeometric Properties****" << std::endl;
    Message* src = new QuerySensorGeometricProperties(Address((UInt)1234),Address((UInt)4321));
    Message* dest = new QuerySensorGeometricProperties(Address((UInt)5678),Address((UInt)8765));
    prop1 = GeometricProperties(1);
    prop4 = GeometricProperties(4);
    ((QuerySensorGeometricProperties*)src)->AddSensor(prop1);
    ((QuerySensorGeometricProperties*)src)->AddSensor(prop4);
    TestMessage(src, dest);
    system ("pause");

    //Test Report Sensor Geometric Properties
    std::cout << "****Report SensorGeometric Properties****" << std::endl;
    src = new ReportSensorGeometricProperties(Address((UInt)1234),Address((UInt)4321));
    dest = new ReportSensorGeometricProperties(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message;
    prop1 = GeometricProperties(1234,1);
    prop2 = ManipulatorGeometricProperties(12,1,1,1,12,34,45,123,456,789,321);
    prop3 = StaticGeometricProperties(21,43,54,321,654,987,123);
    prop1.SetManipulatorGeometricProperties(prop2);
    prop1.SetStaticGeometricProperties(prop3);
    prop4 = GeometricProperties(1234,2);
    prop5 = ManipulatorGeometricProperties(12,1,1,1,12,34,45,123,456,789,321);
    prop6 = StaticGeometricProperties(21,43,54,321,654,987,123);
    prop4.SetManipulatorGeometricProperties(prop5);
    prop4.SetStaticGeometricProperties(prop6);
    ((ReportSensorGeometricProperties*)src)->AddSensor(prop1);
    ((ReportSensorGeometricProperties*)src)->AddSensor(prop4);
    TestMessage(src, dest);
    system ("pause");
}

void TestRangeMessages()
{
    RangeSensorCapabilities rangeCapabilities1;
    RangeSensorCapabilities rangeCapabilities2;
    RangeSensorConfiguration rangeConfiguration1;
    RangeSensorConfiguration rangeConfiguration2;

    //Test Query Range Sensor Capabilities
    std::cout << "****Query Range Sensor Capabilities****" << std::endl;
    Message* src = new QueryRangeSensorCapabilities(Address((UInt)1234),Address((UInt)4321));
    Message* dest = new QueryRangeSensorCapabilities(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message;
    rangeCapabilities1 = RangeSensorCapabilities(0x7FF,123, "Test Sensor #1");
    rangeCapabilities2 = RangeSensorCapabilities(0x0,456, "Test Sensor #2");
    ((QueryRangeSensorCapabilities*)src)->AddSensor(rangeCapabilities1);
    ((QueryRangeSensorCapabilities*)src)->AddSensor(rangeCapabilities2);
    TestMessage(src, dest);
    system ("pause");

    //Test Query Range Sensor Configurations
    std::cout << "****Query Range Sensor Configurations****" << std::endl;
    src = new QueryRangeSensorConfigurations(Address((UInt)1234),Address((UInt)4321));
    dest = new QueryRangeSensorConfigurations(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message;
    rangeConfiguration1 = RangeSensorConfiguration(0x7FF,123);
    rangeConfiguration1.SetHorizontalFieldOfViewStartAngle(12);
    rangeConfiguration2 = RangeSensorConfiguration(0x0,456);
    rangeConfiguration2.SetHorizontalFieldOfViewStartAngle(12);
    ((QueryRangeSensorConfigurations*)src)->AddSensor(rangeConfiguration1);
    ((QueryRangeSensorConfigurations*)src)->AddSensor(rangeConfiguration2);
    TestMessage(src, dest);
    system ("pause");

    //Test Report Range Sensor Capabilities
    std::cout << "****Report Range Sensor Capabilities****" << std::endl;
    src = new ReportRangeSensorCapabilities(Address((UInt)1234),Address((UInt)4321));
    dest = new ReportRangeSensorCapabilities(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message;
    rangeCapabilities1 = RangeSensorCapabilities(0x7FF,123, "Test Sensor #1");
    rangeCapabilities1.SetMaximumRange(30);
    rangeCapabilities2 = RangeSensorCapabilities(0x0,456, "Test Sensor #2");
    rangeCapabilities2.SetMaximumRange(30);
    ((ReportRangeSensorCapabilities*)src)->AddSensor(rangeCapabilities1);
    ((ReportRangeSensorCapabilities*)src)->AddSensor(rangeCapabilities2);
    TestMessage(src, dest);
    system ("pause");

    //Test Report Range Sensor Configurations
    std::cout << "****Report Range Sensor Configurations****" << std::endl;
    src = new ReportRangeSensorConfigurations(Address((UInt)1234),Address((UInt)4321));
    dest = new ReportRangeSensorConfigurations(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message;
    rangeConfiguration1 = RangeSensorConfiguration(0x7FF,123);
    rangeConfiguration1.SetHorizontalFieldOfViewStartAngle(12);
    rangeConfiguration2 = RangeSensorConfiguration(0x0,456);
    rangeConfiguration2.SetHorizontalFieldOfViewStartAngle(12);
    ((ReportRangeSensorConfigurations*)src)->AddSensor(rangeConfiguration1);
    ((ReportRangeSensorConfigurations*)src)->AddSensor(rangeConfiguration2);
    TestMessage(src, dest);
    system ("pause");

    //Test Set Range Sensor Configurations
    std::cout << "****Set Range Sensor Configurations****" << std::endl;
    src = new SetRangeSensorConfigurations(Address((UInt)1234),Address((UInt)4321));
    dest = new SetRangeSensorConfigurations(Address((UInt)5678),Address((UInt)8765));
    ((SetRangeSensorConfigurations*)src)->SetRequestId((Byte) 5);
    //Add sensors to the source message;
    rangeConfiguration1 = RangeSensorConfiguration(0x7FF,123);
    rangeConfiguration1.SetHorizontalFieldOfViewStartAngle(12);
    rangeConfiguration2 = RangeSensorConfiguration(0x0,456);
    rangeConfiguration2.SetHorizontalFieldOfViewStartAngle(12);
    ((SetRangeSensorConfigurations*)src)->AddSensor(rangeConfiguration1);
    ((SetRangeSensorConfigurations*)src)->AddSensor(rangeConfiguration2);
    TestMessage(src, dest);
    system ("pause");
}

void TestStillImageMessages()
{
    StillImageData data1, data2;
    StillImageSensorCapabilities capabilities1, capabilities2; 
    StillImageSensorConfiguration configuration1, configuration2;

    //Test Query Still Image Data
    std::cout << "****Test for Query Still Image Data****" << std::endl;
    Message* src = new QueryStillImageData(Address((UInt)1234),Address((UInt)4321));
    Message* dest = new QueryStillImageData(Address((UInt)5678),Address((UInt)8765));
    data1 = StillImageData(0x1,123,0);
    data1.SetTimeStamp(123456);
    Packet p = Packet();
    p.Write((Byte) 1);
    p.Write((Byte) 2);
    p.Write((Byte) 3);
    data1.SetImageFrameData(p,StillImageData::JPEG);
    data2 = StillImageData(0,456,1);
    ((QueryStillImageData*)src)->AddSensor(data1);
    ((QueryStillImageData*)src)->AddSensor(data2);
    TestMessage(src, dest);
    system ("pause");

    //Test Query Still Image Capabilities
    std::cout << "****Test for Query Still Image Capabilities****" << std::endl;
    src = new QueryStillImageSensorCapabilities(Address((UInt)1234),Address((UInt)4321));
    dest = new QueryStillImageSensorCapabilities(Address((UInt)5678),Address((UInt)8765));
    capabilities1 = StillImageSensorCapabilities(0x3,123);
    capabilities1.SetSupportedFrameSizes(StillImageSensorCapabilities::SupportedFrameSizesMask::cga_320x200 | StillImageSensorCapabilities::SupportedFrameSizesMask::ega_640x350);
    capabilities1.SetSupportedImageFormats(StillImageSensorCapabilities::SupportedImageFormatsMask::BMP | StillImageSensorCapabilities::SupportedImageFormatsMask::JPEG);
    capabilities2 = StillImageSensorCapabilities(0,456);
    ((QueryStillImageSensorCapabilities*)src)->AddSensor(capabilities1);
    ((QueryStillImageSensorCapabilities*)src)->AddSensor(capabilities2);
    TestMessage(src, dest);
    system ("pause");

    //Test Query Still Image Configurations
    std::cout << "****Test for Query Still Image Configurations****" << std::endl;
    src = new QueryStillImageSensorConfigurations(Address((UInt)1234),Address((UInt)4321));
    dest = new QueryStillImageSensorConfigurations(Address((UInt)5678),Address((UInt)8765));
    configuration1 = StillImageSensorConfiguration(0x3,123);
    configuration2 = StillImageSensorConfiguration(0,456);
    ((QueryStillImageSensorConfigurations*)src)->AddSensor(configuration1);
    ((QueryStillImageSensorConfigurations*)src)->AddSensor(configuration2);
    TestMessage(src, dest);
    system ("pause");

    //Test Report Still Image Data
    std::cout << "****Test Report Still Image Data****" << std::endl;
    src = new ReportStillImageData(Address((UInt)1234),Address((UInt)4321));
    dest = new ReportStillImageData(Address((UInt)5678),Address((UInt)8765));
    data1 = StillImageData(0x1,123,0);
    data1.SetTimeStamp(123456);
    p = Packet();
    p.Write((Byte) 1);
    p.Write((Byte) 2);
    p.Write((Byte) 3);
    data1.SetImageFrameData(p,StillImageData::JPEG);
    data2 = StillImageData(0,456,1);
    ((ReportStillImageData*)src)->AddSensor(data1);
    ((ReportStillImageData*)src)->AddSensor(data2);
    TestMessage(src, dest);
    system ("pause");

    //Test Report Still Image Capabilities
    std::cout << "****Test Report Still Image Capabilities****" << std::endl;
    src = new ReportStillImageSensorCapabilities(Address((UInt)1234),Address((UInt)4321));
    dest = new ReportStillImageSensorCapabilities(Address((UInt)5678),Address((UInt)8765));
    capabilities1 = StillImageSensorCapabilities(0x3,123);
    capabilities1.SetSupportedFrameSizes(StillImageSensorCapabilities::SupportedFrameSizesMask::cga_320x200 | StillImageSensorCapabilities::SupportedFrameSizesMask::ega_640x350);
    capabilities1.SetSupportedImageFormats(StillImageSensorCapabilities::SupportedImageFormatsMask::BMP | StillImageSensorCapabilities::SupportedImageFormatsMask::JPEG);
    capabilities2 = StillImageSensorCapabilities(0,456);
    ((ReportStillImageSensorCapabilities*)src)->AddSensor(capabilities1);
    ((ReportStillImageSensorCapabilities*)src)->AddSensor(capabilities2);
    TestMessage(src, dest);
    system ("pause");

    //Test Report Still Image Configurations
    std::cout << "****Test Report Still Image Configurations****" << std::endl;
    src = new ReportStillImageSensorConfigurations(Address((UInt)1234),Address((UInt)4321));
    dest = new ReportStillImageSensorConfigurations(Address((UInt)5678),Address((UInt)8765));
    configuration1 = StillImageSensorConfiguration(0x3,123);
    configuration1.SetFrameSize(StillImageSensorConfiguration::wsxga_1600x1024);
    configuration1.SetImageFormat(StillImageSensorConfiguration::PNG);
    configuration2 = StillImageSensorConfiguration(0,456);
    ((ReportStillImageSensorConfigurations*)src)->AddSensor(configuration1);
    ((ReportStillImageSensorConfigurations*)src)->AddSensor(configuration2);
    TestMessage(src, dest);
    system ("pause");

    //Test Set Still Image Configurations
    std::cout << "****Test Set Still Image Configurations****" << std::endl;
    src = new SetStillImageSensorConfigurations(Address((UInt)1234),Address((UInt)4321));
    dest = new SetStillImageSensorConfigurations(Address((UInt)5678),Address((UInt)8765));
    configuration1 = StillImageSensorConfiguration(0x3,123);
    configuration1.SetFrameSize(StillImageSensorConfiguration::wsxga_1600x1024);
    configuration1.SetImageFormat(StillImageSensorConfiguration::PNG);
    configuration2 = StillImageSensorConfiguration(0,456);
    ((SetStillImageSensorConfigurations*)src)->AddSensor(configuration1);
    ((SetStillImageSensorConfigurations*)src)->AddSensor(configuration2);
    TestMessage(src, dest);
    system ("pause");
}

void TestVisualMessages()
{
    VisualSensorCapabilities visualCapabilities1;
    VisualSensorCapabilities visualCapabilities2;
    VisualSensorConfiguration visualConfiguration1;
    VisualSensorConfiguration visualConfiguration2;

    //Test Query Visual Sensor Capabilities
    std::cout << "****Query Visual Sensor Capabilities****" << std::endl;
    Message* src = new QueryVisualSensorCapabilities(Address((UInt)1234),Address((UInt)4321));
    Message* dest = new QueryVisualSensorCapabilities(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message;
    visualCapabilities1 = VisualSensorCapabilities(0x7FFF,123, "Test Sensor #1");
    visualCapabilities2 = VisualSensorCapabilities(0x0,456, "Test Sensor #2");
    ((QueryVisualSensorCapabilities*)src)->AddSensor(visualCapabilities1);
    ((QueryVisualSensorCapabilities*)src)->AddSensor(visualCapabilities2);
    TestMessage(src, dest);
    system ("pause");

    //Test Query Visual Sensor Configurations
    std::cout << "****Query Visual Sensor Configurations****" << std::endl;
    src = new QueryVisualSensorConfigurations(Address((UInt)1234),Address((UInt)4321));
    dest = new QueryVisualSensorConfigurations(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message;
    visualConfiguration1 = VisualSensorConfiguration(0x7FFF,123);
    visualConfiguration2 = VisualSensorConfiguration(0x0,456);
    ((QueryVisualSensorConfigurations*)src)->AddSensor(visualConfiguration1);
    ((QueryVisualSensorConfigurations*)src)->AddSensor(visualConfiguration2);
    TestMessage(src, dest);
    system ("pause");

    //Test Report Visual Sensor Capabilities
    std::cout << "****Report Visual Sensor Capabilities****" << std::endl;
    src = new ReportVisualSensorCapabilities(Address((UInt)1234),Address((UInt)4321));
    dest = new ReportVisualSensorCapabilities(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message;
    visualCapabilities1 = VisualSensorCapabilities(0x7FFF,123, "Test Sensor #1");
    visualCapabilities1.SetExposureModes(VisualSensorCapabilities::ExposureModesMask::Auti | VisualSensorCapabilities::ExposureModesMask::Manual);
    visualCapabilities2 = VisualSensorCapabilities(0x0,456, "Test Sensor #2");
    ((ReportVisualSensorCapabilities*)src)->AddSensor(visualCapabilities1);
    ((ReportVisualSensorCapabilities*)src)->AddSensor(visualCapabilities2);
    TestMessage(src, dest);
    system ("pause");

    //Test Report Visual Sensor Configurations
    std::cout << "****Report Visual Sensor Configurations****" << std::endl;
    src = new ReportVisualSensorConfigurations(Address((UInt)1234),Address((UInt)4321));
    dest = new ReportVisualSensorConfigurations(Address((UInt)5678),Address((UInt)8765));
    //Add sensors to the source message;
    visualConfiguration1 = VisualSensorConfiguration(0x7FFF,123);
    visualConfiguration1.SetExposureMode(VisualSensorConfiguration::ShutterPriority);
    visualConfiguration2 = VisualSensorConfiguration(0x0,456);
    ((QueryVisualSensorConfigurations*)src)->AddSensor(visualConfiguration1);
    ((QueryVisualSensorConfigurations*)src)->AddSensor(visualConfiguration2);
    TestMessage(src, dest);
    system ("pause");

    //Test Set Visual Sensor Configurations
    std::cout << "****Set Visual Sensor Configurations****" << std::endl;
    src = new SetVisualSensorConfigurations(Address((UInt)1234),Address((UInt)4321));
    dest = new SetVisualSensorConfigurations(Address((UInt)5678),Address((UInt)8765));
    ((SetVisualSensorConfigurations*)src)->SetRequestId((Byte) 5);
    ((SetVisualSensorConfigurations*)dest)->SetRequestId((Byte) 9);
    //Add sensors to the source message;
    visualConfiguration1 = VisualSensorConfiguration(0x7FFF,123);
    visualConfiguration1.SetExposureMode(VisualSensorConfiguration::ShutterPriority);
    visualConfiguration2 = VisualSensorConfiguration(0x0,456);
    ((SetVisualSensorConfigurations*)src)->AddSensor(visualConfiguration1);
    ((SetVisualSensorConfigurations*)src)->AddSensor(visualConfiguration2);
    TestMessage(src, dest);
    system ("pause");
}

/* End of File */
