////////////////////////////////////////////////////////////////////////////////////
///
///  \file ExampleRangeSensor.cpp
///  \brief This file contains a test/example program for the RangeSensor services.
///
///  <br>Author(s): Daniel Barber
///  <br>Created: 10 April 2010
///  <br>Copyright (c) 2010
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
#include "jaus/core/Component.h"
#include "jaus/environment/range/RangeSensor.h"
#include "jaus/environment/range/RangeSubscriber.h"
#include <cxutils/Keyboard.h>
#include <cxutils/FileIO.h>
#include <cxutils/math/CxMath.h>
#include <iostream>
#include <iomanip>
#include <algorithm>

#ifdef VLD_ENABLED
#include <vld.h>
#endif


using namespace JAUS;

int main(int argc, char* argv[])
{
    Component component;
    // Add the services we want our component to have
    // beyond the core set.

    RangeSensor* rangeSensor = new RangeSensor();
    // Setup the configuration of a range sensing device.
    Point3D sensorLocation;
    Point3D sensorOrientation;
    UShort sensorID = 1;
    // Add to component.
    component.AddService(rangeSensor);

    // Try load settings files.
    // These files determine your UDP network 
    // settings, what Services to turn on/off 
    // or any Service specific settings. See the
    // example file for settings file format.
    component.LoadSettings("settings/jaus/services.xml");

    // In this example, we are simulating a component on part of a simulated
    // robot.
    component.DiscoveryService()->SetSubsystemIdentification(Subsystem::Vehicle,
                                                             "Simulation");

    // Initialize component component with component given ID.
    if(component.InitializeWithUniqueID() == false)
    {
        std::cout << "Failed to Initialize Component.\n";
        return 0;
    }

    std::cout << "Component Initialized!\n";

    Time::Stamp startTimeMs = Time::GetUtcTimeMs();
    double t = 0.0;
    double scanAngle = 270;
    double bearingIncrement = 0.25;
    while(CxUtils::GetChar() != 27)
    {      
        // Simulate fake range sensor data.
        Point3D::List scan;
        for(double bearing = -scanAngle/2.0;
            bearing <= scanAngle/2.0;
            bearing += bearingIncrement)
        {

            //rangeScan.push_back(fabs(sin(t++)*rangeSensorConfig.mMaxRange*1000.0));
            double range = 6.0;
            if(bearing < -scanAngle/4)
            {
                range = 1.5;
            }
            else if(bearing < -scanAngle/2)
            {
                range = 2.0;
            }
            else if(bearing <= 0)
            {
                range = 2.3;
            }
            if(bearing < scanAngle/2)
            {
                range = 2.1;
            }
            else if(bearing < scanAngle/4)
            {
                range = 0.5;
            }
            else
            {
                range = 1.0;
            }
            scan.push_back(Point3D(range, 0, CxUtils::CxToRadians(bearing)));
        }
        // Pass list of spherical coordinates along with location of
        // sensor on vehicle for automatic coordinate conversion.
        rangeSensor->SetRangeScan(sensorID,
                                       sensorLocation,   // At origin of vehicle.
                                       sensorOrientation,   // Facing front with no roll or pitch change.
                                       scan);
        std::cout << Time::GetUtcTime().ToString() << std::endl;
        CxUtils::SleepMs(10); // Hz
    }

    // Shutdown the components.
    component.Shutdown();

    return 0;
}


/* End of File */
