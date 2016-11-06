////////////////////////////////////////////////////////////////////////////////////
///
///  \file RangeSensorPanel.h
///  \brief Panel for running a subsystem discovery component.
///
///  <br>Author(s): Daniel Barber
///  <br>Created: 2013
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
#ifndef _NODE_MANAGER_RANGE_SENSOR_PANEL__H
#define _NODE_MANAGER_RANGE_SENSOR_PANEL__H


#include "NodeManagerGUI.h"
#include "ImagePanel.h"
#include "MonitorPanel.h"
#include <wx/thread.h>
#include "jaus/core/Component.h"
#include "jaus/environment/range/RangeSensor.h"
#include "jaus/environment/range/RangeSubscriber.h"

////////////////////////////////////////////////////////////////////////////////
///
///   \class RangeSensorPanel
///   \brief Panel to subscribe to and view Range Sensor Data
///
////////////////////////////////////////////////////////////////////////////////
class RangeSensorPanel : public RangeSensorPanelBase,
                         public MonitorPanel,
                         public JAUS::RangeSubscriber::CartesianCallback
{
public:
    /** Constructor */
    RangeSensorPanel(JAUS::Component* jausComponent, wxWindow* parent );
    virtual void AddServices(JAUS::Component* jausComponent);
protected:
    virtual void ProcessCartesianRangeScan(const JAUS::Point3D::List& scan, 
                                           const JAUS::Address& sourceID, 
                                           const JAUS::UShort sensorID,
                                           const JAUS::Time& timestamp);
    virtual void OnNewImage( wxCommandEvent& event);
    virtual void OnUpdateUI( wxUpdateUIEvent& event );
    virtual void OnConnect( wxCommandEvent& event );
    virtual void OnRefreshList( wxCommandEvent& event );
    virtual void OnSetFocus( wxFocusEvent& event ) { event.Skip(); }
    virtual void RefreshPanel() {  }
    virtual void OnShutdown();
    wxMutex mImageMutex;              ///<  Image mutex.
    wxImagePanel* mpImagePanel;       ///<  Image panel.
    wxMutex mDataMutex;               ///<  Mutex for thread protection of data.
    std::map<JAUS::UShort, JAUS::Point3D::List> mScans; ///< Range Sensor Data.
    JAUS::RangeSubscriber* mpRangeSubscriber;           ///< JAUS Range Subscriber.
    wxBitmap mRangeImage;                               ///< Range image.
    volatile int mScanNumber;                           ///< Scan number.
};

#endif
