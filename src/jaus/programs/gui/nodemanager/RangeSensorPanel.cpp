////////////////////////////////////////////////////////////////////////////////////
///
///  \file RangeSensorPanel.cpp
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
#include "RangeSensorPanel.h"
#include "jaus/environment/range/RangeSensor.h"
#include <wx/msgdlg.h>
#include <sstream>
#include <iomanip>


inline wxString ConvertString(const std::string& str) { return wxString(str.c_str(), wxConvUTF8); }

DECLARE_EVENT_TYPE(wxEVT_NEW_IMAGE, wxCommandEvent)
DEFINE_EVENT_TYPE(wxEVT_NEW_IMAGE);

RangeSensorPanel::RangeSensorPanel( JAUS::Component* jausComponent, wxWindow* parent )
:
RangeSensorPanelBase( parent ), MonitorPanel(jausComponent)
{
    this->Connect(wxEVT_NEW_IMAGE, wxCommandEventHandler(RangeSensorPanel::OnNewImage), NULL, this);
    mScanNumber = 0;
    mpImagePanel = new wxImagePanel(this,
                               wxID_ANY,
                               wxDefaultPosition,
                               wxDefaultSize,
                               wxSIMPLE_BORDER|wxTAB_TRAVERSAL);
    mpImageSizer->Add( mpImagePanel, 1, wxEXPAND | wxALL, 5 );
    mpRangeSubscriber = new JAUS::RangeSubscriber();
    mpRangeSubscriber->RegisterCallback(this, true);
}


void RangeSensorPanel::AddServices(JAUS::Component* jausComponent)
{
    jausComponent->AddService(mpRangeSubscriber);
}


void RangeSensorPanel::ProcessCartesianRangeScan(const JAUS::Point3D::List& scan, 
                                                 const JAUS::Address& sourceID, 
                                                 const JAUS::UShort sensorID,
                                                 const JAUS::Time& timestamp)
{
    mDataMutex.Lock();
    mScans[sourceID] = scan;
    mScanNumber++;
    mDataMutex.Unlock();

    // Add event to main GUI thread to draw picture on screen.
    ::wxCommandEvent customEvent(wxEVT_NEW_IMAGE, this->GetId());
    customEvent.SetEventObject(this);
    this->AddPendingEvent(customEvent);
}


void RangeSensorPanel::OnNewImage( wxCommandEvent& event )
{
    std::map<JAUS::UShort, JAUS::Point3D::List> scans;
    int scanNumber;
    mDataMutex.Lock();
    scans = mScans;
    scanNumber = mScanNumber;
    mDataMutex.Unlock();

    std::stringstream str;
    str << "Update: " << std::setw(4) << std::setfill('0') << scanNumber;
    this->mScanNumberLabel->SetLabel(ConvertString(str.str()));

    // Draw data to image.        
    int width = mpImagePanel->GetSize().GetWidth();
    int height = mpImagePanel->GetSize().GetHeight();
    if(width != mRangeImage.GetWidth() ||
       height != mRangeImage.GetHeight())
    {
        mRangeImage.Create(width, height, -1);
    }

    wxMemoryDC memoryDC;
    memoryDC.SelectObject(mRangeImage);

    // Set line color to green, fill color to green
    int penSize = 2;
    memoryDC.SetPen(wxPen(*wxGREEN, penSize, wxSOLID));
    memoryDC.SetBrush(wxBrush(*wxGREEN, wxSOLID));
    memoryDC.SetBackground(wxBrush(*wxBLACK, wxSOLID));
    memoryDC.Clear();
    double pixMeter = mZoomSlider->GetValue();

    int midY = (int)(height*mVerticalSlider->GetValue()/100.0);
    int midX = (int)(width*mHorizontalSlider->GetValue()/100.0);

    std::map<JAUS::UShort, JAUS::Point3D::List>::iterator scan;
    for(scan = scans.begin();
        scan != scans.end();
        scan++)
    {
        JAUS::Point3D::List::iterator point;
        for(point = scan->second.begin();
            point != scan->second.end();
            point++)
        {
            wxPoint drawPoint;

            int y, x;
            y = pixMeter*point->mX; // X is front.
            x = pixMeter*point->mY; // Y positive right.

            drawPoint.y = (int)(midY - y);
            drawPoint.x = (int)(midX + x);
            if(drawPoint.y >= penSize && drawPoint.y < mRangeImage.GetHeight() - penSize &&
                    drawPoint.x >= penSize && drawPoint.x < mRangeImage.GetWidth() - penSize)
            {
                // Now draw circle.
                memoryDC.DrawCircle(drawPoint, penSize);
            }
        }
    }

    mpImagePanel->SetBitmap(mRangeImage);
}

void RangeSensorPanel::OnUpdateUI( wxUpdateUIEvent& event )
{
    if(this->mConnectButton->GetLabel() == wxT("Connect"))
    {
        int width = mpImagePanel->GetSize().GetWidth();
        int height = mpImagePanel->GetSize().GetHeight();
        if(width != mRangeImage.GetWidth() ||
           height != mRangeImage.GetHeight())
        {
            mRangeImage.Create(width, height, -1);
            wxMemoryDC memoryDC;
            memoryDC.SelectObject(mRangeImage);
            // Set to black.
            memoryDC.SetBackground(wxBrush(*wxBLACK, wxSOLID));
            memoryDC.Clear();
            mpImagePanel->SetBitmap(mRangeImage);
        }
    }
}


void RangeSensorPanel::OnConnect( wxCommandEvent& event )
{
    int index = this->mComponentChoice->GetSelection();
    mScanNumber = 0;
    this->mScanNumberLabel->SetLabel(wxT("Update: 0000"));
    if(index >= 0)
    {
        wxString label = this->mComponentChoice->GetString(index);
        JAUS::Address id(JAUS::Address::FromString(std::string(label.ToAscii().data())));

        if(this->mComponentChoice->IsEnabled())
        {
            if(mpRangeSubscriber)
            {
                if(mpRangeSubscriber->CreateRangeSubscription(id, 0) == false)
                {
                    wxMessageBox(wxT("Failed to Connect to Range Sensor"), wxT("Connection Failed"));
                }
                else
                {
                    this->mComponentChoice->Disable();
                    this->mRefreshButton->Disable();
                    this->mConnectButton->SetLabel(wxT("Disconnect"));
                }
            }
        }
        else
        {
            mpRangeSubscriber->CancelRangeSubscription();
            this->mComponentChoice->Enable();
            this->mRefreshButton->Enable();
            this->mConnectButton->SetLabel(wxT("Connect"));

            wxMemoryDC memoryDC;
            memoryDC.SelectObject(mRangeImage);
            // Set to black.
            memoryDC.SetBackground(wxBrush(*wxBLACK, wxSOLID));
            memoryDC.Clear();
            mpImagePanel->SetBitmap(mRangeImage);

            mDataMutex.Lock();
            mScans.clear();
            mDataMutex.Unlock();
        }
    }
}


/** Called when component is shutdown. */
void RangeSensorPanel::OnShutdown()
{
    mScanNumber = 0;
    this->mScanNumberLabel->SetLabel(wxT("Update: 0000"));
    mpRangeSubscriber->CancelRangeSubscription();
    this->mComponentChoice->Clear();
    this->mComponentChoice->Enable();
    this->mRefreshButton->Enable();
    this->mConnectButton->SetLabel(wxT("Connect"));
    mDataMutex.Lock();
    mScans.clear();
    mDataMutex.Unlock();
}


void RangeSensorPanel::OnRefreshList( wxCommandEvent& event )
{
    this->mComponentChoice->Clear();
    if(mpComponent->IsInitialized())
    {
        JAUS::Address::List components = 
            mpComponent->DiscoveryService()->GetComponentsWithService(JAUS::RangeSensor::Name);
        JAUS::Address::List::iterator component;
        for(component = components.begin();
            component != components.end();
            component++)
        {
            this->mComponentChoice->Append(ConvertString(component->ToString()));
        }
    }

    if(this->mComponentChoice->GetSelection() < 0 && this->mComponentChoice->GetCount() > 0)
    {
        this->mComponentChoice->SetSelection(0);
    }
}

