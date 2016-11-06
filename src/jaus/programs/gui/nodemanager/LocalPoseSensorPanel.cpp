////////////////////////////////////////////////////////////////////////////////////
///
///  \file LocalPoseSensorPanel.cpp
///  \brief Panel for collecting and displaying pose data from a JAUS vehicle.
///
///  <br>Author(s): David Adams
///  <br>Created: 2013
///  <br>Copyright (c) 2013
///  <br>Applied Cognition and Training in Immersive Virtual Environments
///  <br>(ACTIVE) Laboratory
///  <br>Institute for Simulation and Training (IST)
///  <br>University of Central Florida (UCF)
///  <br>All rights reserved.
///  <br>Email: dadams@ist.ucf.edu
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
#include "LocalPoseSensorPanel.h"
#include "cxutils/math/CxMath.h"

#include <iostream>
#include <iomanip>
#include <sstream>

inline wxString ConvertString(const std::string& str) { return wxString(str.c_str(), wxConvUTF8); }
DEFINE_EVENT_TYPE(wxEVT_NEW_IMAGE);



LocalPoseSensorPanel::LocalPoseSensorPanel(JAUS::Component* jausComponent, wxWindow* parent )
:
LocalPoseSensorPanelBase( parent ), MonitorPanel(jausComponent)
{
    this->Connect(wxEVT_NEW_IMAGE, wxCommandEventHandler(LocalPoseSensorPanel::OnNewImage), NULL, this);

    mpImagePanel = new wxImagePanel(this,
                               wxID_ANY,
                               wxDefaultPosition,
                               wxDefaultSize,
                               wxSIMPLE_BORDER|wxTAB_TRAVERSAL);
    mpImageSizer->Add( mpImagePanel, 1, wxEXPAND | wxALL, 5 );
    jausComponent->TransportService()->AddMessageTemplate(new JAUS::ReportGlobalPose());
    jausComponent->TransportService()->AddMessageTemplate(new JAUS::ReportLocalPose());
    jausComponent->TransportService()->AddMessageTemplate(new JAUS::QueryLocalPose());
    jausComponent->TransportService()->AddMessageTemplate(new JAUS::QueryGlobalPose());
    jausComponent->TransportService()->RegisterCallback(JAUS::REPORT_LOCAL_POSE,this);
    jausComponent->TransportService()->RegisterCallback(JAUS::REPORT_GLOBAL_POSE,this);
    mpReportLocalPose = NULL;
    mpReportGlobalPose = NULL;
    mConnectFlag = false;
    
    mConnectButton->Disable();
    

}


void LocalPoseSensorPanel::OnSetFocus( wxFocusEvent& event )
{
// TODO: Implement OnSetFocus
}


void LocalPoseSensorPanel::OnUpdateUI( wxUpdateUIEvent& event )
{

    ::wxCommandEvent customEvent(wxEVT_NEW_IMAGE, this->GetId());
    customEvent.SetEventObject(this);
    this->AddPendingEvent(customEvent);
}


void LocalPoseSensorPanel::OnRefreshList( wxCommandEvent& event )
{
    this->mComponentChoice->Clear();
    if(mpComponent->IsInitialized())
    {
        JAUS::Address::List components = 
            mpComponent->DiscoveryService()->GetComponentsWithService(JAUS::LocalPoseSensor::Name);
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
        mConnectButton->Enable();
    }
    else
    {
        mConnectButton->Disable();
    }
}


void LocalPoseSensorPanel::OnConnect( wxCommandEvent& event )
{
    int index = this->mComponentChoice->GetSelection();
    this->mScanNumberLabel->SetLabel(wxT("Local Pose Data:"));
    if(index >= 0)
    {
        wxString label = this->mComponentChoice->GetString(index);
        JAUS::Address id(JAUS::Address::FromString(std::string(label.ToAscii().data())));
        
        if(this->mComponentChoice->IsEnabled())
        {

            mDataMutex.Lock();
            mConnectFlag = true;
            JAUS::QueryLocalPose lp;
            lp.SetPresenceVector(lp.GetPresenceVectorMask());
            if(this->mpComponent->EventsService()->RequestPeriodicEvent(id, &lp, 20, 5))
            {
                mGPSToMonitorAddress = id;
                this->mComponentChoice->Disable();
                this->mRefreshButton->Disable();
                this->mConnectButton->SetLabel(wxT("Disconnect"));
                JAUS::QueryGlobalPose gp;
                gp.SetPresenceVector(gp.GetPresenceVectorMask());
                this->mpComponent->EventsService()->RequestPeriodicEvent(id, &gp, 20, 5);
            }
            else
            {
                this->mpComponent->EventsService()->CancelSubscription(id,JAUS::QUERY_LOCAL_POSE);
                this->mpComponent->EventsService()->CancelSubscription(id,JAUS::QUERY_GLOBAL_POSE);
                wxMessageBox(wxT("Unable To Get Subscription To Local Pose"), wxT("Subscription Failed"));
            }
            mDataMutex.Unlock();
            
            
        }
        else
        {
            mDataMutex.Lock();
            this->mpComponent->EventsService()->CancelSubscription(id,JAUS::QUERY_LOCAL_POSE);
            this->mpComponent->EventsService()->CancelSubscription(id,JAUS::QUERY_GLOBAL_POSE);
            this->mComponentChoice->Enable();
            this->mRefreshButton->Enable();
            this->mConnectButton->SetLabel(wxT("Connect"));
            
            mConnectFlag = false;
            mDataMutex.Unlock();
        }
    }
}


void LocalPoseSensorPanel::OnZoomScroll( wxScrollEvent& event )
{

}


void LocalPoseSensorPanel::OnShutdown()
{

    this->mComponentChoice->Clear();
    this->mComponentChoice->Enable();
    this->mRefreshButton->Enable();
    this->mConnectButton->SetLabel(wxT("Connect"));


}


void LocalPoseSensorPanel::OnNewImage( wxCommandEvent& event )
{
    int width = mpImagePanel->GetSize().GetWidth();
    int height = mpImagePanel->GetSize().GetHeight();
    if(width <= 0 || height <= 0) return;
    if(width != mLocalPoseImage.GetWidth() ||
       height != mLocalPoseImage.GetHeight())
    {
        mLocalPoseImage.Create(width, height, -1);
    }

    wxMemoryDC memoryDC;
    memoryDC.SelectObject(mLocalPoseImage);

    // Set line color to green, fill color to green
    int penSize = 3;
    int radius = 20;
    int lineLength = 40;
    memoryDC.SetPen(wxPen(*wxGREEN, penSize, wxSOLID));
    memoryDC.SetBrush(wxBrush(*wxGREEN, wxSOLID));
    memoryDC.SetBackground(wxBrush(*wxBLACK, wxSOLID));
    memoryDC.Clear();

    double zoomScale = mZoomSlider->GetValue()/50.0;
    
    double gridScale = 100;
    for(int i = width/2.0; i < width; i+=zoomScale*gridScale)
    {
        int offset = i - width/2.0;

        if(offset == 0)
        {
            memoryDC.SetPen(wxPen(*wxGREEN, penSize, wxSOLID));
            memoryDC.DrawLine(i,0,i,height); 
        }
        else
        {
            memoryDC.SetPen(wxPen(*wxGREEN, penSize, wxDOT));
            memoryDC.DrawLine(i,0,i,height); 
            memoryDC.DrawLine(width/2.0 - offset,0,width/2.0 - offset,height); 
        }   
    }
    for(int j = height/2.0; j < height; j+=zoomScale*gridScale)
    {
        int offset = j - height/2.0;

        if(offset == 0)
        {
            memoryDC.SetPen(wxPen(*wxGREEN, penSize, wxSOLID));
            memoryDC.DrawLine(0,height/2.0,width,height/2.0); 
        }
        else
        {
            memoryDC.SetPen(wxPen(*wxGREEN, penSize, wxDOT));
            memoryDC.DrawLine(0,j,width,j); 
            memoryDC.DrawLine(0,height/2.0 - offset, width,height/2.0 - offset); 
        }      
    }
    if(mConnectFlag)
    {
        mDataMutex.Lock();
        boost::circular_buffer<JAUS::ReportLocalPose>::iterator it;

        if(mPoseHistory.find(mGPSToMonitorAddress.mSubsystem) != mPoseHistory.end())
        {
            it = mPoseHistory[mGPSToMonitorAddress.mSubsystem].begin();
            double xpos,ypos,rotation=0;
            JAUS::ReportLocalPose currentpos = mPoseHistory[mGPSToMonitorAddress.mSubsystem].back();

            std::stringstream str;
            str << "X: "<< std::setprecision(2) << currentpos.GetX() 
                << "  Y: " << std::setprecision(2) << currentpos.GetY() 
                << "  Z: " << std::setprecision(2) << currentpos.GetZ() 
                << "  Yaw: " << std::setprecision(2) << CxUtils::CxToDegrees(currentpos.GetYaw());

            this->mScanNumberLabel->SetLabel(ConvertString(str.str()));
            
            //Draws the most recent pose object as green/red based on if data has been seen in the last
            //few seconds. Draws history as gray.
            for(it; it !=  mPoseHistory[mGPSToMonitorAddress.mSubsystem].end(); it++)
            {
                if(it == mPoseHistory[mGPSToMonitorAddress.mSubsystem].end()-1)
                {
                    if(CxUtils::Time::DifferenceInSeconds(CxUtils::Time(true),it->GetTimeStamp()) > 2)
                    {
                        memoryDC.SetPen(wxPen(*wxRED, penSize, wxSOLID));
                        memoryDC.SetBrush(wxBrush(*wxRED, wxSOLID));
                    }
                    else
                    {
                        memoryDC.SetPen(wxPen(wxColour(255, 255, 0), penSize, wxSOLID));
                        memoryDC.SetBrush(wxBrush(wxColour(255, 255, 0), wxSOLID));
                    }
                }
                else
                {
                    memoryDC.SetPen(wxPen(*wxLIGHT_GREY, penSize, wxSOLID));
                    memoryDC.SetBrush(wxBrush(*wxLIGHT_GREY, wxSOLID));
                }
                xpos = (it->GetY() *gridScale * zoomScale+width/2.0);
                ypos = (it->GetX() *gridScale * -zoomScale+height/2.0);
                rotation = it->GetYaw();
                CxUtils::Point3D line = CxUtils::Point3D(lineLength*zoomScale,0,0);
                // Add 90 so 0 is up.
                line = line.Rotate(CxUtils::Orientation::AddToAngle(rotation, -CxUtils::CX_HALF_PI),
                    CxUtils::Point3D::Z);
                
                memoryDC.DrawCircle(xpos,ypos,radius*zoomScale);
                memoryDC.DrawLine(xpos,ypos,line.mX+xpos, line.mY+ypos);
            }
            
        }
        mDataMutex.Unlock();
    }
    mpImagePanel->SetBitmap(mLocalPoseImage);
}


void LocalPoseSensorPanel::ProcessMessage(const JAUS::Message* message)
{
    mDataMutex.Lock();
    if(message->GetMessageCode() == JAUS::REPORT_GLOBAL_POSE)
    {
        if(mpReportGlobalPose)
        {
            delete(mpReportGlobalPose);
        }
        mpReportGlobalPose = dynamic_cast<JAUS::ReportGlobalPose*>( message->Clone());
        if(mGlobalPoseHistory.find(mpReportGlobalPose->GetSourceID().mSubsystem) == mGlobalPoseHistory.end())
        {
            mGlobalPoseHistory[mpReportGlobalPose->GetSourceID().mSubsystem] = boost::circular_buffer<JAUS::ReportGlobalPose>(15);           
        }
        mGlobalPoseHistory[mpReportGlobalPose->GetSourceID().mSubsystem].push_back(*mpReportGlobalPose);

    }
    if(message->GetMessageCode() == JAUS::REPORT_LOCAL_POSE)
    {
        if(mpReportLocalPose)
        {
            delete(mpReportLocalPose);
        }
        mpReportLocalPose = dynamic_cast<JAUS::ReportLocalPose*>( message->Clone());
        if(mPoseHistory.find(mpReportLocalPose->GetSourceID().mSubsystem) == mPoseHistory.end())
        {
            //Currently maintains history of 15 poses, change the number higher/lower for longer/shorter trail.
            mPoseHistory[mpReportLocalPose->GetSourceID().mSubsystem] = boost::circular_buffer<JAUS::ReportLocalPose>(15);           
        }
        mPoseHistory[mpReportLocalPose->GetSourceID().mSubsystem].push_back(*mpReportLocalPose);
        mLastPoseTime = CxUtils::Time(true);

    }
    mDataMutex.Unlock();
}

/** End of File */
