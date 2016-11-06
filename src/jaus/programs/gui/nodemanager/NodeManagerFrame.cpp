////////////////////////////////////////////////////////////////////////////////////
///
///  \file NodeManagerFrame.cpp
///  \brief Main file for the NodeManagerFrame code.
///
///  <br>Author(s): Daniel Barber
///  <br>Created: 12/2/2011
///  <br>Copyright (c) 2011
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
#include "NodeManagerFrame.h"
#include "DiscoveryPanel.h"
#include <cxutils/networking/Socket.h>
#include <wx/msgdlg.h>
#include <wx/filedlg.h>
#include <sstream>
#include <iomanip>

#ifdef WIN32
std::string MAIN_ICON = "icons/JAUS++Logo.ico";
wxBitmapType MAIN_ICON_TYPE = wxBITMAP_TYPE_ICO;
#else
std::string MAIN_ICON = "icons/JAUS++Logo.png";
wxBitmapType MAIN_ICON_TYPE = wxBITMAP_TYPE_PNG;
#endif


DECLARE_EVENT_TYPE(wxEVT_UPDATE_MAIN_PANEL, wxCommandEvent)
DEFINE_EVENT_TYPE(wxEVT_UPDATE_MAIN_PANEL)

/** Constructor, initializes node manager by default. */
NodeManagerFrame::NodeManagerFrame( wxWindow* parent )
    :
NodeManagerFrameBase( parent )
{
    // Add an icon/logo to the GUI.
    wxIcon icon;
    if(icon.LoadFile(wxString(MAIN_ICON.c_str(), wxConvUTF8), MAIN_ICON_TYPE))
    {
        SetIcon(icon);
    }

    DiscoveryPanel* discoveryPanel = new DiscoveryPanel(mNotebook);
    mpDiscoveryPanel  = discoveryPanel;
    this->mNotebook->AddPage(discoveryPanel, wxT("Manage"), false);
}

/** Called on UI Update. */
void NodeManagerFrame::OnUpdateUI( wxUpdateUIEvent& event )
{

}


/** Triggered on Exit event. */
void NodeManagerFrame::OnExit( wxCommandEvent& event )
{
    Close(true);
}


/*  End of File */
