///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Jun 30 2011)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __NODEMANAGERGUI_H__
#define __NODEMANAGERGUI_H__

#include <wx/artprov.h>
#include <wx/xrc/xmlres.h>
#include <wx/string.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/menu.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/notebook.h>
#include <wx/sizer.h>
#include <wx/statusbr.h>
#include <wx/frame.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/combobox.h>
#include <wx/button.h>
#include <wx/listbox.h>
#include <wx/choice.h>
#include <wx/treectrl.h>
#include <wx/panel.h>
#include <wx/statline.h>
#include <wx/slider.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class NodeManagerFrameBase
///////////////////////////////////////////////////////////////////////////////
class NodeManagerFrameBase : public wxFrame 
{
	private:
	
	protected:
		wxMenuBar* mMenuBar;
		wxMenu* mMainMenu;
		wxNotebook* mNotebook;
		wxStatusBar* mStatusBar;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnUpdateUI( wxUpdateUIEvent& event ) { event.Skip(); }
		virtual void OnExit( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		
		NodeManagerFrameBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("JAUS++ Node Manager"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 940,517 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
		
		~NodeManagerFrameBase();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class DiscoveryPanelBase
///////////////////////////////////////////////////////////////////////////////
class DiscoveryPanelBase : public wxPanel 
{
	private:
	
	protected:
		wxStaticText* m_staticText11;
		wxTextCtrl* mSubsystemTextCtrl;
		wxComboBox* mNodeChoiceBox;
		wxComboBox* mComponentChoiceBox;
		wxButton* mInitializeButton;
		wxButton* mShutdownButton;
		wxStaticText* m_staticText12;
		wxListBox* mSubsystemList;
		wxStaticText* m_staticText10;
		wxChoice* mTTL;
		wxNotebook* mDiscoveryNotebook;
		wxPanel* mServicesPanel;
		wxTreeCtrl* mSubsystemTree;
		wxStaticText* m_staticText18;
		wxChoice* mSelectComponentChoice;
		wxStaticText* m_staticText19;
		wxStaticLine* m_staticline1;
		wxStaticText* m_staticText17;
		wxChoice* mAuthorityChoice;
		wxButton* mControlButton;
		wxButton* mJoystickButton;
		wxStaticLine* m_staticline3;
		wxButton* mResumeButton;
		wxButton* mStandbyButton;
		wxButton* mSendShutdown;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnUpdateUI( wxUpdateUIEvent& event ) { event.Skip(); }
		virtual void OnInitializeButton( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnShutdownButton( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSelectSubsystem( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnTLL( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnNotebookPageChanged( wxNotebookEvent& event ) { event.Skip(); }
		virtual void OnSelectComponentChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnAuthorityLevel( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnTakeControl( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnTakeJoystickControl( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSendResume( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSendStandby( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSendShutdown( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		
		DiscoveryPanelBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 909,466 ), long style = wxTAB_TRAVERSAL ); 
		~DiscoveryPanelBase();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class RangeSensorPanelBase
///////////////////////////////////////////////////////////////////////////////
class RangeSensorPanelBase : public wxPanel 
{
	private:
	
	protected:
		wxSlider* mHorizontalSlider;
		wxStaticText* mScanNumberLabel;
		wxBoxSizer* mpImageSizer;
		wxSlider* mVerticalSlider;
		wxButton* mRefreshButton;
		wxStaticText* m_staticText21;
		wxChoice* mComponentChoice;
		wxButton* mConnectButton;
		wxStaticText* m_staticText20;
		wxSlider* mZoomSlider;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnSetFocus( wxFocusEvent& event ) { event.Skip(); }
		virtual void OnUpdateUI( wxUpdateUIEvent& event ) { event.Skip(); }
		virtual void OnRefreshList( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnConnect( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnZoomScroll( wxScrollEvent& event ) { event.Skip(); }
		
	
	public:
		
		RangeSensorPanelBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 724,518 ), long style = wxTAB_TRAVERSAL ); 
		~RangeSensorPanelBase();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class LocalPoseSensorPanelBase
///////////////////////////////////////////////////////////////////////////////
class LocalPoseSensorPanelBase : public wxPanel 
{
	private:
	
	protected:
		wxStaticText* mScanNumberLabel;
		wxBoxSizer* mpImageSizer;
		wxButton* mRefreshButton;
		wxStaticText* m_staticText21;
		wxChoice* mComponentChoice;
		wxButton* mConnectButton;
		wxStaticText* m_staticText20;
		wxSlider* mZoomSlider;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnSetFocus( wxFocusEvent& event ) { event.Skip(); }
		virtual void OnUpdateUI( wxUpdateUIEvent& event ) { event.Skip(); }
		virtual void OnRefreshList( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnConnect( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnZoomScroll( wxScrollEvent& event ) { event.Skip(); }
		
	
	public:
		
		LocalPoseSensorPanelBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 724,518 ), long style = wxTAB_TRAVERSAL ); 
		~LocalPoseSensorPanelBase();
	
};

#endif //__NODEMANAGERGUI_H__
