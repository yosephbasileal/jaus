///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Jun 30 2011)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "NodeManagerGUI.h"

///////////////////////////////////////////////////////////////////////////

NodeManagerFrameBase::NodeManagerFrameBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	mMenuBar = new wxMenuBar( 0 );
	mMainMenu = new wxMenu();
	wxMenuItem* mExitItem;
	mExitItem = new wxMenuItem( mMainMenu, wxID_EXIT, wxString( wxT("Exit") ) , wxEmptyString, wxITEM_NORMAL );
	mMainMenu->Append( mExitItem );
	
	mMenuBar->Append( mMainMenu, wxT("File") ); 
	
	this->SetMenuBar( mMenuBar );
	
	wxBoxSizer* bSizer5;
	bSizer5 = new wxBoxSizer( wxVERTICAL );
	
	mNotebook = new wxNotebook( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0 );
	
	bSizer5->Add( mNotebook, 1, wxEXPAND | wxALL, 5 );
	
	this->SetSizer( bSizer5 );
	this->Layout();
	mStatusBar = this->CreateStatusBar( 1, wxST_SIZEGRIP, wxID_ANY );
	
	this->Centre( wxBOTH );
	
	// Connect Events
	this->Connect( wxEVT_UPDATE_UI, wxUpdateUIEventHandler( NodeManagerFrameBase::OnUpdateUI ) );
	this->Connect( mExitItem->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( NodeManagerFrameBase::OnExit ) );
}

NodeManagerFrameBase::~NodeManagerFrameBase()
{
	// Disconnect Events
	this->Disconnect( wxEVT_UPDATE_UI, wxUpdateUIEventHandler( NodeManagerFrameBase::OnUpdateUI ) );
	this->Disconnect( wxID_EXIT, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( NodeManagerFrameBase::OnExit ) );
	
}

DiscoveryPanelBase::DiscoveryPanelBase( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxFlexGridSizer* fgSizer1;
	fgSizer1 = new wxFlexGridSizer( 2, 1, 0, 0 );
	fgSizer1->AddGrowableCol( 0 );
	fgSizer1->AddGrowableRow( 1 );
	fgSizer1->SetFlexibleDirection( wxBOTH );
	fgSizer1->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	wxBoxSizer* bSizer101;
	bSizer101 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText11 = new wxStaticText( this, wxID_ANY, wxT("Component ID (S.N.C):"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText11->Wrap( -1 );
	m_staticText11->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer101->Add( m_staticText11, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	mSubsystemTextCtrl = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	bSizer101->Add( mSubsystemTextCtrl, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	mNodeChoiceBox = new wxComboBox( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, wxCB_READONLY ); 
	bSizer101->Add( mNodeChoiceBox, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	mComponentChoiceBox = new wxComboBox( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, wxCB_READONLY ); 
	bSizer101->Add( mComponentChoiceBox, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	mInitializeButton = new wxButton( this, wxID_ANY, wxT("Initialize"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer101->Add( mInitializeButton, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	mShutdownButton = new wxButton( this, wxID_ANY, wxT("Shutdown"), wxDefaultPosition, wxDefaultSize, 0 );
	mShutdownButton->Enable( false );
	
	bSizer101->Add( mShutdownButton, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	fgSizer1->Add( bSizer101, 0, 0, 5 );
	
	wxFlexGridSizer* fgSizer2;
	fgSizer2 = new wxFlexGridSizer( 0, 3, 0, 0 );
	fgSizer2->AddGrowableCol( 1 );
	fgSizer2->AddGrowableRow( 0 );
	fgSizer2->SetFlexibleDirection( wxBOTH );
	fgSizer2->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	wxBoxSizer* bSizer111;
	bSizer111 = new wxBoxSizer( wxVERTICAL );
	
	m_staticText12 = new wxStaticText( this, wxID_ANY, wxT("Subsystems (Select for Details)"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText12->Wrap( -1 );
	bSizer111->Add( m_staticText12, 0, wxALL|wxALIGN_CENTER_VERTICAL|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	mSubsystemList = new wxListBox( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0, NULL, wxLB_SORT ); 
	bSizer111->Add( mSubsystemList, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	wxBoxSizer* bSizer11;
	bSizer11 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText10 = new wxStaticText( this, wxID_ANY, wxT("TTL"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText10->Wrap( -1 );
	m_staticText10->SetToolTip( wxT("Time to Live (TLL) for components and subsystems.  If no updates heard for X seconds, component is dropped.") );
	
	bSizer11->Add( m_staticText10, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxString mTTLChoices[] = { wxT("1"), wxT("2"), wxT("3"), wxT("4"), wxT("5"), wxT("6"), wxT("7"), wxT("8"), wxT("9"), wxT("10") };
	int mTTLNChoices = sizeof( mTTLChoices ) / sizeof( wxString );
	mTTL = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, mTTLNChoices, mTTLChoices, 0 );
	mTTL->SetSelection( 4 );
	mTTL->SetToolTip( wxT("Time to Live in seconds") );
	
	bSizer11->Add( mTTL, 1, wxALL, 5 );
	
	bSizer111->Add( bSizer11, 0, wxEXPAND, 5 );
	
	fgSizer2->Add( bSizer111, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer12;
	bSizer12 = new wxBoxSizer( wxVERTICAL );
	
	mDiscoveryNotebook = new wxNotebook( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0 );
	mServicesPanel = new wxPanel( mDiscoveryNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer20;
	bSizer20 = new wxBoxSizer( wxVERTICAL );
	
	mSubsystemTree = new wxTreeCtrl( mServicesPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTR_DEFAULT_STYLE|wxTR_FULL_ROW_HIGHLIGHT );
	bSizer20->Add( mSubsystemTree, 1, wxALL|wxEXPAND, 5 );
	
	mServicesPanel->SetSizer( bSizer20 );
	mServicesPanel->Layout();
	bSizer20->Fit( mServicesPanel );
	mDiscoveryNotebook->AddPage( mServicesPanel, wxT("Services"), false );
	
	bSizer12->Add( mDiscoveryNotebook, 1, wxALL|wxEXPAND, 5 );
	
	fgSizer2->Add( bSizer12, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer19;
	bSizer19 = new wxBoxSizer( wxVERTICAL );
	
	m_staticText18 = new wxStaticText( this, wxID_ANY, wxT("Select Component"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText18->Wrap( -1 );
	bSizer19->Add( m_staticText18, 0, wxALL|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	wxArrayString mSelectComponentChoiceChoices;
	mSelectComponentChoice = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, mSelectComponentChoiceChoices, 0 );
	mSelectComponentChoice->SetSelection( -1 );
	bSizer19->Add( mSelectComponentChoice, 0, wxALL|wxEXPAND, 5 );
	
	m_staticText19 = new wxStaticText( this, wxID_ANY, wxT("Command Options"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText19->Wrap( -1 );
	bSizer19->Add( m_staticText19, 0, wxALL|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	m_staticline1 = new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
	bSizer19->Add( m_staticline1, 0, wxEXPAND | wxALL, 5 );
	
	m_staticText17 = new wxStaticText( this, wxID_ANY, wxT("Authority Level"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText17->Wrap( -1 );
	bSizer19->Add( m_staticText17, 0, wxALL|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	wxArrayString mAuthorityChoiceChoices;
	mAuthorityChoice = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, mAuthorityChoiceChoices, 0 );
	mAuthorityChoice->SetSelection( 0 );
	bSizer19->Add( mAuthorityChoice, 0, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	mControlButton = new wxButton( this, wxID_ANY, wxT("Take Control"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer19->Add( mControlButton, 0, wxALL|wxEXPAND, 5 );
	
	mJoystickButton = new wxButton( this, wxID_ANY, wxT("Take Joystick Control"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer19->Add( mJoystickButton, 0, wxALL|wxEXPAND, 5 );
	
	m_staticline3 = new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
	bSizer19->Add( m_staticline3, 0, wxEXPAND | wxALL, 5 );
	
	mResumeButton = new wxButton( this, wxID_ANY, wxT("Send Resume"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer19->Add( mResumeButton, 0, wxALL|wxEXPAND, 5 );
	
	mStandbyButton = new wxButton( this, wxID_ANY, wxT("Send Standby"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer19->Add( mStandbyButton, 0, wxALL|wxEXPAND, 5 );
	
	mSendShutdown = new wxButton( this, wxID_ANY, wxT("Send Shutdown"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer19->Add( mSendShutdown, 0, wxALL|wxEXPAND, 5 );
	
	fgSizer2->Add( bSizer19, 0, wxEXPAND, 5 );
	
	fgSizer1->Add( fgSizer2, 0, wxEXPAND, 5 );
	
	this->SetSizer( fgSizer1 );
	this->Layout();
	
	// Connect Events
	this->Connect( wxEVT_UPDATE_UI, wxUpdateUIEventHandler( DiscoveryPanelBase::OnUpdateUI ) );
	mInitializeButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DiscoveryPanelBase::OnInitializeButton ), NULL, this );
	mShutdownButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DiscoveryPanelBase::OnShutdownButton ), NULL, this );
	mSubsystemList->Connect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( DiscoveryPanelBase::OnSelectSubsystem ), NULL, this );
	mTTL->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( DiscoveryPanelBase::OnTLL ), NULL, this );
	mDiscoveryNotebook->Connect( wxEVT_COMMAND_NOTEBOOK_PAGE_CHANGED, wxNotebookEventHandler( DiscoveryPanelBase::OnNotebookPageChanged ), NULL, this );
	mSelectComponentChoice->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( DiscoveryPanelBase::OnSelectComponentChoice ), NULL, this );
	mAuthorityChoice->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( DiscoveryPanelBase::OnAuthorityLevel ), NULL, this );
	mControlButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DiscoveryPanelBase::OnTakeControl ), NULL, this );
	mJoystickButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DiscoveryPanelBase::OnTakeJoystickControl ), NULL, this );
	mResumeButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DiscoveryPanelBase::OnSendResume ), NULL, this );
	mStandbyButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DiscoveryPanelBase::OnSendStandby ), NULL, this );
	mSendShutdown->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DiscoveryPanelBase::OnSendShutdown ), NULL, this );
}

DiscoveryPanelBase::~DiscoveryPanelBase()
{
	// Disconnect Events
	this->Disconnect( wxEVT_UPDATE_UI, wxUpdateUIEventHandler( DiscoveryPanelBase::OnUpdateUI ) );
	mInitializeButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DiscoveryPanelBase::OnInitializeButton ), NULL, this );
	mShutdownButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DiscoveryPanelBase::OnShutdownButton ), NULL, this );
	mSubsystemList->Disconnect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( DiscoveryPanelBase::OnSelectSubsystem ), NULL, this );
	mTTL->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( DiscoveryPanelBase::OnTLL ), NULL, this );
	mDiscoveryNotebook->Disconnect( wxEVT_COMMAND_NOTEBOOK_PAGE_CHANGED, wxNotebookEventHandler( DiscoveryPanelBase::OnNotebookPageChanged ), NULL, this );
	mSelectComponentChoice->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( DiscoveryPanelBase::OnSelectComponentChoice ), NULL, this );
	mAuthorityChoice->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( DiscoveryPanelBase::OnAuthorityLevel ), NULL, this );
	mControlButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DiscoveryPanelBase::OnTakeControl ), NULL, this );
	mJoystickButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DiscoveryPanelBase::OnTakeJoystickControl ), NULL, this );
	mResumeButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DiscoveryPanelBase::OnSendResume ), NULL, this );
	mStandbyButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DiscoveryPanelBase::OnSendStandby ), NULL, this );
	mSendShutdown->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DiscoveryPanelBase::OnSendShutdown ), NULL, this );
	
}

RangeSensorPanelBase::RangeSensorPanelBase( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxBoxSizer* bSizer23;
	bSizer23 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer231;
	bSizer231 = new wxBoxSizer( wxVERTICAL );
	
	wxFlexGridSizer* fgSizer4;
	fgSizer4 = new wxFlexGridSizer( 0, 2, 0, 0 );
	fgSizer4->AddGrowableCol( 0 );
	fgSizer4->AddGrowableRow( 0 );
	fgSizer4->SetFlexibleDirection( wxBOTH );
	fgSizer4->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	mHorizontalSlider = new wxSlider( this, wxID_ANY, 50, 1, 99, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL|wxSL_LABELS );
	fgSizer4->Add( mHorizontalSlider, 0, wxALL|wxEXPAND, 5 );
	
	mScanNumberLabel = new wxStaticText( this, wxID_ANY, wxT("Update:   0000"), wxDefaultPosition, wxDefaultSize, 0 );
	mScanNumberLabel->Wrap( -1 );
	fgSizer4->Add( mScanNumberLabel, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	bSizer231->Add( fgSizer4, 0, wxEXPAND, 5 );
	
	wxFlexGridSizer* bSizer251;
	bSizer251 = new wxFlexGridSizer( 0, 2, 0, 0 );
	bSizer251->AddGrowableCol( 0 );
	bSizer251->AddGrowableRow( 0 );
	bSizer251->SetFlexibleDirection( wxBOTH );
	bSizer251->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	mpImageSizer = new wxBoxSizer( wxVERTICAL );
	
	bSizer251->Add( mpImageSizer, 1, wxEXPAND, 5 );
	
	mVerticalSlider = new wxSlider( this, wxID_ANY, 80, 1, 99, wxDefaultPosition, wxDefaultSize, wxSL_LABELS|wxSL_LEFT|wxSL_VERTICAL );
	bSizer251->Add( mVerticalSlider, 0, wxALL|wxEXPAND, 5 );
	
	bSizer231->Add( bSizer251, 1, wxEXPAND, 5 );
	
	bSizer23->Add( bSizer231, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer24;
	bSizer24 = new wxBoxSizer( wxHORIZONTAL );
	
	mRefreshButton = new wxButton( this, wxID_ANY, wxT("Refresh List"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer24->Add( mRefreshButton, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText21 = new wxStaticText( this, wxID_ANY, wxT("Component:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText21->Wrap( -1 );
	bSizer24->Add( m_staticText21, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxArrayString mComponentChoiceChoices;
	mComponentChoice = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, mComponentChoiceChoices, 0 );
	mComponentChoice->SetSelection( 0 );
	bSizer24->Add( mComponentChoice, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	mConnectButton = new wxButton( this, wxID_ANY, wxT("Connect"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer24->Add( mConnectButton, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText20 = new wxStaticText( this, wxID_ANY, wxT("Zoom:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText20->Wrap( -1 );
	bSizer24->Add( m_staticText20, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	mZoomSlider = new wxSlider( this, wxID_ANY, 50, 5, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL|wxSL_LABELS );
	bSizer24->Add( mZoomSlider, 1, wxALL, 5 );
	
	bSizer23->Add( bSizer24, 0, wxEXPAND, 5 );
	
	this->SetSizer( bSizer23 );
	this->Layout();
	
	// Connect Events
	this->Connect( wxEVT_SET_FOCUS, wxFocusEventHandler( RangeSensorPanelBase::OnSetFocus ) );
	this->Connect( wxEVT_UPDATE_UI, wxUpdateUIEventHandler( RangeSensorPanelBase::OnUpdateUI ) );
	mRefreshButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RangeSensorPanelBase::OnRefreshList ), NULL, this );
	mConnectButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RangeSensorPanelBase::OnConnect ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
}

RangeSensorPanelBase::~RangeSensorPanelBase()
{
	// Disconnect Events
	this->Disconnect( wxEVT_SET_FOCUS, wxFocusEventHandler( RangeSensorPanelBase::OnSetFocus ) );
	this->Disconnect( wxEVT_UPDATE_UI, wxUpdateUIEventHandler( RangeSensorPanelBase::OnUpdateUI ) );
	mRefreshButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RangeSensorPanelBase::OnRefreshList ), NULL, this );
	mConnectButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RangeSensorPanelBase::OnConnect ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( RangeSensorPanelBase::OnZoomScroll ), NULL, this );
	
}

LocalPoseSensorPanelBase::LocalPoseSensorPanelBase( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxBoxSizer* bSizer23;
	bSizer23 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer231;
	bSizer231 = new wxBoxSizer( wxVERTICAL );
	
	wxFlexGridSizer* fgSizer4;
	fgSizer4 = new wxFlexGridSizer( 0, 2, 0, 0 );
	fgSizer4->AddGrowableCol( 0 );
	fgSizer4->AddGrowableRow( 0 );
	fgSizer4->SetFlexibleDirection( wxBOTH );
	fgSizer4->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	mScanNumberLabel = new wxStaticText( this, wxID_ANY, wxT("Pose Data"), wxDefaultPosition, wxDefaultSize, 0 );
	mScanNumberLabel->Wrap( -1 );
	fgSizer4->Add( mScanNumberLabel, 0, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 5 );
	
	bSizer231->Add( fgSizer4, 0, wxEXPAND, 5 );
	
	wxFlexGridSizer* bSizer251;
	bSizer251 = new wxFlexGridSizer( 0, 2, 0, 0 );
	bSizer251->AddGrowableCol( 0 );
	bSizer251->AddGrowableRow( 0 );
	bSizer251->SetFlexibleDirection( wxBOTH );
	bSizer251->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	mpImageSizer = new wxBoxSizer( wxVERTICAL );
	
	bSizer251->Add( mpImageSizer, 1, wxEXPAND, 5 );
	
	bSizer231->Add( bSizer251, 1, wxEXPAND, 5 );
	
	bSizer23->Add( bSizer231, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer24;
	bSizer24 = new wxBoxSizer( wxHORIZONTAL );
	
	mRefreshButton = new wxButton( this, wxID_ANY, wxT("Refresh List"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer24->Add( mRefreshButton, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText21 = new wxStaticText( this, wxID_ANY, wxT("Component:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText21->Wrap( -1 );
	bSizer24->Add( m_staticText21, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxArrayString mComponentChoiceChoices;
	mComponentChoice = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, mComponentChoiceChoices, 0 );
	mComponentChoice->SetSelection( 0 );
	bSizer24->Add( mComponentChoice, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	mConnectButton = new wxButton( this, wxID_ANY, wxT("Connect"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer24->Add( mConnectButton, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText20 = new wxStaticText( this, wxID_ANY, wxT("Zoom:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText20->Wrap( -1 );
	bSizer24->Add( m_staticText20, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	mZoomSlider = new wxSlider( this, wxID_ANY, 50, 10, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL|wxSL_LABELS );
	bSizer24->Add( mZoomSlider, 1, wxALL, 5 );
	
	bSizer23->Add( bSizer24, 0, wxEXPAND, 5 );
	
	this->SetSizer( bSizer23 );
	this->Layout();
	
	// Connect Events
	this->Connect( wxEVT_SET_FOCUS, wxFocusEventHandler( LocalPoseSensorPanelBase::OnSetFocus ) );
	this->Connect( wxEVT_UPDATE_UI, wxUpdateUIEventHandler( LocalPoseSensorPanelBase::OnUpdateUI ) );
	mRefreshButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( LocalPoseSensorPanelBase::OnRefreshList ), NULL, this );
	mConnectButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( LocalPoseSensorPanelBase::OnConnect ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
}

LocalPoseSensorPanelBase::~LocalPoseSensorPanelBase()
{
	// Disconnect Events
	this->Disconnect( wxEVT_SET_FOCUS, wxFocusEventHandler( LocalPoseSensorPanelBase::OnSetFocus ) );
	this->Disconnect( wxEVT_UPDATE_UI, wxUpdateUIEventHandler( LocalPoseSensorPanelBase::OnUpdateUI ) );
	mRefreshButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( LocalPoseSensorPanelBase::OnRefreshList ), NULL, this );
	mConnectButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( LocalPoseSensorPanelBase::OnConnect ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
	mZoomSlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( LocalPoseSensorPanelBase::OnZoomScroll ), NULL, this );
	
}
