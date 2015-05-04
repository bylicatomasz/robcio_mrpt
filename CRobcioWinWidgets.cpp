#include "CRobcioWinWidgets.h"



CRobcioWinWidgets			*theMainWindow = NULL;


CRobcioWinWidgets::CRobcioWinWidgets(wxWindow* parent,wxWindowID id)
		: m_fileHistory( 9 /* Max file list*/ )
{
	theMainWindow = this;

	

	//(*Initialize(xRawLogViewerFrame)
	

	Create(parent, id, _("RawlogViewer - Part of the MRPT project"), wxDefaultPosition, wxDefaultSize, wxCAPTION|wxDEFAULT_FRAME_STYLE|wxSYSTEM_MENU|wxRESIZE_BORDER|wxCLOSE_BOX|wxMAXIMIZE_BOX|wxMINIMIZE_BOX, _T("id"));
	SetClientSize(wxSize(700,500));
	{
		wxIcon FrameIcon;
		FrameIcon.CopyFromBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("MAIN_ICON")),wxART_OTHER));
		SetIcon(FrameIcon);
	}
	
	// Construction of "global" dialog variables:
	// ----------------------------------------------
	/*
	CFormRawMap                     *formRawMap = NULL;
	formRawMap = new CFormRawMap(this);
		formRawMap->ShowModal();*/
	//scanMatchingDialog = new CScanMatching(this);
	OnRawMapOdo();
}

void CRobcioWinWidgets::OnRawMapOdo()
{

	 formRawMap = new CFormRawMap(this);
	 CRawlog rawlog = robcio.rawLog;
	

	// Set slider values:
	//  If they have changed, we have a different rawlog loaded, thus we select
	//  the whole range by default:
	bool selectMaxRange = ((size_t)formRawMap->slFrom->GetMax()) != rawlog.size()-1;
	formRawMap->slFrom->SetRange( 0,(int)rawlog.size()-1 );
	formRawMap->slTo->SetRange( 0,(int)rawlog.size()-1 );

	formRawMap->edFirst->SetRange( 0,(int)rawlog.size()-1 );
	formRawMap->edLast->SetRange( 0,(int)rawlog.size()-1 );

	if (selectMaxRange)
	{
		formRawMap->slFrom->SetValue(0);
		formRawMap->slTo->SetValue((int)rawlog.size()-1);
	}

	// Clear the graphs:
	formRawMap->plotMap->DelAllLayers(true,false);

	// Show:
	formRawMap->edFirst->SetValue( formRawMap->slFrom->GetValue() );
	formRawMap->edLast->SetValue( formRawMap->slTo->GetValue() );

	// Enable "results" buttons:
	formRawMap->btnSaveTxt->Disable();
	formRawMap->btnSave3D->Disable();
	formRawMap->btnSavePath->Disable();
	formRawMap->btnSaveTxt->Disable();
	formRawMap->btnSaveObsPath->Disable();
	formRawMap->btnView3D->Disable();



	//formRawMap->ShowModal();
	formRawMap->generateMap(rawlog);
	
	CRobcioWinWidgets test=this;
	initWebService(&robcio);
	formRawMap->ShowModal();

}
void CRobcioWinWidgets::OnRawMapReload(){
	 CRawlog rawlog = robcio.rawLog;
	formRawMap->generateMap(rawlog);
}

CRobcioWinWidgets::~CRobcioWinWidgets()
{
	//(*Destroy(xRawLogViewerFrame)
	//*)

	// Destroy dialogs:
	/*
	delete formRawMap;
	formRawMap = NULL;
	delete scanMatchingDialog;
	scanMatchingDialog = NULL;

	*/


}