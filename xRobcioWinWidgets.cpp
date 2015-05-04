#include "xRobcioWinWidgets.h"
#include "MyThread.h"
#include <boost/network/protocol/http/server.hpp>
#include <string>
#include <iostream>
#include <thread>

namespace http = boost::network::http;


//-----------------------------------------------------------------------------
// wxStaticBitmapPopup
//-----------------------------------------------------------------------------
const long wxStaticBitmapPopupRobcio ::ID_MENUITEM_IMG_LOAD = wxNewId();
const long wxStaticBitmapPopupRobcio ::ID_MENUITEM_IMG_SAVE = wxNewId();

//-----------------------------------------------------------------------------
//Create webservice
//-----------------------------------------------------------------------------




struct webServiceListner;
typedef http::server<webServiceListner> server;
CRobcioSLAM *robcio;
xRobcioWinWidgetsFrame				*theMainWindow = NULL;
xRobcioWinWidgetsFrame *form;
struct webServiceListner {
    void operator() (server::request const &request,
                     server::response &response) {
        /*std::string ip = source(request);
        response = server::response::stock_reply(
            server::response::ok, std::string("Hello, ") + ip + "!");
			*/

		server::string_type ip = source(request);
		server::string_type body_ = body(request);
		
		server::string_type method_ = method(request);
		unsigned int port = request.source_port;
        std::ostringstream data;
		std::ostringstream data2;

		data << "OK"<< "\n";
		if(body_.empty()==false && body_.length()>10){
			//::robcio->readDataScanFromCSV(urlDecode(body_).replace(0,8,""));
		//	::xRobcioWinWidgetsFrame->OnRawMapReload();
			//::form->OnRawMapReload();
		}
        response = server::response::stock_reply(

			server::response::ok, data.str());


    }
	void log(...) {
        // do nothing
    }
	std::string urlDecode(std::string &SRC) {
			std::string ret;
			char ch;
			int i, ii;
			for (i=0; i<SRC.length(); i++) {
				if (int(SRC[i])==37) {
					sscanf(SRC.substr(i+1,2).c_str(), "%x", &ii);
					ch=static_cast<char>(ii);
					ret+=ch;
					i=i+2;
				} else {
					ret+=SRC[i];
				}
			}
			return (ret);
		}
};
void startWebService(){
  try {
	  	char *address="127.0.0.1";
	   char *port="5151";
       webServiceListner handler;
		server::options options(handler);
		server server_(options.address(address).port(port));
        //server server_(address, port, handler);
        server_.run();
    }
    catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
   
    }

};
std::thread helper1(startWebService);


// Constructors
wxStaticBitmapPopupRobcio ::wxStaticBitmapPopupRobcio ( wxWindow *parent, wxWindowID id, const wxBitmap&img, const wxPoint &pos, const wxSize &size, int flag,const wxString &name )
		: wxStaticBitmap( parent, id, img, pos, size, flag, name )
{
	wxMenuItem *mnu1 = new wxMenuItem((&mnuImages), ID_MENUITEM_IMG_SAVE, _("Save image to file..."), wxEmptyString, wxITEM_NORMAL);
	wxMenuItem *mnu2 = new wxMenuItem((&mnuImages), ID_MENUITEM_IMG_LOAD, _("Replace image by another one from file..."), wxEmptyString, wxITEM_NORMAL);

	mnuImages.Append( mnu1 );
	mnuImages.Append( mnu2 );
}
wxStaticBitmapPopupRobcio ::~wxStaticBitmapPopupRobcio ( )
{
}
void wxStaticBitmapPopupRobcio ::OnShowPopupMenu(wxMouseEvent &event)
{
	PopupMenu( &mnuImages );
}


BEGIN_EVENT_TABLE(xRobcioWinWidgetsFrame, wxFrame)
EVT_COMMAND  (NUMBER_UPDATE_ID, wxEVT_COMMAND_TEXT_UPDATED, xRobcioWinWidgetsFrame::OnRawMapReload)
END_EVENT_TABLE()

xRobcioWinWidgetsFrame::xRobcioWinWidgetsFrame(wxWindow* parent,wxWindowID id)
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
	 // create the thread
      
	OnRawMapOdo();
}



void xRobcioWinWidgetsFrame::OnRawMapOdo()
{

	 formRawMap = new CFormRawMap(this);
	 //CRawlog rawlog = robcio.createRawlog();
	  CRawlog rawlog ;
	

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
//	formRawMap->generateMap(rawlog);
//	xRobcioWinWidgetsFrame test=this;

	//
		::form=this;
		initWebService(&robcio);
		  MyThread* t = new MyThread(this);
        wxThreadError err = t->Create();
 
        if (err != wxTHREAD_NO_ERROR)
        {
            wxMessageBox( _("Couldn't create thread!") );
          //  return false;
        }
 
        err = t->Run();

	formRawMap->ShowModal();


}
void xRobcioWinWidgetsFrame::OnRawMapReload(wxCommandEvent& evt){
	 CRawlog rawlog = robcio.rawLog;
	// COccupancyGridMap2D gridmapLoad=robcio.gridmap;
	 /*
	 if(rawlog.size()>0){
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

	
		
	 }*/
		std::cout << "test\n";
	 
}
void xRobcioWinWidgetsFrame::initWebService(CRobcioSLAM *rob){

	
	std::cout << "starting first helper...\n";
	::robcio=rob;
	::robcio->initFileGridExport();
	CPointsMapPtr gridmapLoad=::robcio->loadMapFromGrid();
	CRawlog rawlog = robcio.rawLog;
	formRawMap->generateMapFromMapGrid(gridmapLoad,1);
	CPointsMapPtr mapGrid2=::robcio->createEmptyMap();
	::robcio->importFromCSVFile(mapGrid2);
	formRawMap->generateMapFromMapGrid(mapGrid2,2);
	//::robcio->closeFileGridExport();
   // ::form=form;
};


xRobcioWinWidgetsFrame::~xRobcioWinWidgetsFrame()
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