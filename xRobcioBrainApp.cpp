
#include "xRobcioBrainApp.h"
#include <wx/stdpaths.h>

//(*AppHeaders
#include "xRobcioWinWidgets.h"
#include <wx/image.h>
//*)
#include <wx/log.h>

#include <locale.h>

IMPLEMENT_APP(xRobcioBrainApp)

	
bool xRobcioBrainApp::OnInit()
{
	// Starting in wxWidgets 2.9.0, we must reset numerics locale to "C",
	//  if we want numbers to use "." in all countries. The App::OnInit() is a perfect place to undo
	//  the default wxWidgets settings. (JL @ Sep-2009)
	wxSetlocale(LC_NUMERIC,wxString(wxT("C")));

  

    //(*AppInitialize
    bool wxsOK = true;
    wxInitAllImageHandlers();
    if ( wxsOK )
    {
		
    	xRobcioWinWidgetsFrame* Frame = new xRobcioWinWidgetsFrame(0);
    	Frame->Show();
    	SetTopWindow(Frame);
    }
    //*)
    return wxsOK;

}

int xRobcioBrainApp::OnExit()
{


    return 0;
}
