#include "RobcioWinFrame.h"


#include <wx/msgdlg.h>
#include <wx/filedlg.h>
#include <wx/progdlg.h>
#include <wx/app.h>

//(*InternalHeaders(CFormRawMap)
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/font.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
#include <thread>
//*)
// General global variables:

int sizeX=500;
int sizeY=500;


using namespace std;
RobcioWinFrame::RobcioWinFrame(const wxString& title,wxWindow *parent)
	: wxFrame(parent, wxID_ANY, title, wxDefaultPosition, wxSize(sizeX, sizeY))
{
	
	SetFocus();
	Centre();
	initMap();


};
void RobcioWinFrame::initMap(){
	plotMap = new mpWindow( this, -1, wxPoint(0,0), wxSize(sizeX-10,sizeY-10), wxSUNKEN_BORDER );

	lyPoints = new mpFXYVector();
	lyPath   = new mpFXYVector();
	lyPath->SetPen( wxPen(wxColor(255,0,0),2) );
	lyPath->SetContinuity( true );
	lyPoints->SetPen( wxPen(wxColor(0,0,255),0) );
	plotMap->AddLayer( lyPoints );
	plotMap->AddLayer( lyPath );
	plotMap->EnableDoubleBuffer(true);

	plotMap->LockAspect(false);
	// Update the window to show the new data fitted.
	plotMap->LockAspect(true);
	plotMap->AddLayer( new mpScaleX() );
	plotMap->AddLayer( new mpScaleY() );

}
void RobcioWinFrame:: generateMapFromVectorXY(vector<double>    Xs,vector<double> Ys,  vector<double>    pathX,vector<double>   pathY,wxColour colorMap,wxColour colorPath,bool *lock )
{
	if(*lock==false){
		*lock=true;
		lyPath->SetData( pathX,pathY );
		lyPoints->SetData(Xs,Ys);
		plotMap->Fit(); 
		pathX.clear();
		pathY.clear();
		Xs.clear();
		Ys.clear();
		*lock=false;
	}else{
		generateMapFromVectorXY(	Xs,	 Ys,	pathX,	 pathY, colorMap, colorPath,lock );
	}
}