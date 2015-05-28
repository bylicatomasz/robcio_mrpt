#include "RobcioWinApp.h"



IMPLEMENT_APP_NO_MAIN(RobcioWinApp);
IMPLEMENT_WX_THEME_SUPPORT;

bool RobcioWinApp::OnInit()
{
    scanFrame = new RobcioWinFrame(wxT("Scan range view"));
    scanFrame->Show(true);
	isReady=true;
    return true;
}
void RobcioWinApp::updateScan(vector<double>    *Xs,vector<double> *Ys,  vector<double>    *pathX,vector<double>   *pathY){
	if(isReady){
		scanFrame->generateMapFromVectorXY(  *Xs,*Ys,*pathX,*pathY,wxColour(255,0,0) ,wxColour(0,0,255));
		scanFrame->Refresh();
	}
}
int RobcioWinApp:: runWin(int argc, char *argv[])
{
	isReady=false;
    wxEntryStart( argc, argv );

    wxTheApp->CallOnInit();
    wxTheApp->OnRun();

    return 0;
}