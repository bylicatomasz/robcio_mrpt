#include "RobcioWinPlot.h"

using namespace std;
using namespace mrpt::gui;

//*)
// General global variables:

int sizeX=500;
int sizeY=500;


using namespace std;
RobcioWinPlot::RobcioWinPlot()
	
{
	
	initPlot();

};
void RobcioWinPlot::initPlot(){
		plot=new CDisplayWindowPlots ("View Scane Range",sizeX,sizeY);
	
		
}
void RobcioWinPlot:: updateScan(vector<double>    Xs,vector<double> Ys,  vector<double>    pathX,vector<double>   pathY,bool *lock )
{
	if(*lock==false){
		*lock=true;
		plot->plot(Xs,Ys,"b.","plotXY");
		plot->plot(pathX,pathY,"r-","pathXY");
		pathX.clear();
		pathY.clear();
		Xs.clear();
		Ys.clear();
		*lock=false;
	}else{
		updateScan(	Xs,	 Ys,	pathX,	 pathY,lock );
	}
}