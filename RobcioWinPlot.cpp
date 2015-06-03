#include "RobcioWinPlot.h"
#include "RobcioData.h"
using namespace std;
using namespace mrpt::gui;

//*)
// General global variables:

int sizeX=500;
int sizeY=500;


using namespace std;
RobcioWinPlot::RobcioWinPlot(RobcioData *robcioDataArg)
	
{
	isReady=false;
	this->robcioData=robcioDataArg;
	initPlot();

};
void RobcioWinPlot::initPlot(){
		plot=new CDisplayWindowPlots ("View Scane Range",sizeX,sizeY);
		
		isReady=true;

		
		
		
}

void RobcioWinPlot::updateScan(){
	if(isReady && plot->isOpen()){
		plot->axis_fit(true);
		vector<double> mX=robcioData->getDataMapX();
		vector<double> mY=robcioData->getDataMapY();
		vector<double> pX=robcioData->getDataPathX();
		vector<double> pY=robcioData->getDataPathY();
		if(mX.size()==mY.size() && pX.size()==pY.size()){
			updateScan(mX,mY,pX,pY,false);
		}
	}
	
		
}
void RobcioWinPlot:: updateScan(vector<double>    Xs,vector<double> Ys,  vector<double>    pathX,vector<double>   pathY,bool *lock )
{
		
		plot->plot(Xs,Ys,"b.","plotXY");
		plot->plot(pathX,pathY,"r-","pathXY");

}