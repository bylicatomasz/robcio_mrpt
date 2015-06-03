
#include <mrpt/otherlibs/mathplot/mathplot.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#ifndef ROBCIOWINPLOT_H
#define ROBCIOWINPLOT_H
#include "RobcioData.h"
using namespace std;
using namespace mrpt::gui;
class RobcioWinPlot 
{
private:
	bool isReady;
	CDisplayWindowPlots *plot;
	RobcioData *robcioData;
  public:
	RobcioWinPlot(RobcioData *robcioDataArg);
    void initPlot();
	
	int runWin(int argc, char *argv[]);
	void updateScan();
	void updateScan(vector<double>    Xs,vector<double> Ys,  vector<double>    pathX,vector<double>   pathY,bool *lock );
};
#endif
