#include <mrpt/otherlibs/mathplot/mathplot.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
using namespace std;
using namespace mrpt::gui;
class RobcioWinPlot 
{
private:
	bool isReady;
	CDisplayWindowPlots *plot;
  public:
	RobcioWinPlot();
    void initPlot();
	int runWin(int argc, char *argv[]);
	void updateScan(vector<double>    Xs,vector<double> Ys,  vector<double>    pathX,vector<double>   pathY,bool *lock );
};
