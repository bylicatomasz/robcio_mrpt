#include <wx/wx.h>
#include "RobcioWinFrame.h"
using namespace std;
class RobcioWinApp : public wxApp
{
private:
	RobcioWinFrame *scanFrame;
	bool isReady;
  public:
    virtual bool OnInit();
	int runWin(int argc, char *argv[]);
	void updateScan(vector<double>    *Xs,vector<double> *Ys,  vector<double>    *pathX,vector<double>   *pathY);
};
