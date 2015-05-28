#include <wx/wx.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/spinctrl.h>
#include <wx/slider.h>
#include <wx/panel.h>
#include <wx/bmpbuttn.h>
#include <wx/button.h>
#include <wx/dialog.h>
// General global variables:

#include <mrpt/otherlibs/mathplot/mathplot.h>

using namespace std;

class RobcioWinFrame : public wxFrame
{
private:
	mpWindow* plotMap;
public:
	std::vector<float> dupa ;
	//vector<float> dupa;
    RobcioWinFrame(const wxString& title);
	void generateMapFromVectorXY(vector<double>    Xs,vector<double> Ys,  vector<double>    pathX,vector<double>   pathY,wxColour colorMap,wxColour colorPath);
};
