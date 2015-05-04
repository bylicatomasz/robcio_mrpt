/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef XROBCIOWINWIDGETS_H
#define XROBCIOWINWIDGETS_H


//(*Headers(xRobcioWinWidgets)

#include <wx/notebook.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/menu.h>
#include <wx/textctrl.h>
#include <wx/things/toggle.h>
#include <wx/splitter.h>
#include <wx/statline.h>

#include <wx/slider.h>
#include <wx/panel.h>
#include <wx/statbmp.h>
#include <wx/button.h>
#include <wx/frame.h>
#include <wx/timer.h>
#include <wx/combobox.h>
#include <wx/statusbr.h>
#include <wx/msgdlg.h>
//*)

#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/artprov.h>
#include <wx/combobox.h>

#include <map>
#include <string>
#include <wx/docview.h>

// General global variables:
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/gui/CDisplayWindowPlots.h>

#include "CScanMatching.h"
#include "CFormRawMap.h"
#include "CRobcioSLAM.h"



// A list of sensor labels (and the times they appear) in the currently loaded rawlog.
struct TInfoPerSensorLabel
{
	TInfoPerSensorLabel() : occurences(0), first(INVALID_TIMESTAMP) ,last(INVALID_TIMESTAMP)
	{}
	size_t					occurences;
	mrpt::system::TTimeStamp	first,last;
};


class wxStaticBitmapPopupRobcio : public wxStaticBitmap
{
public:
    wxStaticBitmapPopupRobcio  () {}
    wxStaticBitmapPopupRobcio  ( wxWindow *parent, wxWindowID id,
                          const wxBitmap&img,
                          const wxPoint &pos = wxDefaultPosition,
                          const wxSize &size = wxDefaultSize,
                          int flags = 0, const wxString &name=wxT(""));
    ~wxStaticBitmapPopupRobcio  ();

    void OnShowPopupMenu(wxMouseEvent &event);

protected:
    wxMenu mnuImages;

    static const long ID_MENUITEM_IMG_LOAD;
    static const long ID_MENUITEM_IMG_SAVE;

};



// The "custom class" mpWindow, from the wxMathPlot libray by David Schalig
//  See http://sourceforge.net/projects/wxmathplot
#include <mrpt/otherlibs/mathplot/mathplot.h>

// Auxiliary data types used to import ALOG files:
struct TAlogRecord
{
	std::string		label;
	char   			type;  // 0: odo, 1: 2d laser, 2:3d laser, 3: image
	std::vector<float>   data;
	std::string		imgFile;
	double          startElev, endElev;
};


class xRobcioWinWidgetsFrame: public wxFrame
{
	friend class wxStaticBitmapPopupRobcio;

public:

    xRobcioWinWidgetsFrame(wxWindow* parent,wxWindowID id = -1);
    virtual ~xRobcioWinWidgetsFrame();
	void OnRawMapReload(wxCommandEvent& evt);


   DECLARE_EVENT_TABLE();

private:

    /** Loads the given file in memory, in the varibale "rawlog"
      */
      wxFileHistory	m_fileHistory;
	  CRobcioSLAM robcio;
	  CFormRawMap                     *formRawMap ;
	  void OnRawMapOdo();
	  void initWebService(CRobcioSLAM *rob);

  


  


 
};



#endif // XROBCIOWINWIDGETS_H
