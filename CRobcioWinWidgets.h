/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CROBCIOWINWIDGETS_H
#define CROBCIOWINWIDGETS_H


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
#include "CRobcioWebService.h"

class CRobcioWinWidgets: public wxFrame
{
	
public:

    CRobcioWinWidgets(wxWindow* parent,wxWindowID id = -1);
    virtual ~CRobcioWinWidgets();
	void OnRawMapReload();


 

private:

    /** Loads the given file in memory, in the varibale "rawlog"
      */
      wxFileHistory	m_fileHistory;
	  CRobcioSLAM robcio;
	  CFormRawMap                     *formRawMap ;
	  void OnRawMapOdo();

  


  


 
};



#endif // XROBCIOWINWIDGETS_H
