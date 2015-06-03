/* +---------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)               |
|                          http://www.mrpt.org/                             |
|                                                                           |
| Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
| See: http://www.mrpt.org/Authors - All rights reserved.                   |
| Released under BSD License. See details in http://www.mrpt.org/License    |
+---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
APPLICATION: Particle Filter (Global) Localization Demo
FILE: pf_localization_main.cpp
AUTHOR: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>

For instructions and more:
http://www.mrpt.org/Application:pf-localization
---------------------------------------------------------------*/



#include "RobcioData.h"

#include "RobcioLocalizationPF.h"
#include "RobcioMapTools.h"
#include "RobcioTools.h"
#include "RobcioWinPlot.h"
#include "RobcioWebService.h"

#include <mrpt/system/threads.h> // sleep()
#include <mrpt/system/os.h>




#include <string>
#include <iostream>
#include <thread>



using namespace mrpt;
using namespace mrpt::system;
using namespace std;




//------------------------------------------------------
//				Forward declaration
//-------------------------------------------------------


void  startProcUpdateWindowScanView();
void  startProcUpdateSLAM();
void  startWebService();




//------------------------------------------------------
//--------------------Varibles--------------------------
//------------------------------------------------------


string file_rawlog_output;
string file_input_csv="C:/Robotics/mrpt-1.2.1/apps/RobcioBrain/gridMap/PartSmallMoved.csv";

string CONFIG_FILE="C:/Robotics/mrpt-1.2.1/apps/RobcioApp/config_files/localization_demo.ini";
string RAW_LOG="C:/Robotics/mrpt-1.2.1/apps/RobcioApp/config_files/CprrectPartMapMoved.rawlog";

RobcioData *robcioData;

RobcioWinPlot *robcioWinPlot;
RobcioWebService *robcioWebService;
RobcioLocalizationPF *robcioLocalizationPF;
bool isRunMain=true;



// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		
		
		printf(" RobcioApp using as base code pf-localization\n");	
		printf("-------------------------------------------------------------------\n");
		isRunMain=true;
		robcioData=new RobcioData();
		sleep(1000);
		robcioWinPlot=new RobcioWinPlot(robcioData);
		sleep(1000);
		robcioWebService=new RobcioWebService(robcioData);
		sleep(1000);
		robcioLocalizationPF=new RobcioLocalizationPF(robcioData);
		
		sleep(2000);
	


		//------------------------------------------------------------
		//				Thread 
		//------------------------------------------------------------
		std::thread helperThreadWebService(startWebService);
		sleep(1000);
			
		std::thread helperThreadTimerProcUpdateSLAM(startProcUpdateSLAM);
			sleep(1000);
		std::thread helperThreadTimerProcUpdateWindowScanView(startProcUpdateWindowScanView);

		
		//robcioLocalizationPF->importFromCSVFile();
		pause();

		return 1;
	}
	catch (exception &e)
	{
		cout << "Caught MRPT exception:\n" << e.what() << endl;

		pause();
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		pause();
		return -1;
	}
}

//------------------------------------------------------------
//				WinForm
//-----------------------------------------------------------

void   startProcUpdateWindowScanView()
{

	while(isRunMain){
		sleep(1323);		
		robcioWinPlot->updateScan();
	}

};
void startProcUpdateSLAM(){

	robcioLocalizationPF->do_pf_localization_slam(CONFIG_FILE);


};
void startWebService(){


	robcioWebService->startWebService();

};



