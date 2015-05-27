/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CROBCIOSLAM_H
#define CROBCIOSLAM_H



// The "custom class" mpWindow, from the wxMathPlot libray by David Schalig
//  See http://sourceforge.net/projects/wxmathplot
#include <mrpt/obs.h>
#include <mrpt/base.h>
#include <mrpt/utils.h>
#include <mrpt/maps.h>

// General global variables:
#include <mrpt/slam.h>
#include <mrpt/topography.h>
#include <mrpt/system/datetime.h>
using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::system;

class CRobcioSLAM
{
	private:

		

		//(*Declarations(CRobcioSLAM)
		bool isStart;
		double compasRadius;
		double dystance;
		string actionStepLast;
		double lastX;
		double lastY;
		int lendmarkId;

		
		
		CPose2D  odoLast;
		CFileGZOutputStream	out_file;
		CSensoryFrame	curSF;
		//*)

		 //(*Handlers(CRobcioSLAM)
		vector<double> parseLineLog(string line);
		string parseLineGetStat(string line);
		vector<double> getCordinate(double dystance, double radius);
		
		//*)
		public:
			

	   //(*Handlers(CRobcioSLAM)
			void test();
			CRawlog rawLog;
			void testMapBuilderRBPF();
			//COccupancyGridMap2D gridmap;
			CPointsMapPtr mainMap;
			std::string currentDateTime();
			CPointsMapPtr createEmptyMap();
			void importFromCSVFile(CPointsMapPtr gridmapLoad);
			void readDataScanFromString( std::string line,CPointsMapPtr gridmapLoad);
			void initFileGridExport();
			void closeFileGridExport();
			//void readDataScanAndCreateMap(std::string line);
			CRawlog createRawlog();
			CPointsMapPtr loadMap(string path);
			CPointsMapPtr loadMapFromRawLog(string pathToRawLog);
			void saveMap(CPointsMapPtr mapToSave,string path);
			void testMapBuilderRBPF(string pathToRawLog,string pathToMainMap,CSimpleMap &finalMap);
			void testSLAM(string pathToRawLog,string pathToMainMap,CSimpleMap &finalMap); 
			//void saveMapToRawLog(CPointsMapPtr mapToSave);
			void alignICP(CSimplePointsMap		*mainMap,CSimplePointsMap *scanedMap,float thresholdDist);
			CSimpleMap createSimpleMapPointSLAM(string pathToRawLog);
			void saveSimpleMap(CSimpleMap mapToSave,string path);
			CSimpleMap loadSimpleMap(string path) ;
			CPointsMapPtr testSimpleMap(string pathToRawLog,string pathToFile);
			void testKFSLAM2D();
			void testMapICP(string pathToRawLog,string pathToMainMap,vector<float> *mapX,vector<float>  *mapY,vector<double> *pathX,vector<double>  *pathY);
			void testOtherSLAM(string pathToSimpleMap,string pathToRawLog,vector<double> *poseX,vector<double> *poseY,vector<float> *poseMapX,vector<float> *poseMapY);
			CMultiMetricMap	createMetricMap(string pathToMainMap);
			void importFromCSV(std::string csvFile);
			//xRobcioWinWidgetsFrame form;
		//*)


};

#endif
