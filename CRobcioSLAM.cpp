#include "CRobcioSLAM.h"
// General global variables:
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/slam/CPointsMap.h>
#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/math/geometry.h>
#include <mrpt/topography.h>
#include <wx/string.h>
#include "CRobcioTools.h"

void CRobcioSLAM::test(){
	//	RobcioWinApp winapp;
	//	winapp.OnInit();
	printf("Test");
};
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}
std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, elems);
	return elems;
}
string CRobcioSLAM:: currentDateTime() {
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M-%S", &tstruct);

	return buf;
};
vector<double> CRobcioSLAM::parseLineLog(string line){
	vector<double> returnArr;	
	vector<string> arrSplit = split(line, ';');
	for(int i=0;i<arrSplit.size();i++){			
		if(i==4){
			continue;
		}
		replace(arrSplit[i].begin(),arrSplit[i].end(),',','.');				
		double num=atof(arrSplit[i].c_str());
		returnArr.push_back(num);


	}
	return returnArr;
};
string CRobcioSLAM::parseLineGetStat(string line){
	vector<double> returnArr;	
	vector<string> arrSplit = split(line, ';');
	return arrSplit[4];
};
vector<double> CRobcioSLAM::getCordinate(double dystance, double radius){
	double x = dystance * cos(radius);
	double y = dystance * sin(radius);


	vector<double>  arrayDouble;

	arrayDouble.push_back(x);
	arrayDouble.push_back(y);

	return arrayDouble;

};
CPointsMapPtr CRobcioSLAM::createEmptyMap() { 


	// Go: generate the map:

	CMultiMetricMap			theMap;


	ifstream inputConfig( "c:\\Temp\\map.ini" );
	std::string configMapStr;
	for( std::string line; getline( inputConfig, line ); )
	{
		configMapStr+=line+"\n";

	}

	// Create a memory "ini file" with the text in the window:
	CConfigFileMemory       configSrc( CStringList( std::string(configMapStr.c_str()) ) );

	TSetOfMetricMapInitializers		lstMaps;
	lstMaps.loadFromConfigFile( configSrc, "map" );
	theMap.setListOfMaps( &lstMaps );

	CPointsMapPtr	thePntsMap;

	if( !theMap.m_pointsMaps.empty() )
		thePntsMap = theMap.m_pointsMaps[0];
	else if (theMap.m_colourPointsMap.present())
		thePntsMap = theMap.m_colourPointsMap;



	return thePntsMap;


}
void CRobcioSLAM::testMapBuilderRBPF(string pathToRawLog,string pathToMainMap,CSimpleMap &finalMap){
	// ---------------------------------
	//		MapPDF opts
	// ---------------------------------
	CMetricMapBuilderRBPF::TConstructionOptions		rbpfMappingOptions;
	string INI_FILENAME="C:/mrpt-1.0.2-new/mrpt-1.0.2/share/mrpt/config_files/rbpf-slam/gridmapping_RBPF_ICPbased_malaga.ini";
	rbpfMappingOptions.loadFromConfigFile(CConfigFile(INI_FILENAME),"MappingApplication");
	rbpfMappingOptions.dumpToConsole();

	// ---------------------------------
	//		Constructor
	// ---------------------------------
	CMetricMapBuilderRBPF mapBuilder( rbpfMappingOptions );

	// ---------------------------------
	//   CMetricMapBuilder::TOptions
	// ---------------------------------
	mapBuilder.options.verbose					= true;
	mapBuilder.options.enableMapUpdating		= true;
	mapBuilder.options.debugForceInsertion		= false;


	

	CActionCollectionPtr action;
	CSensoryFramePtr observations;
	
	//CPointsMapPtr mapMain=loadMap(pathToMainMap);
	CRawlog rawLogTest;
	size_t      decimate =1;
//	CSimpleMap initialMap=(*mapMain).getAsSimplePointsMap();
	rawLogTest.loadFromRawLogFile(pathToRawLog);


	CSimpleMap	initialMap=loadSimpleMap(pathToMainMap) ;
	//mapBuilder.initialize(initialMap);
	// Go: generate the map:
	size_t      i;
	CPose2D     curPose(0,0,0);
	RobcioTools  tools;
	for (i=0; i<rawLogTest.size();i++)
	{

		bool addNewPathEntry = false;
		
		switch( rawLogTest.getType(i) )
		{
		case CRawlog::etActionCollection:
			{
				CActionCollectionPtr    acts = rawLogTest.getAsAction(i);
				CPose2D                 poseIncrement;
				bool                    poseIncrementLoaded = false;

				for (size_t j=0;j<acts->size();j++)
				{
					CActionPtr act = acts->get(j);
					if (act->GetRuntimeClass() == CLASS_ID( CActionRobotMovement2D ))
					{
						CActionRobotMovement2DPtr mov = CActionRobotMovement2DPtr( act );

						// Load an odometry estimation, but only if it is the only movement
						//  estimation source: any other may be a better one:
						if ( !poseIncrementLoaded || mov->estimationMethod!= CActionRobotMovement2D::emOdometry )
						{
							poseIncrementLoaded=true;
							mov->poseChange->getMean( poseIncrement );
							
							double dx=poseIncrement.x();
							double dy=poseIncrement.y();

							tools.log("actionMove x: "+std::to_string(dx)+" y: "+std::to_string(dy));
						}
					}
				}



				
				mapBuilder.processActionObservation( *acts, *observations );
				//mapBuilder.saveCurrentEstimationToImage("C:/Temp/estymate222.bmp",false);
				//mapBuilder.saveCurrentPathEstimationToTextFile("C:/Temp/estymate.txt");
				//mapBuilder.saveCurrentMapToFile("C:/Temp/map.txt",false);
	
				// Save the robot estimated pose for each step:
				CPose3D   meanPose;
				mapBuilder.getCurrentPoseEstimation()->getMean(meanPose);
				double ax=meanPose.x();
				double ay=meanPose.y();
				tools.log("buildXY x: "+std::to_string(ax)+" y: "+std::to_string(ay));	
	
				//OutputDebugStringW(L"My output string.");
				// Save map:
				mapBuilder.getCurrentlyBuiltMap(finalMap);
				int size=mapBuilder.getCurrentlyBuiltMapSize();
	
				// size=mapBuilder.getCurrentlyBuiltMapSize();
				// return;
			}
			break;
		case CRawlog::etSensoryFrame:
			{
				observations=rawLogTest.getAsObservations(i);
				
			}
			break;
		case CRawlog::etObservation:
			{

			}
			break;
		}; // end switch

	}
	
	
}
void CRobcioSLAM::testMapICP(string pathToRawLog,string pathToMainMap,vector<float> *mapX,vector<float>  *mapY,vector<double> *pathX,vector<double>  *pathY){
	CFileGZInputStream					rawlogFile( pathToRawLog.c_str() );
	CFileGZInputStream					rawlogMainFile( pathToMainMap.c_str() );
	//CRawlog rawLogTest;
//	rawLogTest.loadFromRawLogFile(pathToRawLog);
/*	CMetricMapBuilderICP mapBuilde;
	//mapBuilde.ICP_options.mapInitializers.options.
	CSimpleMap initialMap=createSimpleMapPointSLAM(pathToMainMap);
	
	mapBuilde.initialize(initialMap);
	

	int size=mapBuilde.getCurrentlyBuiltMapSize();
	*/
	//	CConfigFile				iniFile("C:/Robotics/mrpt-1.2.1/apps/RobcioBrain/gridMap/icp-slam_demo_classic.ini");
	CConfigFile				iniFile("C:/Robotics/mrpt-1.2.1/share/mrpt/config_files/icp-slam/icp-slam_demo_classic.ini");
		// ------------------------------------
	//		Constructor of ICP-SLAM object
	// ------------------------------------
	CMetricMapBuilderICP mapBuilder;
	const unsigned int rawlog_offset		 = iniFile.read_int("MappingApplication","rawlog_offset",0,  /*Force existence:*/ true);
	mapBuilder.ICP_options.loadFromConfigFile( iniFile, "MappingApplication");
	mapBuilder.ICP_params.loadFromConfigFile ( iniFile, "ICP");
	CSimpleMap initialMap=createSimpleMapPointSLAM(pathToMainMap);
	// Construct the maps with the loaded configuration.
	mapBuilder.initialize(initialMap);
	//mapBuilder.initialize();

	// ---------------------------------
	//   CMetricMapBuilder::TOptions
	// ---------------------------------
	mapBuilder.options.verbose = true;
    mapBuilder.options.alwaysInsertByClass.fromString( iniFile.read_string("MappingApplication","alwaysInsertByClass","") );



	//mapBuilder.saveCurrentEstimationToImage("C:/Temp/BuildICP2.bmp",true);

			CPose2D					odoPose(0,0,0);
		
				size_t						rawlogEntry = 0;
				mrpt::system::TTimeStamp	timeStampLast=NULL;
				mrpt::system::TTimeStamp	timeStampCorrentt=NULL;
			double sumX=0.0;
			double sumY=0.0;
			RobcioTools  tools;
			

for (;;)
	{
		CActionCollectionPtr	action;
		CSensoryFramePtr		observations;
		CObservationPtr			observation;

		if (os::kbhit())
		{
			char c = os::getch();
			if (c==27)
				break;
		}

		// Load action/observation pair from the rawlog:
		// --------------------------------------------------
		if (! CRawlog::getActionObservationPairOrObservation( rawlogFile, action, observations, observation, rawlogEntry) ){
			break; // file EOF
		}




		const bool isObsBasedRawlog = observation.present();
		std::vector<mrpt::slam::CObservation2DRangeScanPtr> lst_current_laser_scans;   // Just for drawing in 3D views

		if (rawlogEntry>=rawlog_offset)
		{
			// Update odometry:
			if (isObsBasedRawlog)
			{
				static CPose2D lastOdo;
				static bool firstOdo = true;
				if (IS_CLASS(observation,CObservationOdometry))
				{
					CObservationOdometryPtr o = CObservationOdometryPtr(observation);
					if (!firstOdo)
						odoPose = odoPose + (o->odometry - lastOdo);

					lastOdo=o->odometry;
					firstOdo=false;
				}
			}
			else
			{
				CActionRobotMovement2DPtr act= action->getBestMovementEstimation();
				if (act)
					odoPose = odoPose + act->poseChange->getMeanVal();
			}

			tools.log("x: "+std::to_string(odoPose.x())+" y: "+std::to_string(odoPose.y()));

			// Execute:
			// ----------------------------------------

			if (isObsBasedRawlog)
					mapBuilder.processObservation( observation );
			else	mapBuilder.processActionObservation( *action, *observations );

				CPose3D   meanPose;
				mapBuilder.getCurrentPoseEstimation()->getMean(meanPose);
				double ax=meanPose.x();
				double ay=meanPose.y();
				sumX+=ax;
				sumY+=ay;
				pathX->push_back(ax);
				pathY->push_back(ay);
				tools.log("estymate x: "+std::to_string(ax)+" estymate y: "+std::to_string(ay));
				/*
				if(pathX->size()>100){
				

					return;
				}*/
		}
}
/*
			//int ile=lx.size();
	CSensoryFramePtr observations;
			// Go: generate the map:
	size_t      i;
	CPose2D     curPose(0,0,0);
	RobcioTools  tools;
	for (i=0; i<rawLogTest.size();i++)
	{

		bool addNewPathEntry = false;
		
		switch( rawLogTest.getType(i) )
		{
		case CRawlog::etActionCollection:
			{
				CActionCollectionPtr    acts = rawLogTest.getAsAction(i);
				CPose2D                 poseIncrement;
				bool                    poseIncrementLoaded = false;

				for (size_t j=0;j<acts->size();j++)
				{
					CActionPtr act = acts->get(j);
					if (act->GetRuntimeClass() == CLASS_ID( CActionRobotMovement2D ))
					{
						CActionRobotMovement2DPtr mov = CActionRobotMovement2DPtr( act );

						// Load an odometry estimation, but only if it is the only movement
						//  estimation source: any other may be a better one:
						if ( !poseIncrementLoaded || mov->estimationMethod!= CActionRobotMovement2D::emOdometry )
						{
							poseIncrementLoaded=true;
							mov->poseChange->getMean( poseIncrement );
							
							double dx=poseIncrement.x();
							double dy=poseIncrement.y();

							tools.log("actionMove x: "+std::to_string(dx)+" y: "+std::to_string(dy));
						}
					}
				}



				
				mapBuilder.processActionObservation( *acts, *observations );
				//mapBuilder.saveCurrentEstimationToImage("C:/Temp/estymate222.bmp",false);
				//mapBuilder.saveCurrentPathEstimationToTextFile("C:/Temp/estymate.txt");
				//mapBuilder.saveCurrentMapToFile("C:/Temp/map.txt",false);
	
				// Save the robot estimated pose for each step:
				CPose3D   meanPose;
				mapBuilder.getCurrentPoseEstimation()->getMean(meanPose);
				double ax=meanPose.x();
				double ay=meanPose.y();
				pathX->push_back(ax);
				pathY->push_back(ay);

				if(pathX->size()>100){
				

					return;
				}
				//OutputDebugStringW(L"My output string.");
				// Save map:
			//	mapBuilder.getCurrentlyBuiltMap(finalMap);
				int size=mapBuilder.getCurrentlyBuiltMapSize();
				tools.log("buildXY x: "+std::to_string(ax)+" y: "+std::to_string(ay)+" size: "+std::to_string(size));	
				// size=mapBuilder.getCurrentlyBuiltMapSize();
				// return;
			}
			break;
		case CRawlog::etSensoryFrame:
			{
				observations=rawLogTest.getAsObservations(i);
				
			}
			break;
		case CRawlog::etObservation:
			{

			}
			break;
		}; // end switch

	}*/
	
		mapBuilder.getCurrentMapPoints(*mapX,*mapY);
	
		
		string str = format("C:/Temp/_finalmap_.simplemap");
		mapBuilder.saveCurrentMapToFile(str);

			CMultiMetricMap  *finalPointsMap = mapBuilder.getCurrentlyBuiltMetricMap();
	str = format("C:/Temp/_finalmaps_.txt");
	printf("Dumping final metric maps to %s_XXX\n", str.c_str() );
	finalPointsMap->saveMetricMapRepresentationToFile( str );
}
CPointsMapPtr CRobcioSLAM::loadMap(string path) { 

	CPointsMapPtr mapToLoad;
	CFileInputStream (path) >> mapToLoad;

	return mapToLoad; 
	//mapToSave->saveMetricMapRepresentationToFile("C:\\Temp\\testSaveRavlog.rawlog");

}
void CRobcioSLAM::saveMap(CPointsMapPtr mapToSave,string path) { 


	CFileOutputStream (path) << mapToSave;

}
CSimpleMap CRobcioSLAM::loadSimpleMap(string path) { 

	CSimpleMap mapToLoad;
	CFileInputStream (path) >> mapToLoad;
	return mapToLoad; 
}
void CRobcioSLAM::saveSimpleMap(CSimpleMap mapToSave,string path) { 
	CFileOutputStream (path) << mapToSave;
}
	
CMultiMetricMap	CRobcioSLAM::createMetricMap(string pathToMainMap){
			CPose2D posytionRobot;
			CRawlog rawLog;
			rawLog.loadFromRawLogFile(pathToMainMap);
			CConfigFile	iniFile("C:/Robotics/mrpt-1.2.1/apps/RobcioBrain/gridMap/localization_demo.ini");
			CMultiMetricMap metricMap;
			TSetOfMetricMapInitializers				mapList;
			mapList.loadFromConfigFile( iniFile,"MetricMap");
			metricMap.setListOfMaps( &mapList );
			size_t i;
			for (i=0; i<rawLog.size();i++)
	{
		CActionCollectionPtr action;
	CSensoryFramePtr observations;
	
		bool addNewPathEntry = false;
		
		switch( rawLog.getType(i) )
		{
		case CRawlog::etActionCollection:
			{
				CPose2D newPosytionXY;
				CActionCollectionPtr    action = rawLog.getAsAction(i);
				CActionRobotMovement2DPtr move=action->getBestMovementEstimation();
				move->poseChange->getMean(newPosytionXY);
				double x=newPosytionXY.x();
				double y=newPosytionXY.y();
				posytionRobot=posytionRobot+newPosytionXY;
			}
			break;
		case CRawlog::etSensoryFrame:
			{
				
				observations=rawLog.getAsObservations(i);
				CObservationPtr ptr=(observations->getObservationByIndex(0));
				
				metricMap.insertObservation(ptr.pointer(),new CPose3D(posytionRobot));
				
			}
			break;
		case CRawlog::etObservation:
			{

			}
			break;
		}; // end switch


	}

			metricMap.getAsSimplePointsMap()->save2D_to_text_file("C:/Temp/savePoints.txt");
			metricMap.m_gridMaps[0]->saveAsBitmapFile("C:/Temp/imageTest.bmp");
		//	rawLog=NULL;
			return metricMap;

}
void CRobcioSLAM:: testOtherSLAM(string pathToPartMap,string pathToMainMap,vector<double> *poseX,vector<double> *poseY,vector<float> *poseMapX,vector<float> *poseMapY){
	CMultiMetricMap	metricMainMap=createMetricMap(pathToMainMap);
	CRawlog rawLogTest;
	rawLogTest.loadFromRawLogFile(pathToPartMap);
	
	CPointsMapPtr mainMap=loadMapFromRawLog(pathToMainMap);	
		CConfigFile	iniFile("C:/Robotics/mrpt-1.2.1/apps/RobcioBrain/gridMap/localization_demo.ini");
	//

	// Load configuration:
	// -----------------------------------------
	string iniSectionName ( "LocalizationExperiment" );


	// Mandatory entries:
	string		OUT_DIR_PREFIX		= iniFile.read_string(iniSectionName,"logOutput_dir","",true );


	string		RAWLOG_FILE;


			// Non-mandatory entries:
			string		MAP_FILE			= iniFile.read_string(iniSectionName,"map_file","" );
			size_t		rawlog_offset		= iniFile.read_int(iniSectionName,"rawlog_offset",0);
			string		GT_FILE				= iniFile.read_string(iniSectionName,"ground_truth_path_file","");
			int		NUM_REPS			= iniFile.read_int(iniSectionName,"experimentRepetitions",1);
			int		SCENE3D_FREQ		= iniFile.read_int(iniSectionName,"3DSceneFrequency",10);
			bool 		SCENE3D_FOLLOW = iniFile.read_bool(iniSectionName,"3DSceneFollowRobot",true);
			unsigned int	testConvergenceAt   = iniFile.read_int(iniSectionName,"experimentTestConvergenceAtStep",-1);

			bool    	SAVE_STATS_ONLY = iniFile.read_bool(iniSectionName,"SAVE_STATS_ONLY",false);

			bool 		SHOW_PROGRESS_3D_REAL_TIME = iniFile.read_bool(iniSectionName,"SHOW_PROGRESS_3D_REAL_TIME",false);
			int			SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS = iniFile.read_int(iniSectionName,"SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS",1);
			double 		STATS_CONF_INTERVAL = iniFile.read_double(iniSectionName,"STATS_CONF_INTERVAL",0.2);


				// PF-algorithm Options:
			// ---------------------------
			CParticleFilter::TParticleFilterOptions		pfOptions;
			pfOptions.loadFromConfigFile( iniFile, "PF_options" );

			// PDF Options:
			// ------------------
			TMonteCarloLocalizationParams	pdfPredictionOptions;
			pdfPredictionOptions.KLD_params.loadFromConfigFile( iniFile, "KLD_options");

			// Metric map options:
			// -----------------------------
			TSetOfMetricMapInitializers				mapList;
			mapList.loadFromConfigFile( iniFile,"MetricMap");
			
			cout<< "-------------------------------------------------------------\n"
				<< "\t RAWLOG_FILE = \t "   << RAWLOG_FILE << endl
				<< "\t MAP_FILE = \t "      << MAP_FILE << endl
				<< "\t GT_FILE = \t "       << GT_FILE << endl
				<< "\t OUT_DIR_PREFIX = \t "<< OUT_DIR_PREFIX << endl
				<< "\t #particles = \t "    << 5 << endl
				<< "-------------------------------------------------------------\n";
			pfOptions.dumpToConsole();
			mapList.dumpToConsole();

			// --------------------------------------------------------------------
			//						EXPERIMENT PREPARATION
			// --------------------------------------------------------------------
		
			
			CParticleFilter::TParticleFilterStats	PF_stats;



			 size_t m_particles=1;
			CMonteCarloLocalization2D  pdf(m_particles);

			// PDF Options:
			pdf.options = pdfPredictionOptions;

			pdf.options.metricMap = &metricMainMap;


		//	CSimplePointsMap *point=metricMap.getAsSimplePointsMap();
			vector<float> xs;
			 vector<float> ys;
	
			 
			//point->getAllPoints(
			// Create the PF object:
			CParticleFilter	PF;
			PF.m_options = pfOptions;


			if ( !iniFile.read_bool(iniSectionName,"init_PDF_mode",false, true) )
				pdf.resetUniformFreeSpace(
					metricMainMap.m_gridMaps[0].pointer(),
					0.3f,
					m_particles ,
					iniFile.read_float(iniSectionName,"init_PDF_min_x",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_max_x",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_min_y",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_max_y",0,true),
					DEG2RAD(iniFile.read_float(iniSectionName,"init_PDF_min_phi_deg",0)),
					DEG2RAD(iniFile.read_float(iniSectionName,"init_PDF_max_phi_deg",120))
					);
			else
				pdf.resetUniform(
					iniFile.read_float(iniSectionName,"init_PDF_min_x",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_max_x",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_min_y",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_max_y",0,true),
					DEG2RAD(iniFile.read_float(iniSectionName,"init_PDF_min_phi_deg",-180)),
					DEG2RAD(iniFile.read_float(iniSectionName,"init_PDF_max_phi_deg",180)),
					-1
					);
		
			CPose2D				pdfEstimation, odometryEstimation;
			CMatrixDouble		cov;
			RobcioTools  tools;
			size_t i;
	for (i=1500; i<rawLogTest.size();i++)
	{
		
		CActionCollectionPtr action;
	CSensoryFramePtr observations;
	
		bool addNewPathEntry = false;
		
		switch( rawLogTest.getType(i) )
		{
		case CRawlog::etActionCollection:
			{
				CActionCollectionPtr    action = rawLogTest.getAsAction(i);
				CPose2D                 poseIncrement;
				bool                    poseIncrementLoaded = false;

				

									// Map estimated by the SLAM filter ----------------------
						{
							CPosePDFGaussian estRobotPose;
							vector<TPoint2D>  LMs;
							map<unsigned int,CLandmark::TLandmarkID> landmarkIDs;

							CVectorDouble     Xkk;  // Full mean & cov
							CMatrixDouble     Pkk;

							
						PF.executeOn(
							pdf,
							action.pointer(),			// Action
							observations.pointer(),	// Obs.
							&PF_stats		// Output statistics
							);
				
						CPose3D poseTheBestFirst;
						action.pointer()->getFirstMovementEstimationMean(poseTheBestFirst);
						pdf.getMean( pdfEstimation );
							//tools.log("Estimated map (landmarks)"+std::to_string(LMs.size())+" len "+std::to_string(landmarkIDs.size()));
						double ssx=poseTheBestFirst.x();
						double ssy=poseTheBestFirst.y();
						poseX->push_back(pdfEstimation.x()/10.0f);
						poseY->push_back(pdfEstimation.y()/10.0f);

						if(poseY->size()>10000){
						
						}
							// mean robot pose:
							//tools.log("estRobotPose.mean.x(): "+std::to_string(pdfEstimation.x())+" "+" estRobotPose.mean.x(): "+std::to_string(pdfEstimation.y()));
							
							
						
					//	 pdf.getMostLikelyParticle();
						}



	
				// size=mapBuilder.getCurrentlyBuiltMapSize();
				// return;
			}
			break;
		case CRawlog::etSensoryFrame:
			{
				
				observations=rawLogTest.getAsObservations(i);
			
				
			}
			break;
		case CRawlog::etObservation:
			{

			}
			break;
		}; // end switch


	}
	CSimplePointsMapPtr	poinmap=metricMainMap.m_pointsMaps[0];
						//	CSimplePointsMap *poinmap= metricMainMap.m_gridMaps[0]->getAsSimplePointsMap();
							poinmap->getAllPoints(*poseMapX,*poseMapY);
					
				

};

CSimpleMap CRobcioSLAM::createSimpleMapPointSLAM(string pathToRawLog) { 
	
	CRawlog rawLogTest;
	rawLogTest.loadFromRawLogFile(pathToRawLog);
	CSimpleMap simpleMap;
	
	
	CPose2D     curPose(0,0,0);
	size_t      i;
	
	CSensoryFramePtr observations;
	CPosePDFGaussianPtr posePDFGaussian;
	for (i=0; i<rawLogTest.size()-1;i++)
	{
		bool addNewPathEntry = false;


		switch( rawLogTest.getType(i) )
		{
		case CRawlog::etActionCollection:
			{
				CActionCollectionPtr    acts = rawLogTest.getAsAction(i);
				CPose2D                 poseIncrement;
				bool                    poseIncrementLoaded = false;

				for (size_t j=0;j<acts->size();j++)
				{
					CActionPtr act = acts->get(j);
					if (act->GetRuntimeClass() == CLASS_ID( CActionRobotMovement2D ))
					{
						CActionRobotMovement2DPtr mov = CActionRobotMovement2DPtr( act );

						// Load an odometry estimation, but only if it is the only movement
						//  estimation source: any other may be a better one:
						if ( !poseIncrementLoaded || mov->estimationMethod!= CActionRobotMovement2D::emOdometry )
						{
							poseIncrementLoaded=true;
							mov->poseChange->getMean( poseIncrement );
						}
					}
				}
				curPose = curPose + poseIncrement; 
				posePDFGaussian = CPosePDFGaussian::Create();
				posePDFGaussian->mean = curPose;
				simpleMap.insert(posePDFGaussian,observations);
			}
			break;
		case CRawlog::etSensoryFrame:
			{
				

			observations=rawLogTest.getAsObservations(i);
			
				
			}
			break;
		case CRawlog::etObservation:
			{

			}
			break;
		}; // end switch

		

	}

	return simpleMap;

}
CPointsMapPtr CRobcioSLAM::testSimpleMap(string pathToRawLog,string pathToFile){
		//	CSimpleMap simpleMap= createSimpleMapPointSLAM(pathToRawLog);
		//	saveSimpleMap(simpleMap,pathToFile);
			CSimpleMap	simpleMap=loadSimpleMap(pathToFile) ;
			CPointsMapPtr pointMap=createEmptyMap();
			pointMap->loadFromSimpleMap(simpleMap);
			return pointMap;
}

void CRobcioSLAM::testSLAM(string pathToRawLog,string pathToMainMap,CSimpleMap &finalMap) { 

	CActionCollectionPtr action;
	CSensoryFramePtr observations;
	
	//CPointsMapPtr mapMain=loadMap(pathToMainMap);
	CRawlog rawLogTest;
	size_t      decimate =1;
//	CSimpleMap initialMap=(*mapMain).getAsSimplePointsMap();
	rawLogTest.loadFromRawLogFile(pathToRawLog);


	CSimpleMap	simpleMap=loadSimpleMap(pathToMainMap) ;
	// Go: generate the map:
	size_t      i;
	CPose2D     curPose(0,0,0);
	for (i=0; i<rawLogTest.size()-1;i++)
	{
		bool addNewPathEntry = false;

		switch( rawLogTest.getType(i) )
		{
		case CRawlog::etActionCollection:
			{
				action = rawLogTest.getAsAction(i);
				
				//testMapBuilderRBPF(action, observations,simpleMap,finalMap);
			}
			break;
		case CRawlog::etSensoryFrame:
			{
				observations=rawLogTest.getAsObservations(i);
				
			}
			break;
		case CRawlog::etObservation:
			{

			}
			break;
		}; // end switch

	}
	

}
CPointsMapPtr CRobcioSLAM::loadMapFromRawLog(string pathToRawLog) { 



	CRawlog rawLogTest;
	size_t      decimate =1;

	rawLogTest.loadFromRawLogFile(pathToRawLog);

	rawLogTest.size();

	// Go: generate the map:
	size_t      i;
	CPose2D     curPose(0,0,0);
	TTimeStamp	last_tim = INVALID_TIMESTAMP;
	CMultiMetricMap			theMap;


	ifstream inputConfig( "c:\\Temp\\map.ini" );
	std::string configMapStr;
	for( std::string line; getline( inputConfig, line ); )
	{
		configMapStr+=line+"\n";

	}




	// Create a memory "ini file" with the text in the window:
	CConfigFileMemory       configSrc( CStringList( std::string(configMapStr.c_str()) ) );

	TSetOfMetricMapInitializers		lstMaps;
	lstMaps.loadFromConfigFile( configSrc, "map" );
	theMap.setListOfMaps( &lstMaps );

	CPointsMapPtr	thePntsMap;

	if( !theMap.m_pointsMaps.empty() )
		thePntsMap = theMap.m_pointsMaps[0];
	else if (theMap.m_colourPointsMap.present())
		thePntsMap = theMap.m_colourPointsMap;

	if (thePntsMap) {
		thePntsMap->reserve( (rawLogTest.size()+1)*800 );


	}

	for (i=0; i<rawLogTest.size()-1;i++)
	{
		bool addNewPathEntry = false;

		switch( rawLogTest.getType(i) )
		{
		case CRawlog::etActionCollection:
			{
				CActionCollectionPtr    acts = rawLogTest.getAsAction(i);
				CPose2D                 poseIncrement;
				bool                    poseIncrementLoaded = false;

				for (size_t j=0;j<acts->size();j++)
				{
					CActionPtr act = acts->get(j);
					if (act->GetRuntimeClass() == CLASS_ID( CActionRobotMovement2D ))
					{
						CActionRobotMovement2DPtr mov = CActionRobotMovement2DPtr( act );

						// Load an odometry estimation, but only if it is the only movement
						//  estimation source: any other may be a better one:
						if ( !poseIncrementLoaded || mov->estimationMethod!= CActionRobotMovement2D::emOdometry )
						{
							poseIncrementLoaded=true;
							mov->poseChange->getMean( poseIncrement );
						}
					}
				}



				curPose = curPose + poseIncrement;
				addNewPathEntry=true;
			}
			break;
		case CRawlog::etSensoryFrame:
			{
				if (( (i>>1) % decimate)==0)
				{
					CPose3D		dumPose(curPose);

					rawLogTest.getAsObservations(i)->insertObservationsInto( &theMap, &dumPose );
				}
				addNewPathEntry=true;
			}
			break;
		case CRawlog::etObservation:
			{
				// Always, process odometry:
				const CObservation* obs = rawLogTest.getAsObservation(i).pointer();
				if (IS_CLASS(obs,CObservationOdometry))
				{
					const CObservationOdometry* obsOdo = static_cast<const CObservationOdometry*>(obs);
					curPose = obsOdo->odometry;
				}

				if (( (i>>1) % decimate)==0)
				{
					CPose3D		dumPose(curPose);
					theMap.insertObservation( rawLogTest.getAsObservation(i).pointer(), &dumPose );
					last_tim = rawLogTest.getAsObservation(i)->timestamp;
				}
				addNewPathEntry=true;
			}
			break;
		}; // end switch

	}


	std::cout<<"sss"; 
	return thePntsMap;

}
void CRobcioSLAM::initFileGridExport(){
	lastX=0;
	lastY=0;
	isStart=true;
	compasRadius=0;
	dystance=0;
	odoLast=CPose2D(0,0,0);
	string rawlog_filename="C:\\Temp\\my_dataset4_"+this->currentDateTime()+".rawlog";
	int 			rawlog_GZ_compress_level  = 1; 
	out_file.open( rawlog_filename, rawlog_GZ_compress_level );

};
void CRobcioSLAM::closeFileGridExport(){


	out_file.close();

};
void CRobcioSLAM::importFromCSVFile(CPointsMapPtr gridmapLoad){

	printf("\n importFromCSVFile \n");
	lendmarkId=0;
	ifstream input( "C:/Robotics/mrpt-1.2.1/apps/RobcioBrain/gridMap/PartSmallMoved.csv" );		
	for( std::string line; getline( input, line ); )
	{
		readDataScanFromString(line,gridmapLoad);
	}
};

CRawlog CRobcioSLAM::createRawlog(){

	/*
	printf("\n importFromCSVFile \n");



	ifstream input( "c:\\Temp\\logRecor_2014-08-10 22_28_01_.csv" );
	initFileGridExport();
	for( std::string line; getline( input, line ); )
	{

	readDataScanFromCSV(line);
	}
	return rawLog*/
	return rawLog;

};

void CRobcioSLAM::importFromCSV(std::string csvFile){
	mrpt::system::TTimeStamp	timeStampLast=NULL;
	mrpt::system::TTimeStamp	timeStampCorrentt=NULL;
	
	lastX=0;
	lastY=0;
	isStart=true;
	compasRadius=0;
	dystance=0;
	CPose2D odoLastPoint=CPose2D(0,0,0);
	string rawlog_filename="C:\\Temp\\my_dataset4_"+this->currentDateTime()+".rawlog";
	int 			rawlog_GZ_compress_level  = 1; 
	CFileGZOutputStream	out_file;
	out_file.open( rawlog_filename, rawlog_GZ_compress_level );


	ifstream input( csvFile);
	
		TTimeStamp time=mrpt::system::now();
		long i=0;
	for( std::string line; getline( input, line ); )
	{

		i++;
	
	//printf("\n readDataScanFromCSV \n");
	CPose2D odometryIncrements;
	CPose2D odometryIncrementsTest;
	vector<double> changeStatusArrayData=parseLineLog(line);
	string actionStep=parseLineGetStat(line); 

	double newCompasRadius=changeStatusArrayData[2];
	double newDystance=changeStatusArrayData[1];

	double overAll=360;
	double radiusFromCompas=overAll*(newCompasRadius);
	
	double phiCompasRadius=DEG2RAD(radiusFromCompas);
//	printf("phiCompasRadius: %f , radiusFromCompas: %f  \n",phiCompasRadius,radiusFromCompas);
	

	vector<double> arrayXY=getCordinate((newDystance), DEG2RAD(overAll*(newCompasRadius)));
	compasRadius=newCompasRadius;

	//isStart=false;
	//actionStepLast="Left";
	//actionStep="Left";
	CObservationOdometry cob;
	cob.sensorLabel="RobcioSensor";
	if(isStart){

		
		odometryIncrements=CPose2D(0,0,phiCompasRadius);		
		
		isStart=false;

	}else if(actionStep=="Forward" ){
	//}else if(actionStep=="Forward" && (actionStepLast=="Forward" || actionStepLast=="Back" || actionStepLast=="Stop")){
		lastX+=arrayXY[0];
		lastY+=arrayXY[1];
		odometryIncrementsTest=CPose2D(arrayXY[0],arrayXY[1],phiCompasRadius);					

		odometryIncrements=CPose2D(lastX,lastY,phiCompasRadius);					
	//}else if(actionStep=="Back" && (actionStepLast=="Back" || actionStepLast=="Forward" || actionStepLast=="Stop")){
	}else if(actionStep=="Back" ){
		lastX+=arrayXY[0];
		lastY+=arrayXY[1];
		odometryIncrementsTest=CPose2D(arrayXY[0],arrayXY[1],phiCompasRadius);	
		odometryIncrements=CPose2D(lastX,lastY,phiCompasRadius);	
	}else if(actionStep=="Left" && actionStepLast=="Left"){
		odometryIncrementsTest=CPose2D(0,0,phiCompasRadius);	
		odometryIncrements=CPose2D(lastX,lastY,phiCompasRadius);
	}else if(actionStep=="Right" && actionStepLast=="Right"){
		odometryIncrementsTest=CPose2D(0,0,phiCompasRadius);	
		odometryIncrements=CPose2D(lastX,lastY,phiCompasRadius);
	}else{
		odometryIncrementsTest=CPose2D(0,0,phiCompasRadius);	
		odometryIncrements=CPose2D(lastX,lastY,phiCompasRadius);
		
	
	}
	actionStepLast=actionStep;

	
	
	cob.odometry=odometryIncrements;		
	
	//printf("X: %f , Y: %f cord: %s \n",arrayXY[0],arrayXY[1],cob.odometry.asString().c_str());
	cob.timestamp=mrpt::system::time_tToTimestamp(changeStatusArrayData[0]);
	CObservationOdometryPtr odom = CObservationOdometryPtr(new CObservationOdometry(cob));

	
	//printf(" cord: %s %s %i \n",odometryIncrements.asString().c_str(),actionStep.c_str(),radiusFromCompas);

	/*
	*The scan
	*/

	
	CObservation2DRangeScanPtr the_scan = CObservation2DRangeScan::Create();
	the_scan->rightToLeft=false;
	the_scan->aperture = DEG2RAD(120);
	the_scan->timestamp = time+secondsToTimestamp( i);
	//the_scan->sensorPose=cob.odometry; // this is not workin I get wrong map
	the_scan->sensorLabel="RobcioSensor";
	the_scan->maxRange=2.50f;
	for(int i=7;i<changeStatusArrayData.size();i++){
		if(changeStatusArrayData[i]>the_scan->maxRange){

			the_scan->scan.push_back(changeStatusArrayData[i]);
			the_scan->validRange.push_back(0);
		}else{
			the_scan->scan.push_back(changeStatusArrayData[i]);
			the_scan->validRange.push_back(1);
		}

	}
	
	
	

	CActionCollection    acts;
	//CActionRobotMovement2D move;

	
	CActionRobotMovement2D	act;
	CActionRobotMovement2D::TMotionModelOptions	opts;
	opts.modelSelection = CActionRobotMovement2D::mmGaussian;

	CPose2D  Aodom =cob.odometry- odoLastPoint;
					
	act.computeFromOdometry(Aodom, opts);
	



	odoLastPoint=cob.odometry;
	CSensoryFrame frame;
	

	
	
		
	
		cob.timestamp=time+secondsToTimestamp( i);;
		act.timestamp = time+secondsToTimestamp( i);;
		acts.insert(act);
			frame.insert(the_scan);
			frame.getObservationByIndex(0)->timestamp=time+secondsToTimestamp( i);
	out_file << frame;
	out_file <<acts;
	//out_file << odom;
	}
	out_file.close();
};
void CRobcioSLAM::readDataScanFromString(std::string line,CPointsMapPtr gridmapLoad){

	//printf("\n readDataScanFromCSV \n");
	CPose2D odometryIncrements;
	CPose2D odometryIncrementsTest;
	vector<double> changeStatusArrayData=parseLineLog(line);
	string actionStep=parseLineGetStat(line); 

	double newCompasRadius=changeStatusArrayData[2];
	double newDystance=changeStatusArrayData[1];

	double overAll=360;
	double radiusFromCompas=overAll*(newCompasRadius);

	double phiCompasRadius=DEG2RAD(radiusFromCompas);
	//	printf("phiCompasRadius: %f , radiusFromCompas: %f  \n",phiCompasRadius,radiusFromCompas);


	vector<double> arrayXY=getCordinate((newDystance), DEG2RAD(overAll*(newCompasRadius)));
	compasRadius=newCompasRadius;

	//isStart=false;
	//actionStepLast="Left";
	//actionStep="Left";
	CObservationOdometry cob;
	cob.sensorLabel="RobcioSensor";
	if(isStart){


		odometryIncrements=CPose2D(0,0,phiCompasRadius);		

		isStart=false;

	}else if(actionStep=="Forward" ){
		//}else if(actionStep=="Forward" && (actionStepLast=="Forward" || actionStepLast=="Back" || actionStepLast=="Stop")){
		lastX+=arrayXY[0];
		lastY+=arrayXY[1];
		odometryIncrementsTest=CPose2D(arrayXY[0],arrayXY[1],phiCompasRadius);					

		odometryIncrements=CPose2D(lastX,lastY,phiCompasRadius);					
		//}else if(actionStep=="Back" && (actionStepLast=="Back" || actionStepLast=="Forward" || actionStepLast=="Stop")){
	}else if(actionStep=="Back" ){
		lastX+=arrayXY[0];
		lastY+=arrayXY[1];
		odometryIncrementsTest=CPose2D(arrayXY[0],arrayXY[1],phiCompasRadius);	
		odometryIncrements=CPose2D(lastX,lastY,phiCompasRadius);	
	}else if(actionStep=="Left" && actionStepLast=="Left"){
		odometryIncrementsTest=CPose2D(0,0,phiCompasRadius);	
		odometryIncrements=CPose2D(lastX,lastY,phiCompasRadius);
	}else if(actionStep=="Right" && actionStepLast=="Right"){
		odometryIncrementsTest=CPose2D(0,0,phiCompasRadius);	
		odometryIncrements=CPose2D(lastX,lastY,phiCompasRadius);
	}else{
		odometryIncrementsTest=CPose2D(0,0,phiCompasRadius);	
		odometryIncrements=CPose2D(lastX,lastY,phiCompasRadius);


	}
	actionStepLast=actionStep;



	cob.odometry=odometryIncrements;		

	//printf("X: %f , Y: %f cord: %s \n",arrayXY[0],arrayXY[1],cob.odometry.asString().c_str());
	cob.timestamp=mrpt::system::time_tToTimestamp(changeStatusArrayData[0]);
	CObservationOdometryPtr odom = CObservationOdometryPtr(new CObservationOdometry(cob));


	//printf(" cord: %s %s %i \n",odometryIncrements.asString().c_str(),actionStep.c_str(),radiusFromCompas);

	/*
	*The scan
	*/

	    CObservationBearingRange	m_lastObservation;
		CObservationBearingRange obsRange;
		obsRange.maxSensorDistance=2.5;
		obsRange.timestamp = mrpt::system::time_tToTimestamp(changeStatusArrayData[0]);
		
				//obs.insertObservationInto(

	CObservation2DRangeScanPtr the_scan = CObservation2DRangeScan::Create();
	the_scan->rightToLeft=false;
	the_scan->aperture = DEG2RAD(120);
	the_scan->timestamp = mrpt::system::time_tToTimestamp(changeStatusArrayData[0]);
	//the_scan->sensorPose=cob.odometry; // this is not workin I get wrong map
	the_scan->sensorLabel="RobcioSensor";
	the_scan->maxRange=2.50f;

	vector<CObservationBearingRange::TMeasurement> dataM;
	for(int i=7;i<changeStatusArrayData.size();i++){
		
		CObservationBearingRange::TMeasurement messs;
		messs.landmarkID=lendmarkId;
		messs.pitch=0;
		messs.range=changeStatusArrayData[i];
		messs.yaw=DEG2RAD(90.0F-(30.0f +((i-7.0f))/2.0f));
		//messs.yaw=DEG2RAD(((60.0F+changeStatusArrayData.size()-(i-7.0f))/2.0f));
		m_lastObservation.sensedData.push_back(messs);

		lendmarkId++;
		//	m_lastObservation.sensedData[i].range=changeStatusArrayData[i];
		//    m_lastObservation.sensedData[i].yaw=DEG2RAD((i-7)/2);

		if(changeStatusArrayData[i]>the_scan->maxRange){
			
			the_scan->scan.push_back(changeStatusArrayData[i]);
			the_scan->validRange.push_back(0);
		}else{
			the_scan->scan.push_back(changeStatusArrayData[i]);
			the_scan->validRange.push_back(1);
		}

	}




	CActionCollection    acts;
	//CActionRobotMovement2D move;


	CActionRobotMovement2D	act;
	CActionRobotMovement2D::TMotionModelOptions	opts;
	opts.modelSelection = CActionRobotMovement2D::mmGaussian;

	CPose2D  Aodom =cob.odometry- odoLast;

	act.computeFromOdometry(Aodom, opts);
	act.timestamp = cob.timestamp;

	acts.insert(act);

	odoLast=cob.odometry;
	CSensoryFrame frame;

	frame.insert(the_scan);
	CObservationBearingRange *rang= new CObservationBearingRange(m_lastObservation);
	rang->timestamp= cob.timestamp;
	frame.insert( CObservationBearingRangePtr(rang ));
	//frame.insert( CObservationBearingRangePtr( new CObservationBearingRange(m_lastObservation) ));
	CMetricMap *refMap =  (CMetricMap*)&gridmapLoad;
	CPose3D    pose3D=odometryIncrements;
	frame.insertObservationsInto(gridmapLoad,&pose3D);


	gridmapLoad->insertObservation(new CObservationOdometry(cob));
	out_file << frame;
	out_file <<acts;
	//out_file << odom;




};

void CRobcioSLAM::alignICP(CSimplePointsMap		*mainMap,CSimplePointsMap *scanedMap,float thresholdDist){



	float					runningTime;
	CICP::TReturnInfo		info;
	CICP					ICP;

	int  ICP_method = (int) icpClassic;
	// -----------------------------------------------------
	ICP.options.ICP_algorithm = icpLevenbergMarquardt;
	//ICP.options.ICP_algorithm = icpClassic;
	//ICP.options.ICP_algorithm = (TICPAlgorithm)ICP_method;

	ICP.options.maxIterations			= 200;
	ICP.options.onlyClosestCorrespondences=true;

	ICP.options.thresholdAng			= DEG2RAD(10.0f);

	ICP.options.thresholdDist			= thresholdDist;
	ICP.options.ALFA					= 0.9f;
	ICP.options.smallestThresholdDist	= 0.001;
	ICP.options.doRANSAC = false;

	ICP.options.dumpToConsole();


	
	



	// -----------------------------------------------------
	//CPose2D		transfer(1.2f,0.5f,(float)DEG2RAD(0.0f));
	CPose2D		initialPose(0.0f,0.0f,(float)DEG2RAD(0.0f));



	CPosePDFPtr pdf = ICP.Align(
		scanedMap,
		mainMap,
		initialPose,
		&runningTime,
		(void*)&info);
	printf("ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%% goodness\n -> ",
		runningTime*1000,
		info.nIterations,
		runningTime*1000.0f/info.nIterations,
		info.goodness*100 );
	cout << "Mean of estimation: " << pdf->getMeanVal() << endl<< endl;
	CPosePDFGaussian  gPdf;
	gPdf.copyFrom(*pdf);



	cout << "Covariance of estimation: " << endl << gPdf.cov << endl;

	cout << " std(x): " << sqrt( gPdf.cov(0,0) ) << endl;
	cout << " std(y): " << sqrt( gPdf.cov(1,1) ) << endl;
	cout << " std(phi): " << RAD2DEG(sqrt( gPdf.cov(2,2) )) << " (deg)" << endl;

	//cout << "Covariance of estimation (MATLAB format): " << endl << gPdf.cov.inMatlabFormat()  << endl;

	double x=sqrt( gPdf.cov(0,0));
	double y=sqrt( gPdf.cov(1,1) );
	double rad= RAD2DEG(sqrt( gPdf.cov(2,2) ));
	cout << "-> Saving transformed map to align as scan2_trans.txt" << endl;

	double dx=gPdf.mean.x();
	double dy=gPdf.mean.y();
	//CPose2D		transfer(1.2f,0.5f,(float)DEG2RAD(0.0f));
	CPose2D test0(0.0f,0.0f,(float)DEG2RAD(0.0f));
	CPose2D tran=test0 - gPdf.mean;
	(*scanedMap).changeCoordinatesReference(tran);


	//(*scanedMap).

	//CPose2D		transfer(1.2f,0.5f,(float)DEG2RAD(0.0f));
	//(*scanedMap).changeCoordinatesReference(transfer);



};
