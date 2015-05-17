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
#include <mrpt/math/geometry.h>
#include <mrpt/topography.h>

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
void CRobcioSLAM::testMapBuilderRBPF(CActionCollectionPtr action,CSensoryFramePtr observations,CSimpleMap &initialMap){
	// ---------------------------------
	//		MapPDF opts
	// ---------------------------------
	CMetricMapBuilderRBPF::TConstructionOptions		rbpfMappingOptions;
	string INI_FILENAME="C:\mrpt-1.0.2-new\mrpt-1.0.2\share\mrpt\config_files\rbpf-slam\gridmapping_RBPF_ICPbased_intel.ini";
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

	mapBuilder.initialize(initialMap);
	mapBuilder.processActionObservation( *action, *observations );
	mapBuilder.saveCurrentEstimationToImage("C:/Temp/estymate.bmp");
	mapBuilder.saveCurrentPathEstimationToTextFile("C:/Temp/estymate.txt");

	// Save the robot estimated pose for each step:
	CPose3D   meanPose;
	mapBuilder.getCurrentPoseEstimation()->getMean(meanPose);

	CSimpleMap				finalMap;
	// Save map:
	mapBuilder.getCurrentlyBuiltMap(finalMap);


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
CSimpleMap CRobcioSLAM::createSimpleMapPointSLAM(string pathToRawLog) { 
	
	CRawlog rawLogTest;
	CSimpleMap simpleMap;
	rawLogTest.loadFromRawLogFile(pathToRawLog);

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
			CSimpleMap simpleMap= createSimpleMapPointSLAM(pathToRawLog);
			saveSimpleMap(simpleMap,pathToFile);
			simpleMap=loadSimpleMap(pathToFile) ;
			CPointsMapPtr pointMap=createEmptyMap();
			pointMap->loadFromSimpleMap(simpleMap);
			return pointMap;
}

void CRobcioSLAM::testSLAM(string pathToRawLog,string pathToMainMap) { 

	CActionCollectionPtr action;
	CSensoryFramePtr observations;
	
	CPointsMapPtr mapMain=loadMap(pathToMainMap);
	CRawlog rawLogTest;
	size_t      decimate =1;
//	CSimpleMap initialMap=(*mapMain).getAsSimplePointsMap();
	rawLogTest.loadFromRawLogFile(pathToRawLog);


	
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
				CActionCollectionPtr    acts = rawLogTest.getAsAction(i);
				

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

	ifstream input( "c:\\Temp\\logRecor_2015-05-04 20_56_58_.csv" );		
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


	CObservation2DRangeScanPtr the_scan = CObservation2DRangeScan::Create();
	the_scan->rightToLeft=false;
	the_scan->aperture = DEG2RAD(120);
	the_scan->timestamp = mrpt::system::time_tToTimestamp(changeStatusArrayData[0]);
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

	CPose2D  Aodom =cob.odometry- odoLast;

	act.computeFromOdometry(Aodom, opts);
	act.timestamp = cob.timestamp;

	acts.insert(act);

	odoLast=cob.odometry;
	CSensoryFrame frame;

	frame.insert(the_scan);

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
