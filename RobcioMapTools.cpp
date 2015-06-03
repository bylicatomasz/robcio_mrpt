#include <mrpt/slam/CMonteCarloLocalization2D.h>

#include <mrpt/utils/CConfigFile.h>
#include <mrpt/math/ops_vectors.h> // << for vector<>
#include <mrpt/system/filesystem.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/random.h>

#include <mrpt/slam/CActionRobotMovement2D.h>
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/slam/CRawlog.h>
#include <mrpt/slam/CSimpleMap.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/slam/CObservationOdometry.h>

#include <mrpt/system/os.h>
#include <mrpt/system/threads.h> // sleep()
#include <mrpt/system/vector_loadsave.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/utils.h>
#include <mrpt/utils/CTicTac.h>


#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CFileInputStream.h>

#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/CDisk.h>


#include "RobcioMapTools.h"
//*)
// General global variables:



using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::bayes;
using namespace std;
RobcioMapTools::RobcioMapTools(RobcioData *robcioDataArg):RobcioLocalizationPF(robcioDataArg)
{
	
	robcioData=robcioDataArg;

};

void RobcioMapTools:: readDataScanFromString(std::string lineData){


	rowNumber++;
	//printf("\n readDataScanFromCSV \n");
	CPose2D odometryIncrements;
	CPose2D odometryIncrementsTest;
	vector<double> changeStatusArrayData=parseLineLog(lineData);
	string actionStep=parseLineGetStat(lineData); 

	if(startTimestamp==0){
		startTimestamp=mrpt::system::time_tToTimestamp(changeStatusArrayData[0]);

	}
	mrpt::system::TTimeStamp timestamp=startTimestamp+mrpt::system::secondsToTimestamp(rowNumber);


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
	cob.timestamp=timestamp;
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
	the_scan->timestamp = timestamp;
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
			vector<double> arrayMapXY=getCordinate(messs.range,DEG2RAD(overAll*(newCompasRadius))+ messs.yaw);
			robcioData->putDataMapXY(odoLast.x()+arrayMapXY[0],odoLast.y()+arrayMapXY[1]);						
			the_scan->validRange.push_back(1);
		}

	}



	//pathX.push_back(odoLast.x());
	//pathY.push_back(odoLast.y());
	CActionCollection    acts;
	//CActionRobotMovement2D move;


	CActionRobotMovement2D	act;
	CActionRobotMovement2D::TMotionModelOptions	opts;
	opts.modelSelection = CActionRobotMovement2D::mmGaussian;

	CPose2D  Aodom =cob.odometry- odoLast;

	act.computeFromOdometry(Aodom, opts);
	act.timestamp = timestamp;

	acts.insert(act);

	odoLast=cob.odometry;
	CSensoryFrame frame;


	CObservationBearingRange *rang= new CObservationBearingRange(m_lastObservation);
	rang->timestamp= timestamp;
	frame.insert(the_scan);
	frame.insert( CObservationBearingRangePtr(rang ));


	robcioData->putDataPathXY(odoLast.x(),odoLast.y());	
	out_file << frame;
	out_file <<acts;
	//out_file << odom;




};

void RobcioMapTools:: parseLineData(std::string lineData){


	rowNumber++;
	//printf("\n readDataScanFromCSV \n");
	CPose2D odometryIncrements;
	CPose2D odometryIncrementsTest;
	vector<double> changeStatusArrayData=parseLineLog(lineData);
	string actionStep=parseLineGetStat(lineData); 

	if(startTimestamp==0){
		startTimestamp=mrpt::system::time_tToTimestamp(changeStatusArrayData[0]);

	}
	mrpt::system::TTimeStamp timestamp=startTimestamp+mrpt::system::secondsToTimestamp(rowNumber);


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
	cob.timestamp=timestamp;
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
	the_scan->timestamp = timestamp;
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
			vector<double> arrayMapXY=getCordinate(messs.range,DEG2RAD(overAll*(newCompasRadius))+ messs.yaw);
			robcioData->putDataMapXY(odoLast.x()+arrayMapXY[0],odoLast.y()+arrayMapXY[1]);						
			the_scan->validRange.push_back(1);
		}

	}



	//pathX.push_back(odoLast.x());
	//pathY.push_back(odoLast.y());
	CActionCollection    acts;
	//CActionRobotMovement2D move;


	CActionRobotMovement2D	act;
	CActionRobotMovement2D::TMotionModelOptions	opts;
	opts.modelSelection = CActionRobotMovement2D::mmGaussian;

	CPose2D  Aodom =cob.odometry- odoLast;

	act.computeFromOdometry(Aodom, opts);
	act.timestamp = timestamp;

	acts.insert(act);

	odoLast=cob.odometry;
	CSensoryFrame frame;


	CObservationBearingRange *rang= new CObservationBearingRange(m_lastObservation);
	rang->timestamp= timestamp;
	frame.insert(the_scan);
	frame.insert( CObservationBearingRangePtr(rang ));


	robcioData->putDataPathXY(odoLast.x(),odoLast.y());	
	out_file << frame;
	out_file <<acts;

	//out_file << odom;




};


void RobcioMapTools:: simpleReadRawlog(string RAWLOG_FILE){
	// --------------------------
	// Load the rawlog:
	// --------------------------
	printf("Opening the rawlog file...");
	CFileGZInputStream rawlog_in_stream(RAWLOG_FILE);
	printf("OK\n");
	bool end=false;
	size_t rawlogEntry = 0;
	while (!end)
	{
		// Finish if ESC is pushed:
		if (os::kbhit())
			if (os::getch()==27)
				end = true;

		// Load pose change from the rawlog:
		// ----------------------------------------
		CActionCollectionPtr action;
		CSensoryFramePtr     observations;
		CObservationPtr 	 obs;

		if (!CRawlog::getActionObservationPairOrObservation(
			rawlog_in_stream,      // In stream
			action,	observations,  // Out pair <action,SF>, or:
			obs,                   // Out single observation
			rawlogEntry            // In/Out index counter.
			))
		{
			end = true;
			continue;
		}


		getPointFromObject(action,observations,obs);
	}
}



void RobcioMapTools::getPointFromObject(	CActionCollectionPtr action,CSensoryFramePtr     observations,CObservationPtr 	 obs){
	CPose2D pose2d;
	CActionRobotMovement2DPtr obs2 = action->getActionByClass<CActionRobotMovement2D>();
	obs2->poseChange->getMean(pose2d);
	poseLastRobot=poseLastRobot+pose2d;


	//	putDataPathXY(poseLastRobot.x(),poseLastRobot.y());
	CSimplePointsMap	map;																
	map.clear();
	observations->insertObservationsInto( &map );


	vector<float> partMapX, partMapY;


	map.changeCoordinatesReference(poseLastRobot);
	map.getAllPoints(partMapX,partMapY);


	robcioData->putDataPathXY(poseLastRobot.x(),poseLastRobot.y());
	for(int i=0;i<partMapX.size();i++){
		//CPose2D posePoint(partMapX[i],partMapY[i],0.0);
		//posePoint=posePoint+poseLastRobot;
		robcioData->putDataMapXY(partMapX[i],partMapY[i]);
		//printf(to_string(i).c_str());

	}

	printf("End get point\n");
	//std::count<<"End all point";

}


// ------------------------------------------------------
//				SimpleMap
// ------------------------------------------------------
void RobcioMapTools::exportSimpleMapToFile(string rawlog){
	printf("Start exportSimpleMapToFile");
	importFromCSVFile();
//	printf("\n Start createSimpleMapPointSLAM \n");
//	CSimpleMap simpleMap=createSimpleMapPointSLAM(rawlog);
//	saveSimpleMap(simpleMap,"C:/Temp/newSimplemap.simplemap");
};
CSimpleMap RobcioMapTools:: loadSimpleMap(string path) { 

	CSimpleMap mapToLoad;
	CFileInputStream (path) >> mapToLoad;
	return mapToLoad; 
};
void  RobcioMapTools::saveSimpleMap(CSimpleMap mapToSave,string path) { 
	CFileOutputStream (path) << mapToSave;
};
CSimpleMap RobcioMapTools::createSimpleMapPointSLAM(string pathToRawLog) { 

	CRawlog rawLogTest;
	rawLogTest.loadFromRawLogFile(pathToRawLog);
	CSimpleMap simpleMap;


	CPose2D     curPose(0,0,0);
	size_t      i;
	int count=0;
	CSensoryFramePtr observations;
	CPosePDFGaussianPtr posePDFGaussian;
	for (i=0; i<rawLogTest.size()-1;i++)
	{
		printProgress(&count);
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


