#ifndef ROBCIOMAPTOOLS_H
#define ROBCIOMAPTOOLS_H
#include <mrpt/otherlibs/mathplot/mathplot.h>
#include"RobcioLocalizationPF.h"
#include"RobcioData.h"
using namespace std;
class RobcioMapTools: public RobcioLocalizationPF
{
private:
	
	/*
	bool isReady;
	int lendmarkId;
	int rowNumber;
	CPose2D poseLastRobot;
	CPose2D  odoLast;
	bool isStart;
	mrpt::system::TTimeStamp startTimestamp;
	CFileGZOutputStream	out_file;
	double compasRadius;
	double dystance;
	string actionStepLast;
	double lastX;
	double lastY;*/
	RobcioData *robcioData;
  public:
	  
	RobcioMapTools(RobcioData *robcioDataArg);
	
	void  parseLineData(std::string lineData);
	void readDataScanFromString(std::string lineData);
	void simpleReadRawlog(string RAWLOG_FILE);
	void exportSimpleMapToFile(string rawlog);
	CSimpleMap loadSimpleMap(string path);
	void  saveSimpleMap(CSimpleMap mapToSave,string path);
	CSimpleMap createSimpleMapPointSLAM(string pathToRawLog) ;
	void closeFileExport();
	void initFileExport();
	//void importFromCSVFile();
	//void printProgress(int *count);
	void getPointFromObject(	CActionCollectionPtr action,CSensoryFramePtr     observations,CObservationPtr 	 obs);
	/*
	vector<std::string>  split(const std::string &s, char delim, std::vector<std::string> &elems) ;
	vector<std::string>  split(const std::string &s, char delim);
	vector<double>  parseLineLog(string line);
	string  parseLineGetStat(string line);
	vector<double> getCordinate(double dystance, double radius);
	void do_pf_localization_slam(const std::string &ini_fil,string CONFIG_FILE);
	bool parseLineDataForSLAM(CActionCollection &out_action,CSensoryFrame     &out_observations);
	void getGroundTruth( CPose2D &expectedPose, size_t rawlogEntry, const CMatrixDouble &GT, const TTimeStamp &cur_time);*/
};
#endif