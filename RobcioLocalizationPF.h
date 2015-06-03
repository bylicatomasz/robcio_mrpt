#include "RobcioData.h"
#ifndef ROBCIOLOCALIZATIONPF_H
#define ROBCIOLOCALIZATIONPF_H


#include <mrpt/otherlibs/mathplot/mathplot.h>

#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDFGaussian.h>

#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/utils/CTicTac.h>

#include <mrpt/math/ops_vectors.h> // << for vector<>
#include <mrpt/math/distributions.h>
#include <mrpt/math/utils.h>

#include <mrpt/slam/CMonteCarloLocalization2D.h>
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
#include <mrpt/system/filesystem.h>

#include <mrpt/random.h>
#include <mrpt/bayes/CParticleFilter.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::bayes;
using namespace std;

class RobcioLocalizationPF
{
protected:
	RobcioData *robcioData;
	string file_rawlog_output;
	string file_input_csv;
	string file_csv;
	string actionStepLast;

	bool isReady;
	bool isStart;
	int lendmarkId;
	int rowNumber;
	CPose2D poseLastRobot;
	CPose2D  odoLast;
	
	mrpt::system::TTimeStamp startTimestamp;
	CFileGZOutputStream	out_file;
	double radiusFromCompasSum;
	double radiusFromCompasLast;
	double compasRadius;
	double dystance;	
	double lastX;
	double lastY;
	double newDystanceSum;
	double newCompasRadiusSum;
  public:
	RobcioLocalizationPF(RobcioData *robcioDataArg);
	vector<std::string>  split(const std::string &s, char delim, std::vector<std::string> &elems) ;
	vector<std::string>  split(const std::string &s, char delim);
	vector<double>  parseLineLog(string line);
	string  parseLineGetStat(string line);
	vector<double> getCordinate(double dystance, double radius);
	void do_pf_localization_slam(const std::string &ini_fil);
	bool parseLineDataForSLAM(CActionCollection &out_action,CSensoryFrame     &out_observations);
	void getGroundTruth( CPose2D &expectedPose, size_t rawlogEntry, const CMatrixDouble &GT, const TTimeStamp &cur_time);
	void closeFileExport();
	void initFileExport();
	void importFromCSVFile();
	void printProgress(int *count);
	string currentDateTime();
};
#endif