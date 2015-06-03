#include <mrpt/otherlibs/mathplot/mathplot.h>
#ifndef ROBCIODATA_H
#define ROBCIODATA_H
using namespace std;

class RobcioData
{
 private:
	bool isReady;
	vector<string> dataStorage;
	vector<double> mapX;
	vector<double> mapY;
	vector<double> pathX;
	vector<double> pathY;
	bool isLock;
	bool isLockXY;
	bool isLockUpdate;
	int randomMax;
  public:
	RobcioData();
	void putDataMapXY(double x,double y);
	void putDataPathXY(double x,double y);
	void putData(string data);
	string getData();
	vector<double> getDataPathY();
	vector<double> getDataPathX();
	vector<double> getDataMapY();
	vector<double> getDataMapX();
};
#endif