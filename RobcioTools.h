#ifndef ROBCIOTOOLS_H
#define ROBCIOTOOLS_H
#include <mrpt/otherlibs/mathplot/mathplot.h>
using namespace std;
class RobcioTools
{
private:
	string file_csv;
	string file_log;
  public:
	RobcioTools();
	void log(char *message);
	void log(string message);
	void writeToCSV(string message);


};
#endif