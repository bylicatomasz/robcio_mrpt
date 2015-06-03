#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CFileInputStream.h>

#include "RobcioTools.h"

//*)
// General global variables:



using namespace mrpt;
using namespace std;

RobcioTools::RobcioTools()
{
	


};

//------------------------------------------------------
//				Tools
//-----------------------------------------------------

void RobcioTools::log(char *message){

	ofstream myfile (file_log, ios::out | ios::app );
	if (myfile.is_open())
	{
		time_t rawtime;
		time (&rawtime);
		string messageOutput;
		messageOutput.append(ctime (&rawtime));
		messageOutput.append(message);
		messageOutput.append("\n");
		myfile << messageOutput;    
		myfile.close();
	}
};
void RobcioTools:: log(string message){

	ofstream myfile (file_log, ios::out | ios::app );
	if (myfile.is_open())
	{
		time_t rawtime;
		time (&rawtime);
		string messageOutput;
		messageOutput.append(ctime (&rawtime));
		messageOutput.append(message);
		messageOutput.append("\n");
		myfile << messageOutput;    
		myfile.close();
	}
};
//------------------------------------------------------
//				CSV File
//-----------------------------------------------------

void RobcioTools:: writeToCSV(string message){

	ofstream myfile (file_csv, ios::out | ios::app );
	if (myfile.is_open())
	{
		myfile << message;    
		myfile.close();
	}
};

// 

