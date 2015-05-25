#include "CRobcioTools.h"
#include <time.h>
void RobcioTools::log(char *message){

 ofstream myfile ("C:\\Temp\\LogRobcio.txt", ios::out | ios::app );
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
}
void RobcioTools::log(string message){
	 


  
 ofstream myfile ("C:\\Temp\\LogRobcio.txt", ios::out | ios::app );
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
}