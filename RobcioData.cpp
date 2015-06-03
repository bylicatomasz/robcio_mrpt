#include <mrpt/system/threads.h> 
#include "RobcioData.h"




// General global variables:



using namespace std;
using namespace mrpt;
using namespace mrpt::system;

//--------------------------------------------------------
//			Access to data 
//--------------------------------------------------------

RobcioData::RobcioData(){
	isLock=false;
	isLockXY=false;
	isLockUpdate=false;
	int randomMax=100;
}

void RobcioData:: putData(string data){
	if(isLock==false){
		isLock=true;
		dataStorage.push_back(data);
		isLock=false;
		return;
	}else{	
		int maxRand=rand()%randomMax;
		sleep(maxRand);
		putData(data);
	}

}
string RobcioData:: getData(){
	if(isLock==false ){
		isLock=true;
		if(dataStorage.size()>0){
			string returnData=dataStorage.front();
			//dataStorage.pop_back();
			dataStorage.erase( dataStorage.begin() );
			isLock=false;	
			return returnData;
		}
		isLock=false;	
		return "";
	}else{
		int maxRand=rand()%randomMax;
		sleep(maxRand);
		return getData();
	}



}
void RobcioData::putDataPathXY(double x,double y){
	if(isLockXY==false){
		
		isLockXY=true;
		pathX.push_back(x);
		pathY.push_back(y);		
		isLockXY=false;
		return;
	}else{	
		int maxRand=rand()%randomMax;
		sleep(maxRand);
		return putDataPathXY(x,y);
	}

}
void RobcioData:: putDataMapXY(double x,double y){
	if(isLockXY==false){
		
		isLockXY=true;
		mapX.push_back(x);
		mapY.push_back(y);		
		isLockXY=false;
		return;
	}else{	
		int maxRand=rand()%randomMax;
		sleep(maxRand);
		return putDataMapXY(x,y);
	}

}
vector<double> RobcioData:: getDataMapX(){
	if(isLockXY==false){
		
		isLockXY=true;
		vector<double> mapXCopy(mapX);
	//	mapX.clear();
		isLockXY=false;
		return mapXCopy;
	}else{
		int maxRand=rand()%randomMax;
		sleep(maxRand);
		return getDataMapX();
	}

}
vector<double> RobcioData:: getDataMapY(){
	if(isLockXY==false){
		
		isLockXY=true;
		vector<double> mapYCopy(mapY);;
	//	mapY.clear();
		isLockXY=false;
		return mapYCopy;
	}else{	
		return getDataMapY();
	}

}

vector<double> RobcioData:: getDataPathX(){
	if(isLockXY==false){
		
		isLockXY=true;
		vector<double> pathXCopy(pathX);
	//	pathX.clear();
		isLockXY=false;
		return pathXCopy;
	}else{	
		return getDataPathX();
	}

}
vector<double> RobcioData:: getDataPathY(){
	if(isLockXY==false){
		
		isLockXY=true;
		vector<double> pathYCopy(pathY);
	//	pathY.clear();
		isLockXY=false;
		return pathYCopy;
	}else{	
		return getDataPathY();
	}

}
