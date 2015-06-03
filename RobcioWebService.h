#ifndef ROBCIOWEBSERVICE_H
#define ROBCIOWEBSERVICE_H
#include "RobcioData.h"
#include <mrpt/otherlibs/mathplot/mathplot.h>
#include <boost/network/protocol/http/server.hpp>
using namespace std;
namespace http = boost::network::http;
class RobcioWebService
{
private:
	bool isReady;
	RobcioData *robcioData;
  public:
	
	RobcioWebService(RobcioData *robcioDataArg);
	struct webServiceListner;
	typedef http::server<webServiceListner> server;
	void startWebService();
};
#endif
