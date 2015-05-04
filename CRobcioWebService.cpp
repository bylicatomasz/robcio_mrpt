#include "CRobcioWebService.h"

#include <boost/network/protocol/http/server.hpp>
#include <string>
#include <iostream>
#include <thread>

namespace http = boost::network::http;

struct webServiceListner;
typedef http::server<webServiceListner> server;
CRobcioSLAM *robcio;
//xRobcioWinWidgetsFrame *form;
struct webServiceListner {
    void operator() (server::request const &request,
                     server::response &response) {
        /*std::string ip = source(request);
        response = server::response::stock_reply(
            server::response::ok, std::string("Hello, ") + ip + "!");
			*/

		server::string_type ip = source(request);
		server::string_type body_ = body(request);
		
		server::string_type method_ = method(request);
		unsigned int port = request.source_port;
        std::ostringstream data;
		std::ostringstream data2;

		data << "OK"<< "\n";
		if(body_.empty()==false && body_.length()>10){
			::robcio->readDataScanFromCSV(urlDecode(body_).replace(0,8,""));
		//	::form->OnRawMapReload();
		}
        response = server::response::stock_reply(
            server::response::ok, data.str());


    }
	void log(...) {
        // do nothing
    }
	std::string urlDecode(std::string &SRC) {
			std::string ret;
			char ch;
			int i, ii;
			for (i=0; i<SRC.length(); i++) {
				if (int(SRC[i])==37) {
					sscanf(SRC.substr(i+1,2).c_str(), "%x", &ii);
					ch=static_cast<char>(ii);
					ret+=ch;
					i=i+2;
				} else {
					ret+=SRC[i];
				}
			}
			return (ret);
		}
};
void startWebService(){
  try {
	  	char *address="127.0.0.1";
	   char *port="8181";
       webServiceListner handler;
		server::options options(handler);
		server server_(options.address(address).port(port));
        //server server_(address, port, handler);
        server_.run();
    }
    catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
   
    }

};
std::thread helper1(startWebService);
void CRobcioWebService::initWebService(CRobcioSLAM *rob){
	std::cout << "starting first helper...\n";
	::robcio=rob;
	::robcio->initFileGridExport();
	//::form=form;
};
