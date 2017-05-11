// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _DSOCKET_H
#define _DSOCKET_H

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include "Define.h"

#include <stdio.h>
#include <Winsock.h>
#include <sstream>
#include <string>
#include <memory>

using namespace std;

//socket client to communicate with socket server
class DSocket{
public:
	typedef std::shared_ptr<DSocket> Ptr;
	typedef std::shared_ptr<DSocket const> ConstPtr;

	DSocket(const string &serverIp, const unsigned short portNo);
	~DSocket();

	bool openSocket(); //open socket
	void closeSocket(); //close socket

	bool doOpenCamera();
	bool doCloseCamera();
	int doGetImageWidth();
	int doGetImageHeight();
	bool doGetNewImages(Vector4u *rgb, short *depth, const int width, const int height);

private:
	SOCKET sockClient;
	string serverIp;
	unsigned short portNo;
};

#endif //DSOCKET_H

