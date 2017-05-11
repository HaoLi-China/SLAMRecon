// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include "DSocket.h"
#include <string.h>

const int MAXRECV = 10240;

DSocket::DSocket(const string &serverIp, const unsigned short portNo)
{
	this->serverIp = serverIp;
	this->portNo = portNo;
}

DSocket::~DSocket()
{
}

bool DSocket::openSocket()
{
	WORD wVersionRequested;
	WSADATA wsaData;
	int err;

	wVersionRequested = MAKEWORD(1, 1);

	err = WSAStartup(wVersionRequested, &wsaData);
	if (err != 0) {
		return false;
	}

	if (LOBYTE(wsaData.wVersion) != 1 ||
		HIBYTE(wsaData.wVersion) != 1) {
		WSACleanup();
		return false;
	}
	sockClient = socket(AF_INET, SOCK_STREAM, 0);

	SOCKADDR_IN addrSrv;
	addrSrv.sin_addr.S_un.S_addr = inet_addr(serverIp.c_str());
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(portNo);
	printf("send connection...\n");
	connect(sockClient, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));
	printf("connected\n");

	return true;
}

void DSocket::closeSocket()
{
	send(sockClient, "c", strlen("c") + 1, 0);
	closesocket(sockClient);
	WSACleanup();
}

bool DSocket::doOpenCamera()
{
	char sendData[1];
	sendData[0] = 'a';
	if (send(sockClient, sendData, 1, 0) <= 0){
		printf("doOpenCamera send error\n");
		return false;
	}

	char rcvData[9];
	recv(sockClient, rcvData, 9, 0);
	if (strcmp(rcvData, "finished")){
		printf("doOpenCamera error\n");
		return false;
	}

	return true;
}

bool DSocket::doCloseCamera()
{
	char sendData[1];
	sendData[0] = 'b';
	if (send(sockClient, sendData, 1, 0) <= 0){
		printf("doCloseCamera send error\n");
		return false;
	}

	char rcvData[9];
	recv(sockClient, rcvData, 9, 0);
	if (strcmp(rcvData, "finished")){
		printf("doCloseCamera error\n");
		return false;
	}

	return true;
}

int DSocket::doGetImageWidth()
{
	char sendData[1];
	sendData[0] = 'i';
	send(sockClient, sendData, 1, 0);

	char recvData[256];
	int n = recv(sockClient, recvData, 257, 0);
	int wid;
	memcpy(&wid, &recvData, 4);
	printf("wid=%d\n", wid);
	return wid;
}

int DSocket::doGetImageHeight()
{
	char sendData[1];
	sendData[0] = 'j';
	send(sockClient, sendData, 1, 0);

	char recvData[256];
	int n = recv(sockClient, recvData, 257, 0);
	int hei;
	memcpy(&hei, &recvData, 4);
	printf("hei=%d\n", hei);
	return hei;
}

bool DSocket::doGetNewImages(Vector4u *rgb, short *depth, const int width, const int height)
{
	char sendData[1];
	sendData[0] = 'd';
	send(sockClient, sendData, 1, 0);

	int n = 0;
	int count = 0;
	int totalLen = (width * height * (sizeof(uchar) * 3 + sizeof(short)));
	char pkgData[MAXRECV];
	char* rcvData = new char[totalLen];
	int ignore = 0;
	while (1){
		if (count == totalLen)
			break;
		if (count > totalLen){
			ignore = 1;
			break;
		}
		n = recv(sockClient, pkgData, MAXRECV, 0);
		if (count + n <= totalLen)
			memcpy(&rcvData[count], &pkgData, n);
		else
			memcpy(&rcvData[count], &pkgData, totalLen - count);
		count += n;
	}
	if (ignore == 1){
		delete[] rcvData;
		printf("ignore\n");
		return false;
	}
	int crt = 0;
	for (int i = 0; crt < width * height * sizeof(uchar); i++, crt++)
		rgb[i].r = rcvData[crt];
	for (int i = 0; crt < width * height * sizeof(uchar) * 2; i++, crt++)
		rgb[i].g = rcvData[crt];
	for (int i = 0; crt < width * height * sizeof(uchar) * 3; i++, crt++){
		rgb[i].b = rcvData[crt];
		rgb[i].a = 255;
	}

	memcpy(depth, rcvData + sizeof(uchar) * width * height * 3, sizeof(short) * width * height);

	delete[] rcvData;
	return true;
}
