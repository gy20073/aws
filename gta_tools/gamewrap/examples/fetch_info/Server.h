#pragma once

#include "Scenario.h"
#include "log.h"
#include <windows.h>

#include <chrono>
#include <mutex>
#include <ostream>
#include <unordered_map>
#include <queue>
#include <memory>
#include <WinSock2.h>
#include <WS2tcpip.h>

// Need to link with Ws2_32.lib
#pragma comment (lib, "ws2_32")

class Server {
private:
	// Options
	bool sentFrame = false;
	SOCKET listen_socket = 0;
	std::vector<SOCKET> client;
	std::mutex state_mutex;

	WSADATA wsaData;
	u_long iMode = 1; //non-blocking socket
	SOCKET ServerSocket = INVALID_SOCKET;
	SOCKET ClientSocket = INVALID_SOCKET;
	
	int bytesRead = 0;
	int recvMessageLen = 0;
	int sendMessageLen = 0;
	bool readyToSend = false;
	bool frameSent = false;
	bool running;
	HANDLE th;
	char json[4096];
	StringBuffer message;
	const char* chmessage;
	int messageSize = 0;
	std::clock_t lastSentMessage = std::clock();

	void resetState();

	FILE* frame_file;

public:
	bool clientConnected = false;
	Scenario scenario;
	bool sendOutputs = false;
	Server(unsigned int port);
	void checkRecvMessage();
	void checkSendMessage();
	void checkClient();
	static DWORD WINAPI _serve(LPVOID that);
};