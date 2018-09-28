#include "Server.h"
#include <thread>
#include "lib/rapidjson/document.h"
#include "lib/rapidjson/stringbuffer.h"
#include "lib/main.h"

using namespace rapidjson;

Server::Server(unsigned int port) {
	struct sockaddr_in server;
	freopen("deepgtav.log", "w", stdout);

	printf("\nInitializing Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		printf("Failed. Error Code: %d", WSAGetLastError());
	}
	printf("Initialized.\n");

	if ((ServerSocket = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET) {
		printf("Could not create socket: %d", WSAGetLastError());
	}
	printf("Socket created.\n");

	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(port);

	if (bind(ServerSocket, (struct sockaddr *)&server, sizeof(server)) == SOCKET_ERROR) {
		printf("Bind failed with error code: %d", WSAGetLastError());
	}
	printf("Bind done.\n");

	printf("Listening...\n");
	if (listen(ServerSocket, 1) == SOCKET_ERROR) {
		printf("Could not listen: %d", WSAGetLastError());
	}

	if (ioctlsocket(ServerSocket, FIONBIO, &iMode) != NO_ERROR) {
		printf("Server ioctlsocket failed");
	}

	scenario.initRewarder();

}

void Server::checkClient() {
	SOCKET tmpSocket = SOCKET_ERROR;
	tmpSocket = accept(ServerSocket, NULL, NULL);
	if (tmpSocket != SOCKET_ERROR) {
		printf("Connection accepted.\n");
		ClientSocket = tmpSocket;
		int size = 1 << 30;
		uint timeout = 100;
		setsockopt(ClientSocket, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout));
		setsockopt(ClientSocket, SOL_SOCKET, SO_SNDBUF, (const char*)&size, sizeof(size));
		if (ioctlsocket(ClientSocket, FIONBIO, &iMode) != NO_ERROR) {
			printf("Client ioctlsocket failed");
			return;
		}
		clientConnected = true;
	}
}

void Server::checkRecvMessage() {
	printf("\n############ Function called");
	int result;
	Document d;
	int error;
	printf("\n############ bytes received before: %d", recvMessageLen);
	if (recvMessageLen == 0) {
		recv(ClientSocket, (char*)&recvMessageLen, 4, 0); //Receive message len first
		error = WSAGetLastError();
		if (error == WSAEWOULDBLOCK) return;
		if (error != 0) {
			printf("\nError receiving message length: %d", error);
			resetState();
			return;
		}
	}

	while (bytesRead < recvMessageLen) {
		result = recv(ClientSocket, json + bytesRead, recvMessageLen - bytesRead, 0);
		error = WSAGetLastError();
		if (error == WSAEWOULDBLOCK) return;
		if (error != 0 || result < 1) {
			printf("\nError receiving message: %d", error);
			resetState();
			return;
		}
		bytesRead = bytesRead + result;
	}
	printf("\n############ bytes received after: %d", recvMessageLen);

	json[bytesRead] = '\0';
	bytesRead = 0;
	recvMessageLen = 0;

	d.Parse(json);

	if (d.HasMember("commands")) {
		printf("Commands received\n");
		const Value& commands = d["commands"];
		scenario.setCommands(commands["throttle"].GetFloat(), commands["brake"].GetFloat(), commands["steering"].GetFloat(), commands["manual"].GetInt());
		printf("\n############ Commands instructions");
	}
	else if (d.HasMember("config")) {
		//Change the message values and keep the others the same
		printf("Config received\n");
		const Value& config = d["config"];
		const Value& sc = config["scenario"];
		const Value& dc = config["dataset"];
		scenario.config(sc, dc);
		printf("\n############ Config instructions");
	}
	else if (d.HasMember("start")) {
		//Set the message values and randomize the others. Start sending the messages
		printf("Start received\n");
		const Value& config = d["start"];
		const Value& sc = config["scenario"];
		const Value& dc = config["dataset"];
		scenario.start(sc, dc);
		sendOutputs = true;
		printf("\n############ Start instructions");
	}
	else if (d.HasMember("stop")) {
		//Stop sendig messages, keep client connected
		printf("Stop received\n");
		sendOutputs = false;
		scenario.stop();
		printf("\n############ Stop instructions");
	}
	else {
		printf("\n############ No instructions received");
		return; //Invalid message
	}

}

void Server::checkSendMessage() {
	int error;
	int r;


	// if (sendOutputs && (((float)(std::clock() - lastSentMessage) / CLOCKS_PER_SEC) > (1.0 / scenario.rate))) {
	if (sendOutputs) {
		//////
		lastSentMessage = std::clock();
		clock_t t1 = clock();
		clock_t tmsg1 = clock();
		if (messageSize == 0) {
			message = scenario.generateMessage();
			chmessage = message.GetString();
			messageSize = message.GetSize();
		}
		clock_t tmsg2 = clock();
		printf("\n-> Generate Msg time: %f", double(tmsg2 - tmsg1) / CLOCKS_PER_SEC);

		// Sending the frame here
		clock_t begin_time = clock();

		// dummy send frame
		if (!frameSent) {
			if (!readyToSend) {
				readyToSend = true;
				sendMessageLen = 0;
			}
			readyToSend = false;
			frameSent = true;
		}


		clock_t end_time = clock();
		double elapsed_secs = double(end_time - begin_time) / CLOCKS_PER_SEC;
		printf("\n-> Frame sending time: %f", elapsed_secs);

		clock_t tsendmsg1 = clock();
		if (frameSent) {
			if (!readyToSend) {
				send(ClientSocket, (const char*)&messageSize, sizeof(messageSize), 0);
				error = WSAGetLastError();
				if (error == WSAEWOULDBLOCK) return;
				if (error != 0) {
					printf("\nError sending message length: %d", error);
					resetState();
					return;
				}
				readyToSend = true;
				sendMessageLen = 0;
			}

			while (readyToSend && (sendMessageLen < messageSize)) {
				r = send(ClientSocket, (const char*)(chmessage + sendMessageLen), messageSize - sendMessageLen, 0);
				error = WSAGetLastError();
				if (error == WSAEWOULDBLOCK) return;
				if (error != 0 || r <= 1) {
					printf("\nError sending message: %d", error);
					resetState();
					return;
				}
				sendMessageLen = sendMessageLen + r;
			}
			clock_t tsendmsg2 = clock();
			printf("\n-> Send Msg time: %f", double(tsendmsg2 - tsendmsg1) / CLOCKS_PER_SEC);

			readyToSend = false;
			messageSize = 0;
			frameSent = false;
		}
		// lastSentMessage = std::clock();
		clock_t t2 = clock();
		printf("\nInner checkSendMessage time: %f", double(t2 - t1) / CLOCKS_PER_SEC);
	}
	else {
		printf("\n!!!!!!!Judgement False: %d %f", sendOutputs, ((float)(std::clock() - lastSentMessage) / CLOCKS_PER_SEC));
	}
}

void Server::resetState() {
	shutdown(ClientSocket, SD_SEND);
	closesocket(ClientSocket);

	clientConnected = false;
	sendOutputs = false;
	bytesRead = 0;
	recvMessageLen = 0;
	sendMessageLen = 0;
	readyToSend = false;
	frameSent = false;
	messageSize = 0;

	scenario.stop();
}