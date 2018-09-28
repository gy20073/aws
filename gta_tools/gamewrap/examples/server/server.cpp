#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include <chrono>
#include <mutex>
#include <ostream>
#include <unordered_map>
#include <queue>
#include <memory>
#include "log.h"
#include "sdk.h"

#pragma comment(lib, "ws2_32.lib")

const uint16_t DEFAULT_PORT = 8766;
#define MAX_BUFFER_SIZE (1<<14)

double time() {
	auto now = std::chrono::system_clock::now();
	return std::chrono::duration<double>(now.time_since_epoch()).count();
}

// Note on the callback order: You're guaranteed that the game state arrives after all requested frames have been sent

const size_t MESSAGE_HEADER_SIZE = 3;
struct Message {
	enum Command {
		LIST_TARGETS = 0,
		FETCH_TARGET = 1,
		INTERCEPT_KEY = 2,
		SEND_KEY = 3,
		RESET = 4,
		FETCH_GAME_STATE = 5,
		CONTROL_REQUEST = 7,
		TARGET_RECEIVED = 8,
	};
	uint16_t id = 0;
	uint8_t command = 0;
	std::vector<uint8_t> data;
	Message() = default;
	Message(const char * d, uint32_t n) {
		memcpy(this, d, MESSAGE_HEADER_SIZE);
		const uint8_t* cd = (const uint8_t*)d;
		data = std::vector<uint8_t>(cd + MESSAGE_HEADER_SIZE, cd + n);
	}
	Message(uint8_t c, uint16_t i, const std::vector<uint8_t>& d = std::vector<uint8_t>()) :id(i), command(c), data(d) {}
template<typename T>
	Message(uint8_t c, uint16_t i, T d) :id(i), command(c), data((uint8_t*)&d, ((uint8_t*)&d)+sizeof(T)) {
	}
};
void appendString(const std::string & s, std::vector<uint8_t> & b) {
	uint16_t n = (uint16_t)s.size();
	b.insert(b.end(), (const uint8_t*)&n, (const uint8_t*)(&n + 1));
	b.insert(b.end(), s.begin(), s.end());
}
std::string readString(const uint8_t * data, size_t & i, size_t n) {
	if (i + sizeof(uint16_t) > n) return "";
	uint16_t m = *(uint16_t*)(data+i);
	i += sizeof(uint16_t);

	if (i + m > n) return "";
	std::string r(data+i, data + i + m);
	i += m;
	return r;
}
std::vector<uint8_t> targetToData(const std::vector<std::string> & targets) {
	std::vector<uint8_t> r;
	for (const std::string & t : targets)
		appendString(t, r);
	return r;
}
std::vector<std::string> dataToTarget(const uint8_t * data, size_t n ) {
	std::vector<std::string> r;
	for (size_t i = 0; i < n; )
		r.push_back(readString(data, i, n));
	return r;
}
std::vector<std::string> dataToTarget(const std::vector<uint8_t> & data) {
	return dataToTarget(data.data(), data.size());
}

#pragma pack(push)
#pragma pack(1)
struct SendKeyMessage {
	uint8_t key;
	// How long do you want to press the key down? A nonpositive value will release the key.
	float duration;
};
struct InterceptKeyRequest {
	uint8_t start_stop; // 1: start , 0 : stop
};
struct ControlRequestRequest {
	uint8_t start_stop; // should we get the right to send commands (1), or not (0)?
	// you need to get control first, before you can reset or send key commands
};
struct InterceptKeyReply {
	enum Command {
		UP = 0,
		DOWN = 1,
	};
	uint8_t key, state;
	uint64_t timestamp;
};
enum FetchStepType {
	MILLISECONDS = 0, // Steps descrive 1/1000 seconds
	FRAMES = 1,       // Steps are in terms of rendered frames (will change with FPS)
};
struct FetchTargetRequestHdr {
	// For now a fetchTarget request will override any prior fetch target requests (this might change in the future)
	uint8_t step_type = MILLISECONDS;
	// Specify how often you want to receive a set of targets (every n miliseconds or n frames)
	uint16_t step = 0;
	// How many frames do you want to fetch (0 means infinite)
	uint16_t n_frames = 0;
	// Desired resolution (resizing will be done in hardware)
	uint16_t W = 0, H = 0;
	// Followed by a list of target names (separated by '\0')
};
struct FetchTargetReplyHdr {
	uint8_t data_type;
	uint16_t W, H, C;
	uint32_t frame_id;
	uint64_t timestamp;

	//uint16_t target_name_size;
	//uint8_t target_name[];
	// Followed by the actual data
};
struct TargetReceivedHdr {
	uint32_t frame_id;
	//uint8_t target_name[];
};
struct FetchGameStateRequest {
	// For now a fetchTarget request will override any prior fetch target requests (this might change in the future)
	uint8_t step_type = MILLISECONDS;
	// Specify how often you want to receive a set of targets (every n miliseconds or n frames)
	uint16_t step = 0;
	// How many frames do you want to fetch (0 means infinite)
	uint16_t n_frames = 0;
};
struct FetchGameStateReply {
	uint32_t frame_id;
	uint64_t timestamp;
	// Followed by the actual data
};
#pragma pack(pop)


std::ostream & operator<<(std::ostream & o, const in_addr & a) {
	static char buf[256];
	return o << InetNtop(AF_INET, &a, buf, 256);
}
std::ostream & operator<<(std::ostream & o, const sockaddr_in & a) {
	static char buf[256];
	return o << InetNtop(a.sin_family, &a.sin_addr, buf, 256) << ":" << ntohs(a.sin_port);
}
struct RecvCommandQueue {
	size_t buffer_start = 0;
	std::vector<char> buffer;
	std::vector<Message> messages;
	void recv(const char * v, size_t n) {
		buffer.insert(buffer.end(), v, v + n);
		parseCommands();
	}
	void parseCommands() {
		while (buffer_start < buffer.size()) {
			uint32_t n = *(uint32_t*)(buffer.data() + buffer_start);
			if (buffer.size() < n + sizeof(uint32_t)) break;
			const char * msg_start = buffer.data() + buffer_start + sizeof(uint32_t);
			
			// Parse the message and add it to the message buffer
			Message m(msg_start, n);
			messages.push_back(m);

			buffer_start += n + sizeof(uint32_t);

			// Make sure not to call erase too often for small messages
			if (buffer_start > MAX_BUFFER_SIZE) {
				buffer.erase(buffer.begin(), buffer.begin() + buffer_start);
				buffer_start = 0;
			}
		}
	}
};
struct SendCommandQueue {
	std::vector<char> buffer;
	std::mutex mutex;
	size_t sendAll(SOCKET s) {
		std::vector<char> send_buffer;
		{
			// Put all the prior data into the send buffer and free the buffer (this avoids locks in case send takes too long)
			std::unique_lock<std::mutex> lock(mutex);
			buffer.swap(send_buffer);
		}
		size_t r = send(s, send_buffer.data(), (int)send_buffer.size(), 0);
		if (r != send_buffer.size())
			return 0;
		return r;
	}
	void addMessage(const Message & m) {
		std::unique_lock<std::mutex> lock(mutex);
		buffer.reserve(m.data.size() + MESSAGE_HEADER_SIZE + buffer.size() + sizeof(uint32_t));
		uint32_t n = uint32_t(MESSAGE_HEADER_SIZE + m.data.size());
		buffer.insert(buffer.end(), (const char*)&n, (const char*)(&n) + sizeof(n));
		buffer.insert(buffer.end(), (const char*)&m, ((const char*)&m) + MESSAGE_HEADER_SIZE);
		buffer.insert(buffer.end(), m.data.begin(), m.data.end());
	}
};

struct Step {
	FetchStepType type = MILLISECONDS;
	uint64_t next = 0, step = 0, frames_to_fetch = 0;
	bool update(uint32_t frame_id, uint64_t frame_ts) {
		uint64_t t = type == MILLISECONDS ? frame_ts : frame_id;
		if (next == 0)
			next = t;

		if (next <= t) {
			next = next + step;
			frames_to_fetch--;
			return true;
		}
		return false;
	}
	operator bool() const {
		return frames_to_fetch > 0;
	}
	void reset() {
		next = 0;
	}
};

struct ClientState {
	RecvCommandQueue recv;
	SendCommandQueue send;

	bool controlling = false; // Can the current client send control messages (reset, keys)

	// Keyboard intercepts
	bool intercept_keys = 0;
	uint16_t intercept_key_id = 0;

	// Target fetches
	uint32_t fetch_id = 0;
	Step fetch_step;
	bool drawing = false;
	uint32_t target_w = 0, target_h = 0;
	std::vector<std::string> targets;
	uint32_t last_frame_sent = 0;
	std::unordered_map<std::string, bool> targets_received;

	// Gamestate
	uint16_t game_state_id = 0;
	bool fetch_game_state = false;
	Step game_state_step;
};

struct Server : public GameController {
	SOCKET listen_socket = 0;
	std::vector<SOCKET> client;
	std::mutex state_mutex;
	std::unordered_map<SOCKET, std::shared_ptr<ClientState> > state;
	bool running = false;
	HANDLE th;

	double keyup_timeout[256] = { 0 };
	void sendKey(uint8_t key, float duration) {
		if (duration > 0) {
			sendKeyDown(key, false);
			LOG(INFO) << "Key down " << (uint16_t)key;
			keyup_timeout[key] = time() + duration;
		} else if (keyup_timeout[key]){
			keyup_timeout[key] = 0;
			LOG(INFO) << "Key up " << (uint16_t)key;
			sendKeyUp(key, false);
		}
	}
	void handleKeyUp() {
		double t = time();
		for(uint16_t k=0; k<256; k++)
			if (keyup_timeout[k] > 0 && keyup_timeout[k] < t) {
				keyup_timeout[k] = 0;
				LOG(INFO) << "Key up " << k;
				sendKeyUp((uint8_t)k, false);
			}
	}
	virtual void sendKeyMessage(uint8_t key, uint8_t s) {
		std::unique_lock<std::mutex> lock(state_mutex);
		InterceptKeyReply m{ key, s, (uint64_t)(time()*1000) };
		for (const auto & s : state) 
			if (s.second->intercept_keys)
				s.second->send.addMessage({ Message::INTERCEPT_KEY, s.second->intercept_key_id, m });
	}
	virtual bool keyDown(unsigned char key, unsigned char special_status) {
		sendKeyMessage(key, 1);
		return false;
	}
	virtual bool keyUp(unsigned char key) {
		sendKeyMessage(key, 0);
		return false;
	}
	
	void handleMessages(SOCKET socket, std::shared_ptr<ClientState> state) {
		for (const Message & m : state->recv.messages) {
			if (m.command == Message::LIST_TARGETS) {
				state->send.addMessage({ m.command, m.id, targetToData(listTargets()) });
			} else if (m.command == Message::SEND_KEY && state->controlling) {
				SendKeyMessage km;
				memcpy(&km, m.data.data(), sizeof(km));
				sendKey(km.key, km.duration);
			} else if (m.command == Message::INTERCEPT_KEY) {
				InterceptKeyRequest km;
				memcpy(&km, m.data.data(), sizeof(km));
				state->intercept_keys = km.start_stop;
				state->intercept_key_id = m.id;
			} else if (m.command == Message::FETCH_TARGET) {
				// Lets make sure we don't override the output while it's being produced
				std::unique_lock<std::mutex> lock(state_mutex);

				FetchTargetRequestHdr tm;
				memcpy(&tm, m.data.data(), sizeof(tm));
				state->targets = dataToTarget(m.data.data() + sizeof(tm), m.data.size() - sizeof(tm));
				state->fetch_step = { (FetchStepType)tm.step_type, 0, tm.step, tm.n_frames ? tm.n_frames : (uint64_t)-1 };
				state->drawing = false;
				state->target_w = tm.W ? tm.W : defaultWidth();
				state->target_h = tm.H ? tm.H : defaultHeight();
				state->fetch_id = m.id;
			} else if (m.command == Message::RESET && state->controlling) {
				requestGameReset();
			} else if (m.command == Message::FETCH_GAME_STATE) {
				std::unique_lock<std::mutex> lock(state_mutex);

				FetchGameStateRequest tm;
				memcpy(&tm, m.data.data(), sizeof(tm));
				state->game_state_step = { (FetchStepType)tm.step_type, 0, tm.step, tm.n_frames ? tm.n_frames : (uint64_t)-1 };
				state->fetch_game_state = false;
				state->game_state_id = m.id;
			} else if (m.command == Message::CONTROL_REQUEST) {
				ControlRequestRequest cr;
				memcpy(&cr, m.data.data(), sizeof(cr));
				state->controlling = false;
				if (cr.start_stop) {
					bool can_control = true;
					for (const auto & os : this->state)
						if (os.second->controlling)
							can_control = false;
					state->controlling = can_control;
				}
				cr.start_stop = state->controlling;
				state->send.addMessage({ m.command, m.id, cr });
			} else if (m.command == Message::TARGET_RECEIVED) {
				std::unique_lock<std::mutex> lock(state_mutex);

				TargetReceivedHdr tr;
				memcpy(&tr, m.data.data(), sizeof(tr));
				size_t i = sizeof(tr);
				std::string name = readString(m.data.data(), i, m.data.size());
				if (tr.frame_id == state->last_frame_sent)
					state->targets_received[name] = true;
			}
		}
		state->recv.messages.clear();
	}

	Server() : GameController() {
	}
	~Server() {
		shutdown();
	}
	uint64_t frame_timestamp = 0;
	virtual RecordingType recordFrame(uint32_t frame_id) {
		static bool start_once = startServer();
		frame_timestamp = (uint64_t)(time() * 1000);

		// Is there a target that we want to fetch?
		uint8_t r = NONE;
		{
			std::unique_lock<std::mutex> lock(state_mutex);
			for (const auto & i : state) {
				auto s = i.second;
				s->fetch_game_state = s->drawing = false;
				if (s->fetch_step) {
					// Should we skip the frame?
					bool skip = false;
					for (const auto & r : s->targets_received)
						if (!r.second)
							skip = true;

					if (!skip && s->fetch_step.update(frame_id, frame_timestamp)) {
						// Fetch the current frame
						for(const auto & t: s->targets)
							requestOutput(t, s->target_w, s->target_h);
						s->drawing = true;
						s->last_frame_sent = frame_id;
						r |= DRAW;
					}
				} else {
					s->fetch_step.reset();
				}
				if (s->game_state_step) {
					if (s->game_state_step.update(frame_id, frame_timestamp))
						s->fetch_game_state = true;
				}
				else {
					s->game_state_step.reset();
				}
			}
		}

		return (RecordingType)r;
	}
	virtual void endFrame(uint32_t frame_id) {
		{
			// Let's see which targets are still valid (didn't change in the mean time)
			std::unique_lock<std::mutex> lock(state_mutex);
			for (const auto & i : state) {
				auto s = i.second;
				if (s->drawing) {
					for (const auto & t : s->targets)
						if (hasTarget(t)) {
							if (!targetAvailable(t)) {
								LOG(INFO) << "Target '" << t << "' exists, but was not written. Check your plugins to make sure all targets are written properly!";
								continue;
							}
							// The client has not yet received the renter target
							s->targets_received[t] = false;

							// Send the message
							Message m(Message::FETCH_TARGET, s->fetch_id);

							// Write the header
							FetchTargetReplyHdr hdr = { 0 };
							hdr.data_type = outputType(t);
							hdr.C = outputChannels(t);
							hdr.W = s->target_w;
							hdr.H = s->target_h;
							hdr.frame_id = frame_id;
							hdr.timestamp = frame_timestamp;
							m.data.insert(m.data.end(), (uint8_t*)&hdr, (uint8_t*)(&hdr+1));

							// Write the target name
							appendString(t, m.data);

							// Data
							size_t DS = dataSize((DataType)hdr.data_type);
							if (DS == 0) LOG(ERR) << "Unknown data type!";
							size_t o = m.data.size();
							m.data.resize(o + (size_t)hdr.W * (size_t)hdr.H * (size_t)hdr.C * DS, 0);

							// Read the data
							readTarget(t, hdr.W, hdr.H, hdr.C, (DataType)hdr.data_type, m.data.data() + o);
							s->send.addMessage(m);
						}
				}
				const std::string & gs = getGameState();
				if (s->fetch_game_state && gs.size()) {
					Message m(Message::FETCH_GAME_STATE, s->game_state_id);
					m.data.resize(sizeof(FetchGameStateReply));
					FetchGameStateReply hdr = {frame_id, frame_timestamp};
					memcpy(m.data.data(), &hdr, sizeof(hdr));
					appendString(gs, m.data);
					s->send.addMessage(m);
				}
			}
		}
	}

	bool startServer() {
		WSADATA wsaData;
		// Initialize Winsock
		int r = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (r != 0) {
			LOG(WARN) << "WSAStartup failed with error: " << r;
			return false;
		}

		// Create a SOCKET for connecting to server
		listen_socket = socket(AF_INET, SOCK_STREAM, 0);
		if (listen_socket == INVALID_SOCKET) {
			LOG(WARN) << "socket failed with error: " << WSAGetLastError();
			WSACleanup();
			return false;
		}

		// Setup the TCP listening socket
		sockaddr_in server;
		server.sin_family = AF_INET;
		server.sin_addr.s_addr = INADDR_ANY;
		uint16_t port = DEFAULT_PORT;
		{
			char tmp[256];
			if (GetEnvironmentVariableA("SERVER_PORT", tmp, sizeof(tmp)))
				port = atoi(tmp);
		}
		server.sin_port = htons(port);

		r = bind(listen_socket, (struct sockaddr *)&server, sizeof(server));
		if (r == SOCKET_ERROR) {
			LOG(WARN) << "bind failed with error: " << WSAGetLastError();
			closesocket(listen_socket);
			WSACleanup();
			return false;
		}

		LOG(INFO) << "Server fired up @ " << server;
		listen(listen_socket, 3);
		running = true;
		th = CreateThread(NULL, 0, Server::_serve, this, 0, NULL);
		return true;
	}
	void shutdown() {
		if (running) {
			// Stop the server and all clients
			running = false;

			// Shut down
			closesocket(listen_socket);
			WSACleanup();
			// Join the main thread
			WaitForSingleObject(th, 1000);
		}
	}
	static DWORD WINAPI _serve(LPVOID that) {
		return ((Server *)that)->serve();
	}
	char buffer[MAX_BUFFER_SIZE];
	bool serve() {
		fd_set readfds;
		while (running) {
			//clear the socket fd set
			FD_ZERO(&readfds);

			//add master socket to fd set
			FD_SET(listen_socket, &readfds);
			for (auto s : client)
				if (s > 0)
					FD_SET(s, &readfds);

			//wait for an activity on any of the sockets, timeout is NULL , so wait indefinitely
			timeval tv = { 0, 1000 };
			int activity = select(0, &readfds, NULL, NULL, &tv);
			if (activity == SOCKET_ERROR) {
				LOG(WARN) << "select call failed with error code : " << WSAGetLastError();
				return false;
			}
			if (!running) break;

			//If something happened on the master socket , then its an incoming connection
			if (FD_ISSET(listen_socket, &readfds)) {
				sockaddr_in address;
				int addrlen = sizeof(address);
				SOCKET new_client = accept(listen_socket, (sockaddr *)&address, &addrlen);
				if (new_client == INVALID_SOCKET) {
					CHAR *s = NULL;
					int err = WSAGetLastError();
					FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
						NULL, err, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&s, 0, NULL);
					LOG(WARN) << "Failed to accept new socket: '" << s << "' (" << err << ")";
					LocalFree(s);
				} else {
					LOG(INFO) << "Client " << address << " connected.";

					// Let's not block for too long
					DWORD timeout = 100; // ms
					setsockopt(new_client, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout));
					// And make sure the send buffer has enough memory to hold all images even if the client doesn't immediately reads them
					int size = defaultHeight()*defaultWidth() * 32;
					setsockopt(new_client, SOL_SOCKET, SO_SNDBUF, (const char*)&size, sizeof(size));

					client.push_back(new_client);
					std::unique_lock<std::mutex> lock(state_mutex);
					state[new_client] = std::make_shared<ClientState>();
				}
			}
			if (!running) break;

			for (auto & s : client)
				if (FD_ISSET(s, &readfds)) {
					if (!running) break;

					sockaddr_in address;
					int addrlen = sizeof(sockaddr_in);
					getpeername(s, (sockaddr*)&address, (int*)&addrlen);

					// Get a request
					int valread = recv(s, buffer, MAX_BUFFER_SIZE, 0);
					if (valread == SOCKET_ERROR) {
						int error_code = WSAGetLastError();
						if (error_code == WSAECONNRESET || error_code == WSAECONNABORTED) {
							LOG(INFO) << "Host " << address << " disconnected unexpectedly (or timed out).";

							// Close the socket and mark as 0 in list for deletion
							closesocket(s);
							std::unique_lock<std::mutex> lock(state_mutex);
							state.erase(s);
							s = 0;
						} else {
							LOG(WARN) << "recv failed with error code : " << error_code;
						}
					} else if (valread == 0) {
						//Somebody disconnected , get his details and print
						LOG(INFO) << "Host " << address << " disconnected";

						//Close the socket and mark as 0 in list for reuse
						closesocket(s);
						std::unique_lock<std::mutex> lock(state_mutex);
						state.erase(s);
						s = 0;
					} else if (valread > 0) {
						// Handle the message
						state[s]->recv.recv(buffer, valread);
					}
				}

			if (!running) break;

			// Clean up all closed sockets
			for (auto i = client.begin(); i != client.end(); ) {
				if (!*i)
					i = client.erase(i);
				else
					++i;
			}

			// Handle all messages
			for (auto s : client)
				handleMessages(s, state[s]);

			// Send all the outgoing messages
			for (auto s : client)
				state[s]->send.sendAll(s);

			// Handle the key up
			handleKeyUp();
		}
		return true;
	}
};
REGISTER_CONTROLLER(Server);


BOOL WINAPI DllMain(HINSTANCE hInst, DWORD reason, LPVOID) {
	if (reason == DLL_PROCESS_ATTACH) {
		LOG(INFO) << "Server turned on";
	}

	if (reason == DLL_PROCESS_DETACH) {
		LOG(INFO) << "Server turned off";
	}
	return TRUE;
}
