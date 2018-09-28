#include <windows.h>
#include "log.h"
#include "sdk.h"

#include <chrono>
double time() {
	auto now = std::chrono::system_clock::now();
	return std::chrono::duration<double>(now.time_since_epoch()).count();
}

struct KeyLogger : public GameController {
	KeyLogger(): GameController() {
	}
	double up_time = 0;
	virtual bool keyDown(unsigned char key, unsigned char special_status) {
		LOG(INFO) << "Key down " << key;
		if (key == VK_F6) {
			sendKeyDown(VK_UP);
			up_time = time() + 1;
		}
		return false;
	}
	virtual void endFrame(uint32_t frame_id) {
		if (up_time > 0 && up_time < time()) {
			sendKeyUp(VK_UP);
			up_time = 0;
		}
	}
	virtual bool keyUp(unsigned char key) {
		LOG(INFO) << "Key up " << key;
		return false;
	}
};
REGISTER_CONTROLLER(KeyLogger);


BOOL WINAPI DllMain(HINSTANCE hInst, DWORD reason, LPVOID) {
	if (reason == DLL_PROCESS_ATTACH) {
		LOG(INFO) << "Keylogger turned on";
	}

	if (reason == DLL_PROCESS_DETACH) {
		LOG(INFO) << "Keylogger turned off";
	}
	return TRUE;
}