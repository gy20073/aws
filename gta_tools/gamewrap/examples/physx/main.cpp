#define NOMINMAX
#include "log.h"
#include "sdk.h"
#include "../../minhook/include/MinHook.h"
#include "Windows.h"
// This plugin enable the nvidia physx debugger of a game using physx

#pragma comment(lib, "libMinHook.lib")
#pragma comment(lib, "C:\\Program Files (x86)\\AGEIA Technologies\\SDK\\v2.8.0\\SDKs\\lib\\Win32\\PhysXLoader.lib")

void EnsureMinHookInitialized() {
	// Ensure MH_Initialize is only called once.
	static LONG minhook_init_once = 0;
	if (InterlockedCompareExchange(&minhook_init_once, 1, 0) == 0) {
		MH_STATUS status = MH_Initialize();
		if (status != MH_OK && status != MH_ERROR_ALREADY_INITIALIZED)
			LOG(WARN) << "Failed to initialize minhook; MH_Initialize returned" << MH_StatusToString(status);
	}
}

template<typename T, typename... ARGS>
struct Hook {
	typedef T(*FT)(ARGS...);
	FT original_, func_;
	Hook() :original_(0), func_(0) {
		EnsureMinHookInitialized();
	}
	void setup(FT f, FT n) {
		func_ = f;
		MH_STATUS status = MH_CreateHook((LPVOID)func_, (LPVOID)n, (LPVOID *)&original_);
		if (status != MH_OK)
			LOG(WARN) << "Failed to setup minhook " << MH_StatusToString(status);
		this->enable();
	}
	void enable() {
		MH_STATUS status = MH_EnableHook((LPVOID)func_);
		if (status != MH_OK)
			LOG(WARN) << "Failed to enable minhook " << MH_StatusToString(status);
	}
	void disable() {
		MH_STATUS status = MH_DisableHook((LPVOID)func_);
		if (status != MH_OK)
			LOG(WARN) << "Failed to enable minhook " << MH_StatusToString(status);
	}
	T operator()(ARGS... args) {
		return original_(args...);
	}
	~Hook() {
		MH_STATUS status = MH_RemoveHook((LPVOID)func_);
		if (status != MH_OK)
			LOG(WARN) << "Failed to remove minhook " << MH_StatusToString(status);
	}
};


//#define NX_CALL_CONV __cdecl
//#define NX32
//typedef float NxF32;
//typedef double NxF64;
//typedef uint8_t NxU8;
//typedef uint16_t NxU16;
//typedef uint32_t NxU32;
//typedef uint64_t NxU64;
//typedef int8_t NxI8;
//typedef int16_t NxI16;
//typedef int32_t NxI32;
//typedef int64_t NxI64;
#define WIN32
#include <NxPhysics.h>
Hook<NxPhysicsSDK*, NxU32, NxUserAllocator*, NxUserOutputStream*, const NxPhysicsSDKDesc&, NxSDKCreateError*> hNxCreatePhysicsSDK;
NxPhysicsSDK* NX_CALL_CONV nNxCreatePhysicsSDK(NxU32 sdkVersion, NxUserAllocator* allocator = NULL, NxUserOutputStream* outputStream = NULL, const NxPhysicsSDKDesc& desc = NxPhysicsSDKDesc(), NxSDKCreateError* errorCode = NULL) {
	LOG(INFO) << "NxCreatePhysicsSDK";
	NxPhysicsSDK *SDK = hNxCreatePhysicsSDK(sdkVersion, allocator, outputStream, desc, errorCode);
	if (SDK != NULL)
		SDK->getFoundationSDK().getRemoteDebugger()->connect("localhost", 5425);
	return SDK;
}


BOOL WINAPI DllMain(HINSTANCE hInst, DWORD reason, LPVOID) {
	if (reason == DLL_PROCESS_ATTACH) {
		LOG(INFO) << "Physx hook turned on";
		hNxCreatePhysicsSDK.setup(NxCreatePhysicsSDK, nNxCreatePhysicsSDK);
	}

	if (reason == DLL_PROCESS_DETACH) {
		LOG(INFO) << "Physx hook turned off";
		hNxCreatePhysicsSDK.disable();
	}
	return TRUE;
}
