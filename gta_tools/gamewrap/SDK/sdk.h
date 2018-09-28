#pragma once
#include <cstdint>
#include <string>
#include <sstream>
#include <vector>
#include <memory>
#include <unordered_map>

#ifndef IMPORT
#define IMPORT __declspec(dllimport)
#endif

// A few notes on memory handling:
//  Sometimes callback functions are asked to return an object,
//  don't worry about that object leaking, the memory handling
//  will be taken care of in the background. In fact, please do
//  not delete any of these objects yourself, nor keep pointers
//  to these objects in your code! Copy them if needed.


/********************************************************/
/** Shader object                                      **/
/********************************************************/

typedef std::vector<uint8_t> ByteCode;

struct ShaderHash {
	uint32_t h[4] = { 0 };
	bool operator==(const ShaderHash & o) const {
		for (int i = 0; i < 4; i++)
			if (h[i] != o.h[i])
				return false;
		return true;
	}
	bool operator!=(const ShaderHash & o) const {
		for (int i = 0; i < 4; i++)
			if (h[i] != o.h[i])
				return true;
		return false;
	}
	operator bool() const {
		return h[0] || h[1] || h[2] || h[3];
	}
	operator std::string() const {
		std::ostringstream o;
		o << *this;
		return o.str();
	}
	ShaderHash & operator=(const std::string & s) {
		std::istringstream(s) >> *this;
		return *this;
	}
	ShaderHash(const std::string & s) { *this = s; }
	ShaderHash() = default;
};
namespace std {
	template<> struct hash<ShaderHash> {
		size_t operator()(const ShaderHash & h) const {
			std::hash<uint32_t> hasher;
			size_t r = 0;
			for (int i = 0; i<4; i++)
				r ^= hasher(h.h[i]) + 0x9e3779b9 + (r << 6) + (r >> 2);
			return r;
		}
	};
}
IMPORT std::ostream & operator<<(std::ostream & o, const ShaderHash & h);
IMPORT std::istream & operator>>(std::istream & o, ShaderHash & h);

class Shader {
public:
	enum Type {
		UNKNOWN = -1,
		VERTEX = 0,
		PIXEL = 1,
		COMPUTE = 2,
	};
protected:
	Shader() = default;
	Shader(const Shader &) = delete;
	Shader& operator=(const Shader &) = delete;
public:
	~Shader(){}
	struct Buffer {
		struct Variable {
			std::string name, type;
			uint32_t offset = 0;
		};
		std::string name;
		uint32_t bind_point;
		std::vector<Variable> variables;
	};
	struct Binding {
		std::string name;
		uint32_t bind_point;
	};
	static IMPORT std::shared_ptr<Shader> merge(std::shared_ptr<Shader> a, std::shared_ptr<Shader> b);
	static IMPORT std::shared_ptr<Shader> compile(const std::string & src, Type type, std::string * err);
	// name_remap is used to map output names (e.g. SV_Target7) to semantically meaningful names (e.g. depth)
	static IMPORT std::shared_ptr<Shader> create(const ByteCode & src, const std::unordered_map<std::string, std::string> & name_remap = std::unordered_map<std::string, std::string>());

	// Shader subset and OI renaming
	virtual std::shared_ptr<Shader> subset(const std::vector<uint32_t> & outputs) const = 0;
	virtual std::shared_ptr<Shader> subset(const std::vector<std::string> & named_outputs) const = 0;
	virtual void renameCBuffer(const std::string & old_name, const std::string & new_name, int new_slot = -1) = 0;
	virtual void renameOutput(const std::string & old_name, const std::string & new_name, int sys_id=0) = 0;

	virtual std::string disassemble() const = 0;

	// Shader info
	virtual Type type() const = 0;
	virtual const ShaderHash & hash() const = 0;
	
	virtual const std::vector<Binding> & inputs() const = 0;
	virtual const std::vector<Binding> & outputs() const = 0;
	virtual const std::vector<Buffer> & cbuffers() const = 0;
	virtual const std::vector<Buffer> & sbuffers() const = 0;
	virtual const std::vector<Binding> & textures() const = 0;

	// Shader bytecode and length
	virtual const uint8_t * data() const = 0;
	virtual size_t size() const = 0;
};
IMPORT std::ostream & operator<<(std::ostream & o, const Shader::Buffer & h);
IMPORT std::ostream & operator<<(std::ostream & o, const Shader::Buffer::Variable & h);


class CBuffer {
protected:
	virtual void write(const void * data, size_t offset, size_t size) = 0;
	CBuffer() = default;
	CBuffer(const CBuffer &) = delete;
	CBuffer& operator=(const CBuffer &) = delete;
public:
	virtual ~CBuffer() {}
	template<typename T> void set(T v, size_t offset = 0) {
		write(&v, offset, sizeof(T));
	}
	template<typename T> void set(const T* v, size_t n, size_t offset = 0) {
		write(v, offset, sizeof(T)*n);
	}
	template<typename T> void set(const std::vector<T>& v, size_t offset = 0) {
		write(v.data(), offset, sizeof(T)*v.size());
	}
};

/********************************************************/
/** Game Controller                                    **/
/********************************************************/

struct Resource {
	uint32_t id = 0;
	Resource(uint32_t id = 0) :id(id) {}
	operator bool() const { return id; }
	operator uint32_t() const { return id; }
	Resource & operator=(uint32_t i) { id = i; return *this; }
};
struct Texture2D : public Resource { using Resource::Resource; };
struct RenderTargetView : public Resource { using Resource::Resource; uint16_t W=0, H=0; };
struct DepthStencilView : public Resource { using Resource::Resource; uint16_t W = 0, H = 0; };
struct Buffer : public Resource { using Resource::Resource; };

// GPUMemory objects are used to fetch GPU data during a render pass, there are two different types of GPUMemory objects:
//  * Immediate objects read the contents of the memory as soon as they are created
//  * Delayed objects do not contain any data until the frame is completely rendered (postProcess/endFrame), where all data is fetched from the GPU
// Delayed objects are much more efficient and thus preferred. All GPUMemory is cleared after the current frame and will not survive past endFrame!
struct GPUMemory {
	virtual ~GPUMemory() {}
	// Pointer to the data (on the host/CPU), null if the memory is not accessible
	virtual const void * data() const = 0;
	virtual size_t size() const = 0;
	virtual std::shared_ptr<GPUMemory> sub(size_t o, size_t n) = 0;
	operator bool() const { return data(); }
	template<typename T> const T * data() const { return static_cast<const T*>(data()); }
};

struct DrawInfo {
	enum Type {
		VERTEX,
		INDEX,
	};
	uint8_t type;
	uint16_t instances;
	uint32_t n; // Number of vertices or indices
	uint32_t start_index, start_vertex, start_instance;
	Texture2D texture_id;
	Buffer vertex_buffer;
	Buffer index_buffer;
	ShaderHash vertex_shader, pixel_shader;
	std::vector<RenderTargetView> outputs;
	DepthStencilView depth_output;
	std::vector<Buffer> vs_cbuffers;
	std::vector<Buffer> ps_cbuffers;
};
enum DrawType {
	DEFAULT = 0,
	STATIC,
	RIGID,
	DYNAMIC,
	HIDE,
};
enum RecordingType {
	NONE = 0,
	PREDRAW = 1,
	DRAW = 2,
	PREDRAW_AND_DRAW = 3,
};
enum TargetType {
	UNKNOWN = 0,
	R32G32B32A32_TYPELESS = 1,
	R32G32B32A32_FLOAT = 2,
	R32G32B32A32_UINT = 3,
	R32G32B32A32_SINT = 4,
	R32G32B32_TYPELESS = 5,
	R32G32B32_FLOAT = 6,
	R32G32B32_UINT = 7,
	R32G32B32_SINT = 8,
	R16G16B16A16_TYPELESS = 9,
	R16G16B16A16_FLOAT = 10,
	R16G16B16A16_UNORM = 11,
	R16G16B16A16_UINT = 12,
	R16G16B16A16_SNORM = 13,
	R16G16B16A16_SINT = 14,
	R32G32_TYPELESS = 15,
	R32G32_FLOAT = 16,
	R32G32_UINT = 17,
	R32G32_SINT = 18,
	R32G8X24_TYPELESS = 19,
	D32_FLOAT_S8X24_UINT = 20,
	R32_FLOAT_X8X24_TYPELESS = 21,
	X32_TYPELESS_G8X24_UINT = 22,
	R10G10B10A2_TYPELESS = 23,
	R10G10B10A2_UNORM = 24,
	R10G10B10A2_UINT = 25,
	R11G11B10_FLOAT = 26,
	R8G8B8A8_TYPELESS = 27,
	R8G8B8A8_UNORM = 28,
	R8G8B8A8_UNORM_SRGB = 29,
	R8G8B8A8_UINT = 30,
	R8G8B8A8_SNORM = 31,
	R8G8B8A8_SINT = 32,
	R16G16_TYPELESS = 33,
	R16G16_FLOAT = 34,
	R16G16_UNORM = 35,
	R16G16_UINT = 36,
	R16G16_SNORM = 37,
	R16G16_SINT = 38,
	R32_TYPELESS = 39,
	D32_FLOAT = 40,
	R32_FLOAT = 41,
	R32_UINT = 42,
	R32_SINT = 43,
	R24G8_TYPELESS = 44,
	D24_UNORM_S8_UINT = 45,
	R24_UNORM_X8_TYPELESS = 46,
	X24_TYPELESS_G8_UINT = 47,
	R8G8_TYPELESS = 48,
	R8G8_UNORM = 49,
	R8G8_UINT = 50,
	R8G8_SNORM = 51,
	R8G8_SINT = 52,
	R16_TYPELESS = 53,
	R16_FLOAT = 54,
	D16_UNORM = 55,
	R16_UNORM = 56,
	R16_UINT = 57,
	R16_SNORM = 58,
	R16_SINT = 59,
	R8_TYPELESS = 60,
	R8_UNORM = 61,
	R8_UINT = 62,
	R8_SNORM = 63,
	R8_SINT = 64,
	A8_UNORM = 65,
	R1_UNORM = 66,
	R9G9B9E5_SHAREDEXP = 67,
	R8G8_B8G8_UNORM = 68,
	G8R8_G8B8_UNORM = 69,
	BC1_TYPELESS = 70,
	BC1_UNORM = 71,
	BC1_UNORM_SRGB = 72,
	BC2_TYPELESS = 73,
	BC2_UNORM = 74,
	BC2_UNORM_SRGB = 75,
	BC3_TYPELESS = 76,
	BC3_UNORM = 77,
	BC3_UNORM_SRGB = 78,
	BC4_TYPELESS = 79,
	BC4_UNORM = 80,
	BC4_SNORM = 81,
	BC5_TYPELESS = 82,
	BC5_UNORM = 83,
	BC5_SNORM = 84,
	B5G6R5_UNORM = 85,
	B5G5R5A1_UNORM = 86,
	B8G8R8A8_UNORM = 87,
	B8G8R8X8_UNORM = 88,
	R10G10B10_XR_BIAS_A2_UNORM = 89,
	B8G8R8A8_TYPELESS = 90,
	B8G8R8A8_UNORM_SRGB = 91,
	B8G8R8X8_TYPELESS = 92,
	B8G8R8X8_UNORM_SRGB = 93,
	BC6H_TYPELESS = 94,
	BC6H_UF16 = 95,
	BC6H_SF16 = 96,
	BC7_TYPELESS = 97,
	BC7_UNORM = 98,
	BC7_UNORM_SRGB = 99,
	AYUV = 100,
	Y410 = 101,
	Y416 = 102,
	NV12 = 103,
	P010 = 104,
	P016 = 105,
	_420_OPAQUE = 106,
	YUY2 = 107,
	Y210 = 108,
	Y216 = 109,
	NV11 = 110,
	AI44 = 111,
	IA44 = 112,
	P8 = 113,
	A8P8 = 114,
	B4G4R4A4_UNORM = 115,
	P208 = 130,
	V208 = 131,
	V408 = 132,
	FORCE_UINT = 0xffffffff
};
enum BindTarget {
	COLOR0 = 0,
	COLOR1 = 1,
	COLOR2 = 2,
	COLOR3 = 3,
	COLOR4 = 4,
	COLOR5 = 5,
	COLOR6 = 6,
	COLOR7 = 7,
	UNBIND = -1,
};
enum DataType {
	DT_UINT8 = 0,
	DT_UINT16 = 1,
	DT_UINT32 = 2,
	DT_FLOAT = 3,
	DT_HALF = 4,
	DT_UNKNOWN = -1,
};
enum ExtendedDataType {
	DT_STRUCT = 5,
	DT_VECTOR = 6,
};
IMPORT size_t dataSize(DataType t);

struct ProvidedTarget {
	std::string name;
	TargetType type = TargetType::UNKNOWN;
	bool hidden = false;
};

// Standard toJSON conversions
inline std::string toJSON(const std::string & s) { return "\"" + s + "\""; }
inline std::string toJSON(int v) { return std::to_string(v); }
inline std::string toJSON(float v) { return std::to_string(v); }
template<typename T> std::string toJSON(const std::vector<T> & v) {
	std::string r = "[";
	for (auto i : v)
		r += (r.size() == 1 ? "" : ", ") + toJSON(i);
	return r + "]";
}
#define STR(s) #s
#define _TOJSON(x) {if (r.size()>1) r += std::string(","); r += "\"" + std::string(STR(x)) + "\":" + toJSON(t.x); }
// Helper function to do a JSON conversion of a game state (list the class and all variables you want to export)
#define TOJSON(T, ...) inline std::string toJSON(const T & t) { std::string r = "{"; FOR_EACH(_TOJSON, __VA_ARGS__); return r + "}";};


// Interface implementation of the game controller
class BaseGameController {
private: // Callback functions (they can be overwritten, but shouldn't be called directly)
	friend class MainGameController;
	/******* IO *******/
	// IO Callback functions (return true swallows the callback, no other functions are called)
	virtual bool keyDown(unsigned char key, unsigned char special_status) { return false; }
	virtual bool keyUp(unsigned char key) { return false; }

	/******* Rendering *******/
	// Frame callback functions (first figure out if we want to record the frame, then start it, then start and endDraw calls, finally endFrame
	virtual RecordingType recordFrame(uint32_t frame_id) { return NONE; }
	virtual void startFrame(uint32_t frame_id) {}
	virtual void postProcess(uint32_t frame_id) {}
	virtual void endFrame(uint32_t frame_id) {}

	// Draw callback functions
	virtual DrawType startDraw(const DrawInfo & i) { return DEFAULT; }
	virtual void endDraw(const DrawInfo & i) {}

	// Shader injection (optional)
	// This is the main way to control the rendering pipeline, in addition to all the vertex and index buffers as well as cbuffers and texture, you get one cbuffer which you can write content into
	// simply call the cbuffer "InjectBuffer" and assign it to a register of your choosing (you have to make sure there is no collision of cbuffer registers!)
	// The cbuffer InjectBuffer will contain a single int base_id; field and any content you append
	virtual std::shared_ptr<Shader> injectShader(std::shared_ptr<Shader> shader) { return std::shared_ptr<Shader>(); }

	// Target listings (not this value should NOT change during the execution of your program, you should also NOT write any targets you do not provide)
	virtual std::vector<ProvidedTarget> providedTargets() const { return std::vector<ProvidedTarget>(); }
	// Custom targets are render targets that are bound to shaders and cannot be copied into, the shader binding happens according to name
	// To bind a custom render target simply use a RWTexture2D<...> name; in your injected pixel shader.
	virtual std::vector<ProvidedTarget> providedCustomTargets() const { return std::vector<ProvidedTarget>(); }

	// Game state functions (callbacks)
	// Only one plugin should provide a gamestate, in addition please always provide the same game state (DO NOT SWITCH!)
	virtual std::string gameState() const { return ""; }
	virtual bool resetGame() { return false; }
	virtual bool stop() { return true; }
protected:
	BaseGameController(const BaseGameController&) = delete;
	BaseGameController& operator=(const BaseGameController&) = delete;
	BaseGameController() = default;
public:
	typedef int16_t half;

	virtual ~BaseGameController() = default;

	/******* IO *******/
	// IO Control functions (not not override unless you know what you're doing!)
	virtual void sendKeyDown(unsigned char key, bool syskey = false) = 0;
	virtual void sendKeyUp(unsigned char key, bool syskey = false) = 0;
	virtual const std::vector<uint8_t> & keyState() const = 0;
	// x and y are in screen space coordinates -1..1 and button: 1 left, 2 right, 3 middle
	virtual void sendMouseDown(float x, float y, uint8_t button) = 0;
	virtual void sendMouseUp(float x, float y, uint8_t button) = 0;

	/******* Rendering *******/
	// Recording type
	virtual RecordingType currentRecordingType() const = 0;

	// Output functions
	virtual void copyTarget(const std::string & to, const std::string & from) = 0;
	virtual void copyTarget(const std::string & name, const RenderTargetView & rt) = 0;
	virtual std::vector<std::string> listTargets() const = 0;
	virtual TargetType targetType(const std::string & name) const = 0;
	virtual bool hasTarget(const std::string & name) const = 0;

	// Is the target ready to be read? Was it written in the last frame?
	virtual bool targetAvailable(const std::string & name) const = 0;
	
	virtual int defaultWidth() const = 0;
	virtual int defaultHeight() const = 0;

	// If you want to fetch a specific output request it in the startFrame function (or any time before the target is written, not after!)
	// The request needs to be renewed for every new frame
	virtual void requestOutput(const std::string & name) = 0;
	virtual void requestOutput(const std::string & name, int W, int H) = 0;

	// What type does a certain output channel have?
	virtual DataType outputType(const std::string & name) const = 0;
	virtual int outputChannels(const std::string & name) const = 0;

	// You can only read targets in the endFrame function, calling it from anywhere else leads to undefined behavior
	// try to match the datatype it's more efficient
	virtual bool readTarget(const std::string & name, int W, int H, int C, DataType t, void * data) = 0;
	virtual bool readTarget(const std::string & name, int W, int H, int C, half * data) = 0;
	virtual bool readTarget(const std::string & name, int W, int H, int C, float * data) = 0;
	virtual bool readTarget(const std::string & name, int W, int H, int C, uint8_t * data) = 0;
	virtual bool readTarget(const std::string & name, int W, int H, int C, uint16_t * data) = 0;
	virtual bool readTarget(const std::string & name, int W, int H, int C, uint32_t * data) = 0;

	// Buffer handling and reading
	virtual std::shared_ptr<GPUMemory> readBuffer(Buffer b, size_t n, bool immediate = false) { return readBuffer(b, 0, n, immediate); }
	virtual std::shared_ptr<GPUMemory> readBuffer(Buffer b, size_t offset, size_t n, bool immediate = false) { return readBuffer(b, std::vector<size_t>{ offset }, std::vector<size_t>{ n }, immediate); }
	virtual std::shared_ptr<GPUMemory> readBuffer(Buffer b, const std::vector<size_t> & offset, const std::vector<size_t> & n, bool immediate = false) = 0;
	virtual size_t bufferSize(Buffer b) = 0;
	
	// Game state query functions
	virtual std::string getGameState() const = 0;
	virtual bool requestGameReset() = 0;

	// CBuffer handling
	virtual std::shared_ptr<CBuffer> createCBuffer(const std::string & name, size_t max_size) = 0;
	// Bind the cbuffer to the next draw call only. This function call only has an effect within startDraw, and has no effect anywhere else
	virtual void bindCBuffer(std::shared_ptr<CBuffer> b) = 0;

	// Postprocessing functions
	virtual void callPostFx(std::shared_ptr<Shader> shader) = 0;
};

// This is the main wrapper class. To use the game wrapper inherit from this class
// and register your own class using REGISTER_WRAPPER . More than one wrapper may
// be registered at any given time.
class GameController: public BaseGameController {
protected:
	BaseGameController * main_;
	// Contructor
	template<typename T> friend struct TGameControllerFactory;
public:
	virtual void sendKeyDown(unsigned char key, bool syskey = false) final { return main_->sendKeyDown(key, syskey); }
	virtual void sendKeyUp(unsigned char key, bool syskey = false) final { return main_->sendKeyUp(key, syskey); }
	virtual const std::vector<uint8_t> & keyState() const final { return main_->keyState(); }
	virtual void sendMouseDown(float x, float y, uint8_t button) { return main_->sendMouseDown(x, y, button); }
	virtual void sendMouseUp(float x, float y, uint8_t button) { return main_->sendMouseUp(x, y, button); }

	virtual RecordingType currentRecordingType() const final { return main_->currentRecordingType(); }

	virtual void copyTarget(const std::string & to, const std::string & from) final { main_->copyTarget(to, from); }
	virtual void copyTarget(const std::string & name, const RenderTargetView & rt) final { main_->copyTarget(name, rt); }
	virtual std::vector<std::string> listTargets() const final { return main_->listTargets(); }
	virtual TargetType targetType(const std::string & name) const final { return main_->targetType(name); }
	virtual bool hasTarget(const std::string & name) const final { return main_->hasTarget(name); }
	virtual bool targetAvailable(const std::string & name) const final { return main_->targetAvailable(name); }

	virtual int defaultWidth() const final { return main_->defaultWidth(); }
	virtual int defaultHeight() const final { return main_->defaultHeight(); }

	// If you want to fetch a specific output request it in the startDraw function (or any time before the target is written, not after!)
	// The request needs to be renewed for every new frame
	virtual void requestOutput(const std::string & name) final { main_->requestOutput(name); }
	virtual void requestOutput(const std::string & name, int W, int H) final { main_->requestOutput(name, W, H); }

	// What type does a certain output channel have?
	virtual DataType outputType(const std::string & name) const final { return main_->outputType(name); }
	virtual int outputChannels(const std::string & name) const final { return main_->outputChannels(name); }

	// You can only read targets in the endFrame function, calling it from anywhere else leads to undefined behavior
	// You need to match the datatype, or else an error will be thrown
	virtual bool readTarget(const std::string & name, int W, int H, int C, DataType t, void * data) final { return main_->readTarget(name, W, H, C, t, data); }
	virtual bool readTarget(const std::string & name, int W, int H, int C, half * data) final { return main_->readTarget(name, W, H, C, data); }
	virtual bool readTarget(const std::string & name, int W, int H, int C, float * data) final { return main_->readTarget(name, W, H, C, data); }
	virtual bool readTarget(const std::string & name, int W, int H, int C, uint8_t * data) final { return main_->readTarget(name, W, H, C, data); }
	virtual bool readTarget(const std::string & name, int W, int H, int C, uint16_t * data) final { return main_->readTarget(name, W, H, C, data); }
	virtual bool readTarget(const std::string & name, int W, int H, int C, uint32_t * data) final { return main_->readTarget(name, W, H, C, data); }

	// Buffer handling and reading
	using BaseGameController::readBuffer;
	virtual std::shared_ptr<GPUMemory> readBuffer(Buffer b, const std::vector<size_t> & offset, const std::vector<size_t> & n, bool immediate = false) final { return main_->readBuffer(b, offset, n, immediate); }
	virtual size_t bufferSize(Buffer b) final { return main_->bufferSize(b); }

	// Game state query functions
	virtual std::string getGameState() const final { return main_->getGameState(); }
	virtual bool requestGameReset() final { return main_->requestGameReset(); }

	// CBuffer handling
	virtual std::shared_ptr<CBuffer> createCBuffer(const std::string & name, size_t max_size) final { return main_->createCBuffer(name, max_size); }
	virtual void bindCBuffer(std::shared_ptr<CBuffer> b) final { main_->bindCBuffer(b); }

	virtual void callPostFx(std::shared_ptr<Shader> shader) final { main_->callPostFx(shader); }
};


// Feel free to ignore anything below this line
struct GameControllerFactory {
	GameControllerFactory() = default;
	GameControllerFactory(const GameControllerFactory&) = delete;
	GameControllerFactory& operator=(const GameControllerFactory&) = delete;
	virtual ~GameControllerFactory() = default;
	virtual GameController* make(BaseGameController *c) = 0;
};
template<typename T> struct TGameControllerFactory: public GameControllerFactory {
	virtual GameController* make(BaseGameController *main) override {
		GameController * w = new T;
		w->main_ = main;
		return w;
	}
};

IMPORT size_t registerFactory(GameControllerFactory * h, int priority=0);
IMPORT size_t unregisterFactory(GameControllerFactory * h);
template<typename T>
struct FactoryRegistry {
	FactoryRegistry& operator=(const FactoryRegistry &) = delete;
	FactoryRegistry(const FactoryRegistry &) = delete;
	GameControllerFactory * f;
	FactoryRegistry(int P) :f(new TGameControllerFactory<T>()) { registerFactory(f, P); }
	~FactoryRegistry() { unregisterFactory(f); }
};
template<typename T> static size_t REG(int P=0) {
	static FactoryRegistry<T> reg(P);
	return !!reg.f;
}
#define REGISTER_CONTROLLER(C) static size_t reg_##C = REG<C>();
#define REGISTER_CONTROLLER_PRIORITY(C, P) static size_t reg_##C = REG<C>(P);


/* Only the brave continue past this point! If there is a bug in the code below, good luck finding it (you might want to enable the preprocessor output of your compiler and read that file) */
// Note to future self: I'm sorry!

// Macro foreach
// Thank you MSVC! This expansion macro makes soo much more sense than the gcc interpretation of __VA__ARGS__
#define E(x) x
#define VA_COUNT(...) E(VA_COUNT_H(__VA_ARGS__, 64, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1,))
#define VA_COUNT_H(e0, e1, e2, e3, e4, e5, e6, e7, e8, e9, e10, e11, e12, e13, e14, e15, e16, e17, e18, e19, e20, e21, e22, e23, e24, e25, e26, e27, e28, e29, e30, e31, e32, e33, e34, e35, e36, e37, e38, e39, e40, e41, e42, e43, e44, e45, e46, e47, e48, e49, e50, e51, e52, e53, e54, e55, e56, e57, e58, e59, e60, e61, e62, e63, size, ...) size

#define __FE_1(f, x) f(x);
#define __FE_2(f, x, ...) f(x); E(__FE_1(f, __VA_ARGS__))
#define __FE_3(f, x, ...) f(x); E(__FE_2(f, __VA_ARGS__))
#define __FE_4(f, x, ...) f(x); E(__FE_3(f, __VA_ARGS__))
#define __FE_5(f, x, ...) f(x); E(__FE_4(f, __VA_ARGS__))
#define __FE_6(f, x, ...) f(x); E(__FE_5(f, __VA_ARGS__))
#define __FE_7(f, x, ...) f(x); E(__FE_6(f, __VA_ARGS__))
#define __FE_8(f, x, ...) f(x); E(__FE_7(f, __VA_ARGS__))
#define __FE_9(f, x, ...) f(x); E(__FE_8(f, __VA_ARGS__))
#define __FE_10(f, x, ...) f(x); E(__FE_9(f, __VA_ARGS__))
#define __FE_11(f, x, ...) f(x); E(__FE_10(f, __VA_ARGS__))
#define __FE_12(f, x, ...) f(x); E(__FE_11(f, __VA_ARGS__))
#define __FE_13(f, x, ...) f(x); E(__FE_12(f, __VA_ARGS__))
#define __FE_14(f, x, ...) f(x); E(__FE_13(f, __VA_ARGS__))
#define __FE_15(f, x, ...) f(x); E(__FE_14(f, __VA_ARGS__))
#define __FE_16(f, x, ...) f(x); E(__FE_15(f, __VA_ARGS__))
#define __FE_17(f, x, ...) f(x); E(__FE_16(f, __VA_ARGS__))
#define __FE_18(f, x, ...) f(x); E(__FE_17(f, __VA_ARGS__))
#define __FE_19(f, x, ...) f(x); E(__FE_18(f, __VA_ARGS__))
#define __FE_20(f, x, ...) f(x); E(__FE_19(f, __VA_ARGS__))
#define __FE_21(f, x, ...) f(x); E(__FE_20(f, __VA_ARGS__))
#define __FE_22(f, x, ...) f(x); E(__FE_21(f, __VA_ARGS__))
#define __FE_23(f, x, ...) f(x); E(__FE_22(f, __VA_ARGS__))
#define __FE_24(f, x, ...) f(x); E(__FE_23(f, __VA_ARGS__))
#define __FE_25(f, x, ...) f(x); E(__FE_24(f, __VA_ARGS__))
#define __FE_26(f, x, ...) f(x); E(__FE_25(f, __VA_ARGS__))
#define __FE_27(f, x, ...) f(x); E(__FE_26(f, __VA_ARGS__))
#define __FE_28(f, x, ...) f(x); E(__FE_27(f, __VA_ARGS__))
#define __FE_29(f, x, ...) f(x); E(__FE_28(f, __VA_ARGS__))
#define __FE_30(f, x, ...) f(x); E(__FE_29(f, __VA_ARGS__))
#define __FE_31(f, x, ...) f(x); E(__FE_30(f, __VA_ARGS__))
#define __FE_32(f, x, ...) f(x); E(__FE_31(f, __VA_ARGS__))
#define __FE_33(f, x, ...) f(x); E(__FE_32(f, __VA_ARGS__))
#define __FE_34(f, x, ...) f(x); E(__FE_33(f, __VA_ARGS__))
#define __FE_35(f, x, ...) f(x); E(__FE_34(f, __VA_ARGS__))
#define __FE_36(f, x, ...) f(x); E(__FE_35(f, __VA_ARGS__))
#define __FE_37(f, x, ...) f(x); E(__FE_36(f, __VA_ARGS__))
#define __FE_38(f, x, ...) f(x); E(__FE_37(f, __VA_ARGS__))
#define __FE_39(f, x, ...) f(x); E(__FE_38(f, __VA_ARGS__))
#define __FE_40(f, x, ...) f(x); E(__FE_39(f, __VA_ARGS__))
#define __FE_41(f, x, ...) f(x); E(__FE_40(f, __VA_ARGS__))
#define __FE_42(f, x, ...) f(x); E(__FE_41(f, __VA_ARGS__))
#define __FE_43(f, x, ...) f(x); E(__FE_42(f, __VA_ARGS__))
#define __FE_44(f, x, ...) f(x); E(__FE_43(f, __VA_ARGS__))
#define __FE_45(f, x, ...) f(x); E(__FE_44(f, __VA_ARGS__))
#define __FE_46(f, x, ...) f(x); E(__FE_45(f, __VA_ARGS__))
#define __FE_47(f, x, ...) f(x); E(__FE_46(f, __VA_ARGS__))
#define __FE_48(f, x, ...) f(x); E(__FE_47(f, __VA_ARGS__))
#define __FE_49(f, x, ...) f(x); E(__FE_48(f, __VA_ARGS__))
#define __FE_50(f, x, ...) f(x); E(__FE_49(f, __VA_ARGS__))
#define __FE_51(f, x, ...) f(x); E(__FE_50(f, __VA_ARGS__))
#define __FE_52(f, x, ...) f(x); E(__FE_51(f, __VA_ARGS__))
#define __FE_53(f, x, ...) f(x); E(__FE_52(f, __VA_ARGS__))
#define __FE_54(f, x, ...) f(x); E(__FE_53(f, __VA_ARGS__))
#define __FE_55(f, x, ...) f(x); E(__FE_54(f, __VA_ARGS__))
#define __FE_56(f, x, ...) f(x); E(__FE_55(f, __VA_ARGS__))
#define __FE_57(f, x, ...) f(x); E(__FE_56(f, __VA_ARGS__))
#define __FE_58(f, x, ...) f(x); E(__FE_57(f, __VA_ARGS__))
#define __FE_59(f, x, ...) f(x); E(__FE_58(f, __VA_ARGS__))
#define __FE_60(f, x, ...) f(x); E(__FE_59(f, __VA_ARGS__))
#define __FE_61(f, x, ...) f(x); E(__FE_60(f, __VA_ARGS__))
#define __FE_62(f, x, ...) f(x); E(__FE_61(f, __VA_ARGS__))
#define __FE_63(f, x, ...) f(x); E(__FE_62(f, __VA_ARGS__))
#define __FE_64(f, x, ...) f(x); E(__FE_63(f, __VA_ARGS__))

#define CONCATENATE(arg1, arg2)   CONCATENATE1(arg1, arg2)
#define CONCATENATE1(arg1, arg2)  CONCATENATE2(arg1, arg2)
#define CONCATENATE2(arg1, arg2)  arg1##arg2
#define __FE_N(N, f, ...) E(CONCATENATE(__FE_, N)(f, __VA_ARGS__))
#define FOR_EACH(f, ...) E(__FE_N(VA_COUNT(__VA_ARGS__), f, __VA_ARGS__))
