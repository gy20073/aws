#include <windows.h>
#include <unordered_map>
#include "log.h"
#include "sdk.h"
#include <chrono>
#include <iomanip>

double time() {
	auto now = std::chrono::system_clock::now();
	return std::chrono::duration<double>(now.time_since_epoch()).count();
}
const double STAT_RATE = 3; // Show stats every 3 sec

struct DrawStatistics {
	uint64_t draw_calls = 0, draw_vertices = 0, n_frames = 0;
	double draw_time = 0, draw_start = 0;

	virtual void reset() {
		draw_calls = draw_vertices = n_frames = 0;
		draw_time = draw_start = 0;
	}

	virtual void startFrame(uint32_t frame_id) {
		draw_start = time();
		n_frames++;
	}
	virtual void endFrame(uint32_t frame_id) {
		draw_time += time() - draw_start;
	}
	virtual void startDraw(const DrawInfo & info) {
		draw_calls += 1;
		draw_vertices += info.n;
	}
};
std::ostream & operator<<(std::ostream & o, const DrawStatistics & s) {
	double nrm = 1. / s.n_frames;
	return o << "FPS = " << std::setprecision(1) << std::fixed << s.n_frames / s.draw_time << "   draw_calls = " << s.draw_calls * nrm << "   draw_vertices = " << s.draw_vertices * nrm;
}

struct DrawStats : public GameController {
	DrawStats() : GameController() {
	}
	int shader_info = 0;

	virtual bool keyDown(unsigned char key, unsigned char special_status) {
		if (key == VK_F1) {
			LOG(INFO) << "Dumping used shaders";
			return shader_info = 2;
		}
		return false;
	}
	DrawStatistics stat;
	double last_stat = 0;
	virtual void startFrame(uint32_t frame_id) {
		// Reset the stats
		stat.startFrame(frame_id);
	}
	virtual void endFrame(uint32_t frame_id) {
		stat.endFrame(frame_id);
		if (time() > last_stat + STAT_RATE) {
			last_stat = time();
			LOG(INFO) << "Draw stats: " << stat;
			stat.reset();
		}
		if (shader_info)
			shader_info -= 1;
	}
	std::unordered_map<ShaderHash, std::string> disass;
	virtual DrawType startDraw(const DrawInfo & info) {
		stat.startDraw(info);
		if (info.outputs.size()) {
			if (info.outputs[0].W == defaultWidth() && info.outputs[0].H == defaultHeight()) {
				if (shader_info == 1) {
					if (disass.count(info.vertex_shader)) {
						LOG(INFO) << disass[info.vertex_shader];
						disass.erase(info.vertex_shader); // Avoid printing shaders twice
					}
					if (disass.count(info.pixel_shader)) {
						LOG(INFO) << disass[info.pixel_shader];
						disass.erase(info.pixel_shader); // Avoid printing shaders twice
					}
				}
			}
		}
		return DEFAULT;
	}
	virtual std::shared_ptr<Shader> injectShader(std::shared_ptr<Shader> shader) {
		//std::string regs = "00000000";
		//for (uint32_t r : shader->outputRegisters())
		//	if (r < 8)
		//		regs[r] = '1';
		//LOG(INFO) << "Shader : " << shader->hash() << "  " << regs;
		disass[shader->hash()] = shader->disassemble();
		/*LOG(INFO) << "Inputs :";
		for(const std::string & s: shader->inputs())
		LOG(INFO) << " - " << s;
		LOG(INFO) << "Outputs :";
		for (const std::string & s : shader->outputs())
		LOG(INFO) << " - " << s;
		LOG(INFO) << shader->disassemble();*/
		return nullptr;
	}
};
REGISTER_CONTROLLER(DrawStats);


BOOL WINAPI DllMain(HINSTANCE hInst, DWORD reason, LPVOID) {
	if (reason == DLL_PROCESS_ATTACH) {
		LOG(INFO) << "DrawStats turned on";
	}

	if (reason == DLL_PROCESS_DETACH) {
		LOG(INFO) << "DrawStats turned off";
	}
	return TRUE;
}
