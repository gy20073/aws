#include <windows.h>
#include <unordered_map>
#include <unordered_set>
#include "log.h"
#include "sdk.h"
#include <iomanip>
#include <chrono>
#include <fstream>
#include <iterator>
#include "util.h"
#include "ps_output.h"
#include "ps_flow.h"

uint32_t MAX_UNTRACKED_OBJECT_ID = 1 << 14;

// For debugging, use the current matrices, not the past to estimate the flow
//#define CURRENT_FLOW

double time() {
	auto now = std::chrono::system_clock::now();
	return std::chrono::duration<double>(now.time_since_epoch()).count();
}

struct FARCRY : public GameController {
	FARCRY() : GameController() {
	}
	virtual bool keyDown(unsigned char key, unsigned char special_status) {
		return false;
	}
	virtual std::vector<ProvidedTarget> providedTargets() const override {
		return { {"albedo"}, {"final"} };
	}
	virtual std::vector<ProvidedTarget> providedCustomTargets() const {
		// Write the depth into a custom render target (this name needs to match the injection shader buffer name!)
		return { {"flow_depth", TargetType::R32G32B32A32_FLOAT, true}, { "flow", TargetType::R32G32_FLOAT}, { "depth", TargetType::R32_FLOAT }, { "object_id", TargetType::R32_UINT } };
	}
	CBufferVariable rage_matrices = { "rage_matrices", "gWorld", {0}, {4*16*sizeof(float)} };
	std::unordered_set<ShaderHash> final_shader;
	std::shared_ptr<Shader> ps_output_shader = Shader::create(ByteCode(PS_OUTPUT, PS_OUTPUT + sizeof(PS_OUTPUT)), { { "SV_Target6", "flow_depth" }, { "SV_Target7", "object_id" } }),
		flow_shader = Shader::create(ByteCode(PS_FLOW, PS_FLOW + sizeof(PS_FLOW)), { { "SV_Target0", "flow" },{ "SV_Target1", "depth" } });
	virtual std::shared_ptr<Shader> injectShader(std::shared_ptr<Shader> shader) {
		//if (shader->type() == Shader::PIXEL && hasCBuffer(shader, "misc_globals")) {
		//	// Inject the shader output
		//	return ps_output_shader;
		//}
		if (shader->type() == Shader::PIXEL && hasTexture(shader, "SceneColorTexture") && hasTexture(shader, "SceneColorAccumulationTexture")) {
			LOG(INFO) << shader->disassemble();
			final_shader.insert(shader->hash());
		}
		return nullptr;
	}

	virtual void postProcess(uint32_t frame_id) override {
		if (currentRecordingType() & DRAW) {
			callPostFx(flow_shader);
		}
	}

	double start_time;
	virtual void startFrame(uint32_t frame_id) override {
		start_time = time();
	}
	virtual void endFrame(uint32_t frame_id) override {
		if (currentRecordingType() & DRAW) {
			LOG(INFO) << "T = " << time() - start_time;
		}
	}
	RenderTargetView albedo_output;
	virtual DrawType startDraw(const DrawInfo & info) override {
		if ((currentRecordingType() & DRAW) && info.outputs.size() && info.outputs[0].W == defaultWidth() && info.outputs[0].H == defaultHeight() && info.outputs.size() >= 2 && info.type == DrawInfo::INDEX) {

		}
		return DEFAULT;
	}
	virtual void endDraw(const DrawInfo & info) override {
		if (final_shader.count(info.pixel_shader)) {
			// Draw the final image (right before the image is distorted)
			copyTarget("final", info.outputs[0]);
		}
	}
	virtual std::string gameState() const override {
		return "";
	}
	virtual bool reset () {
		return true;
	}
};
REGISTER_CONTROLLER(FARCRY);


BOOL WINAPI DllMain(HINSTANCE hInst, DWORD reason, LPVOID) {
	if (reason == DLL_PROCESS_ATTACH) {
		LOG(INFO) << "FARCRY turned on";
	}

	if (reason == DLL_PROCESS_DETACH) {
		LOG(INFO) << "FARCRY turned off";
	}
	return TRUE;
}
