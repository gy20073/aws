#include <windows.h>
#include <unordered_map>
#include "log.h"
#include "sdk.h"
#include <iomanip>
#include <chrono>
#include <fstream>
#include <iterator>

// Shaders
#include "id_inject.h"
#include "passtru.h"

GAME_STATE(SupersonicSled,
(uint8_t, alive, 255), // Are we alive?
(uint8_t, started, 0), // Has the game started
(uint8_t, finished, 0), // Has the game finished (did we win?)
(uint8_t, complete, 0), // Is the state complete (or did we miss something)
(uint8_t, ready, 0),    // Are we ready to play?
(uint8_t, booster1, 0), // Booster state 1 : 0 ready, 1 active, 2 depleted
(uint8_t, booster2, 0),
(uint8_t, booster3, 0),
(uint8_t, booster4, 0),
(uint8_t, boosterR, 0), // Revierse booster state
(float, position, -0.01f),   // Position along track [0..1]
(float, speed, -1.f),     // Speed 0 .. 800-900
(float, thrust, -1.f),    // Thrust 0 .. 1
(float, brake, -1.f),     // Brake  0 .. 1
(float, time, -1.f));     // Current time
//	uint8_t booster_state)

struct Supersonic : public GameController {
	Supersonic() : GameController() {
	}

	uint8_t dump = 0;
	bool hide = false;
	virtual bool keyDown(unsigned char key, unsigned char special_status) {
		if (key == VK_F2) {
			hide = !hide;
		}
		if (key == VK_F6) {
			LOG(INFO) << "Reset";
			requestGameReset();
		}
		if (key == VK_F7) {
			LOG(INFO) << "dump";
			dump = 2;
		}
		return false;
	}
	virtual std::vector<ProvidedTarget> providedTargets() const override {
		return { { "final" } };
	}
	SupersonicSled last_state;
	SupersonicSled state;
	std::shared_ptr<GPUMemory> speed_buf, thrust_buf, brake_buf, txt_buf, button_buf[8], view_buf, proj_buf, world_pos_buf;
	int ui_count = 0, button_count = 0, transform_read = 0;
	virtual void startFrame(uint32_t frame_id) {
		if (dump > 0) dump--;
		state = SupersonicSled();
		transform_read = button_count = ui_count = 0;
	}
	virtual void endFrame(uint32_t frame_id) {
		if (speed_buf && *speed_buf)  state.speed  = *speed_buf->data<float>() / 1000;
		if (thrust_buf && *thrust_buf) state.thrust = (*thrust_buf->data<float>() - 0.349491701977f) / 0.293978750705719f;  // Normalize to 0..1
		if (brake_buf && *brake_buf)  state.brake  = (*brake_buf->data<float>() - 0.533821742677f) / 0.279384911060333f;   // Normalize to 0..1
		if (txt_buf && *txt_buf) {
			char txt[6] = { 0 };
			// Parse the time
			const float * D = txt_buf->data<float>();
			for (size_t i = 0; 2 * i * sizeof(float) < txt_buf->size() && i < 5; i++) {
				if (fabs(D[2 * i + 0] - 0.8125) < 1e-2 && fabs(D[2 * i + 1] - 0.75) < 1e-2)
					txt[i] = '.';
				else if (fabs(D[2 * i + 0] - 0.9375) < 1e-2 && fabs(D[2 * i + 1] - 0.75) < 1e-2)
					txt[i] = '0';
				else if (fabs(D[2 * i + 1] - 0.625) < 1e-2)
					txt[i] = '1' + (int)(16 * (D[2 * i + 0] + 2e-2));
				else
					txt[0] = 0;
			}
			if (txt[0])
				state.time = (float)atof(txt);
		}
		if (button_buf[0] && *button_buf[0]) state.alive = *button_buf[0]->data<float>() < 0.5;
		if (button_buf[1] && *button_buf[1]) state.ready = *button_buf[1]->data<float>() > 0.5;
		if (button_buf[2] && *button_buf[2]) state.booster1 = (button_buf[2]->data<float>()[0] > 0.5) + 2 * (button_buf[2]->data<float>()[1] > 0.5);
		if (button_buf[3] && *button_buf[3]) state.booster2 = (button_buf[3]->data<float>()[0] > 0.5) + 2 * (button_buf[3]->data<float>()[1] > 0.5);
		if (button_buf[4] && *button_buf[4]) state.booster3 = (button_buf[4]->data<float>()[0] > 0.5) + 2 * (button_buf[4]->data<float>()[1] > 0.5);
		if (button_buf[5] && *button_buf[5]) state.booster4 = (button_buf[5]->data<float>()[0] > 0.5) + 2 * (button_buf[5]->data<float>()[1] > 0.5);
		if (button_buf[6] && *button_buf[6]) state.boosterR = (button_buf[6]->data<float>()[0] > 0.5) + 2 * (button_buf[6]->data<float>()[1] > 0.5);

		if (ui_count > 20) // Disqualified
			state.alive = 0;

		if (world_pos_buf && *world_pos_buf) {
			state.position = (world_pos_buf->data<float>()[2] - 6.96) / 11270; // range 6.96 .. 11274 (fall at 11870, 1.052 relative)
			if (state.position < 0) state.position = 0;
		}

		state.started = state.time > 0;
		state.finished = state.started && (state.time == last_state.time) && state.speed < 0.01;
		state.complete = (state.speed >= 0) && (state.thrust >= 0) && (state.brake >= 0) && (state.time >= 0) && (state.alive < 255);
		last_state = state;
		if (!state.ready && frame_id > 10 /* for some reason sending mouse clicks too early will crash the game */)
			resetGame();

		// Reset all buffers
		speed_buf.reset();
		thrust_buf.reset();
		brake_buf.reset();
		txt_buf.reset();
		for (size_t i = 0; i * sizeof(button_buf[0]) < sizeof(button_buf); i++)
			button_buf[i].reset();
		view_buf.reset();
		proj_buf.reset();
		world_pos_buf.reset();
	}
	virtual const GameState * gameState() const {
		return &last_state;
	}
	virtual bool resetGame() {
		sendKeyDown(VK_RETURN);
		sendKeyUp(VK_RETURN);
		sendKeyDown(VK_RETURN);
		sendKeyUp(VK_RETURN);
		sendMouseDown(-0.33f, -0.97f, 1); // reset button
		sendMouseUp(-0.33f, -0.97f, 1);
		sendMouseDown(-0.78f, -0.97f, 1); // chase camera button
		sendMouseUp(-0.78f, -0.97f, 1);
		return true;
	}
	std::unordered_map<ShaderHash, std::string> disass;
	const ShaderHash final_shader_hash = "9668ba99:9c46be88:663bbe85:8ca46521";
	const ShaderHash text_shader_hash = "e3248088:e0334180:4d28d13a:55a1126e";
	const ShaderHash ui_shader_hash = "4b47f8fb:f551c41e:0c81913e:6658dc0d";
	const ShaderHash button_shader_hash = "aeff097b:e6ccc388:b6e466f8:754ea176";
	const ShaderHash transform_pool_shader = "9fb72916:1fce6919:81788d0a:b831b700";
	const ShaderHash speed_shader_hash = "a68f24e1:f9d34511:a7c041de:7d601f8e";

	struct Location {
		uint32_t bind_point, offset;
	};
	typedef std::unordered_map<ShaderHash, Location> ShaderLocation;
	ShaderLocation world_pos, view, proj, viewProj, transform_model, skin_model, shape_model;

	virtual Location findVariable(std::shared_ptr<Shader> shader, const std::string & cb_name, const std::string & vname) {
		for (const auto & cb : shader->cbuffers())
			if (!cb_name.size() || cb.name == cb_name)
				for (const auto & v : cb.variables)
					if (!vname.size() || v.name == vname)
						return { cb.bind_point, v.offset };
		return { 0xffff, 0xffff };
	}
	virtual void findAndSetVariable(ShaderLocation & map, std::shared_ptr<Shader> shader, const std::string & cb_name, const std::string & vname) {
		auto v = findVariable(shader, cb_name, vname);
		if (v.bind_point != 0xffff)
			map[shader->hash()] = v;
	}

	virtual std::shared_ptr<Shader> injectShader(std::shared_ptr<Shader> shader) {
		if (shader->type() == Shader::VERTEX && shader->cbuffers().size() > 0) {
			// Find the camera world pos
			findAndSetVariable(world_pos, shader, "", "camWorldPos");
			findAndSetVariable(view, shader, "s_cameraParams", "view");
			findAndSetVariable(proj, shader, "s_cameraParams", "proj");
			findAndSetVariable(viewProj, shader, "s_cameraParams", "viewProj");
			findAndSetVariable(transform_model, shader, "l_TransformPool", "model");
			findAndSetVariable(skin_model, shader, "l_SkinCluster", "model");
			findAndSetVariable(shape_model, shader, "l_shapeParams", "model");

			//if (findVariable(shader, "l_shapeParams", "model").bind_point != 0xffff)
			//	return Shader::create(ByteCode(id_inject, id_inject+sizeof(id_inject)));
		}
		if (shader->type() == Shader::PIXEL) {
			//"ed63d061:64885aef:7bbf394f:5196f333";
			//if (findVariable(shader, "s_shaderParams", "").bind_point != 0xffff)
			//if (shader->outputs().size()>0)
				//return Shader::create(ByteCode(passtru, passtru + sizeof(passtru)));
		}
		return nullptr;
	}
	virtual DrawType startDraw(const DrawInfo & info) override {
		if (info.outputs.size() && info.outputs[0].W == defaultWidth() && info.outputs[0].H == defaultHeight()) {
			if (shape_model.count(info.vertex_shader) || transform_model.count(info.vertex_shader)) return RIGID;
		}
		return DEFAULT;
	}
	std::shared_ptr<GPUMemory> read(const ShaderLocation & l, size_t n, const DrawInfo & info, bool immediate=false) {
		auto i = l.find(info.vertex_shader);
		if (i != l.end()) {
			auto j = i->second;
			if (j.bind_point < info.vs_cbuffers.size()) {
				return readBuffer(info.vs_cbuffers[j.bind_point], j.offset, n, immediate);
			}
		}
		return std::shared_ptr<GPUMemory>();
	}
	virtual void endDraw(const DrawInfo & info) override {
		// Should be copy the final image?
		if ((currentRecordingType() & DRAW) && info.pixel_shader == final_shader_hash) {
			copyTarget("final", info.outputs[0]);
		}


		/* Reading the UI info into the game state */
		// Speed
		if (info.vertex_shader == speed_shader_hash)
			// CB[0] the 17th float field capture the speed
			speed_buf = readBuffer(info.vs_cbuffers[0], 16 * sizeof(float), sizeof(float));

		// Thrust and brake
		if (info.vertex_shader == ui_shader_hash) {
			if (ui_count == 2)
				// CB[0] the 8th float field captures the x position of the thrust indicator
				thrust_buf = readBuffer(info.vs_cbuffers[0], 7 * sizeof(float), sizeof(float));
			if (ui_count == 4)
				// CB[0] the 12th float field captures the x position of the thrust indicator
				brake_buf = readBuffer(info.vs_cbuffers[0], 11 * sizeof(float), sizeof(float));
			ui_count++;
		}
		// Time
		if (info.vertex_shader == text_shader_hash && (info.n == 24 || info.n == 30)) {
			std::vector<size_t> offset, size;
			for (size_t i = 0; 6*i < info.n && i < 5; i++) {
				offset.push_back((i * 6 * 4 + 2) * sizeof(float));
				size.push_back(2*sizeof(float));
			}
			txt_buf = readBuffer(info.vertex_buffer, offset, size);
		}
		// Other game state
		if (info.pixel_shader == button_shader_hash && info.ps_cbuffers.size()) {
			uint32_t bid = -1;
			if (3 <= button_count && button_count < 8) {
				bid = 2 + (button_count - 3);
			}
			else if (button_count == 10) {
				bid = 0;
			}
			else if (button_count == 15)
				bid = 1;
			if (bid < 8)
				button_buf[bid] = readBuffer(info.ps_cbuffers[0], 4 * sizeof(float), 2 * sizeof(float));
			button_count++;
		}
		if (world_pos.count(info.vertex_shader))
			world_pos_buf = read(world_pos, 3 * sizeof(float), info);

		//if (view.count(info.vertex_shader)) {
		//	view_buf = read(view, 16 * sizeof(float), info);
		//	setCurrentView(view_buf);
		//}

		//if (proj.count(info.vertex_shader)) {
		//	proj_buf = read(proj, 16 * sizeof(float), info);
		//	setCurrentProjection(proj_buf);
		//}

		//if (info.outputs.size() && info.outputs[0].W == defaultWidth() && info.outputs[0].H == defaultHeight()) {
		//	// Main render pass
		//	if (shape_model.count(info.vertex_shader)) {
		//		setCurrentModel(read(shape_model, 16 * sizeof(float), info));
		//	}
		//}
	}
};
REGISTER_CONTROLLER(Supersonic);


BOOL WINAPI DllMain(HINSTANCE hInst, DWORD reason, LPVOID) {
	if (reason == DLL_PROCESS_ATTACH) {
		LOG(INFO) << "DrawStats turned on";
	}

	if (reason == DLL_PROCESS_DETACH) {
		LOG(INFO) << "DrawStats turned off";
	}
	return TRUE;
}
