#include <windows.h>
#include <unordered_map>
#include <unordered_set>
#include "log.h"
#include "sdk.h"
#include <iomanip>
#include <chrono>
#include <fstream>
#include <iterator>
#include "scripthook\main.h"
#include "scripthook\natives.h"
#include "util.h"
#include "gtastate.h"
#include "ps_output.h"
#include "vs_static.h"
#include "ps_flow.h"

uint32_t MAX_UNTRACKED_OBJECT_ID = 1 << 14;

// For debugging, use the current matrices, not the past to estimate the flow
//#define CURRENT_FLOW

double time() {
	auto now = std::chrono::system_clock::now();
	return std::chrono::duration<double>(now.time_since_epoch()).count();
}

struct TrackData: public TrackedFrame::PrivateData {
	uint32_t id=0, last_frame=0, set_bm=0;
	std::vector<uint8_t> prev_data, current_data;
};
struct VehicleTrack {
	float4x4 rage[4];
	float4x4 wheel[16][2];
};
const size_t RAGE_MAT_SIZE = 4 * sizeof(float4x4);
const size_t VEHICLE_SIZE = RAGE_MAT_SIZE;
const size_t WHEEL_SIZE = RAGE_MAT_SIZE + 2 * sizeof(float4x4);
const size_t BONE_MTX_SIZE = 255 * 4 * 3 * sizeof(float);

struct GTA5 : public GameController {
	GTA5() : GameController() {
	}
	virtual bool keyDown(unsigned char key, unsigned char special_status) {
		return false;
	}
	virtual std::vector<ProvidedTarget> providedTargets() const override {
		return { {"albedo"}, {"final"}, { "prev_depth", TargetType::R32_FLOAT, true } };
	}
	virtual std::vector<ProvidedTarget> providedCustomTargets() const {
		// Write the depth into a custom render target (this name needs to match the injection shader buffer name!)
		return { {"flow_depth", TargetType::R32G32B32A32_FLOAT, true}, { "flow", TargetType::R32G32_FLOAT}, { "depth", TargetType::R32_FLOAT },{ "occlusion", TargetType::R32_FLOAT }, { "object_id", TargetType::R32_UINT } };
	}
	CBufferVariable rage_matrices = { "rage_matrices", "gWorld", {0}, {4*16*sizeof(float)} }, wheel_matrices = { "matWheelBuffer", "matWheelWorld",{ 0 },{ 32 * sizeof(float) } }, rage_bonemtx = { "rage_bonemtx", "gBoneMtx",{ 0 },{ BONE_MTX_SIZE } };
	enum ObjectType {
		UNKNOWN=0,
		VEHICLE=1,
		WHEEL=2,
		TREE=3,
		PEDESTRIAN=4,
		BONE_MTX=5,
	};
	std::unordered_map<ShaderHash, ObjectType> object_type;
	std::unordered_set<ShaderHash> final_shader;
	std::shared_ptr<Shader> vs_static_shader = Shader::create(ByteCode(VS_STATIC, VS_STATIC + sizeof(VS_STATIC))),
		ps_output_shader = Shader::create(ByteCode(PS_OUTPUT, PS_OUTPUT + sizeof(PS_OUTPUT)), { { "SV_Target6", "flow_depth" }, { "SV_Target7", "object_id" } }),
		flow_shader = Shader::create(ByteCode(PS_FLOW, PS_FLOW + sizeof(PS_FLOW)), { { "SV_Target0", "flow" },{ "SV_Target1", "depth" },{ "SV_Target2", "occlusion" } });
	std::unordered_set<ShaderHash> int_position = { ShaderHash("d05510b7:0d9c59d0:612cd23a:f75d5ebd"), ShaderHash("c59148c8:e2f1a5ad:649bb9c7:30454a34"), ShaderHash("8a028f64:80694472:4d55d5dd:14c329f2"), ShaderHash("53a6e156:ff48c806:433fc787:c150b034"), ShaderHash("86de7f78:3ecdef51:f1c6f9e3:c5e6338f"), ShaderHash("f37799c8:b304710d:3296b36c:46ea12d4"), ShaderHash("4b031811:b6bf1c7f:ef4cd0c1:56541537") };
	virtual std::shared_ptr<Shader> injectShader(std::shared_ptr<Shader> shader) {
		if (shader->type() == Shader::VERTEX) {
			bool has_rage_matrices = rage_matrices.scan(shader);
			if (has_rage_matrices) {
				bool has_wheel = wheel_matrices.scan(shader);
				bool has_rage_bonemtx = rage_bonemtx.scan(shader);
				ObjectType ot;
				if (has_wheel)
					object_type[shader->hash()] = ot = WHEEL;
				else if (hasCBuffer(shader, "vehicle_globals") && hasCBuffer(shader, "vehicle_damage_locals"))
					object_type[shader->hash()] = ot = VEHICLE;
				else if (hasCBuffer(shader, "trees_common_locals"))
					object_type[shader->hash()] = ot = TREE;
				else if (hasCBuffer(shader, "ped_common_shared_locals"))
					object_type[shader->hash()] = ot = PEDESTRIAN;
				else if (has_rage_bonemtx)
					object_type[shader->hash()] = ot = BONE_MTX;

				bool can_inject = true;
				for (const auto & b : shader->cbuffers())
					if (b.bind_point == 0)
						can_inject = false;
				if (int_position.count(shader->hash()))
					can_inject = false;
				if (can_inject) {
					// Duplicate the shader and copy rage matrices
					auto r = shader->subset({ "SV_Position" });
					r->renameOutput("SV_Position", "PREV_POSITION");
					r->renameCBuffer("rage_matrices", "prev_rage_matrices");
					if (ot == WHEEL)
						r->renameCBuffer("matWheelBuffer", "prev_matWheelBuffer", 5);
					if (ot == PEDESTRIAN || ot == BONE_MTX)
						r->renameCBuffer("rage_bonemtx", "prev_rage_bonemtx", 5);
					// TODO: Handle characters properly
					return r;

					return vs_static_shader;
				}
			}
		}
		if (shader->type() == Shader::PIXEL && hasCBuffer(shader, "misc_globals")) {
			// Inject the shader output
			return ps_output_shader;
		}
		if (shader->type() == Shader::PIXEL && hasTexture(shader, "BackBufferTexture")) {
			final_shader.insert(shader->hash());
		}
		return nullptr;
	}

	virtual void postProcess(uint32_t frame_id) override {
		if (currentRecordingType() & DRAW) {
			callPostFx(flow_shader);
		}
	}

	float4x4 avg_world = 0, avg_world_view = 0, avg_world_view_proj = 0, prev_view = 0, prev_view_proj = 0;
	uint8_t main_render_pass = 0;
	uint32_t oid = 1, base_id = 1;
	std::shared_ptr<CBuffer> id_buffer, prev_buffer, prev_wheel_buffer, prev_rage_bonemtx;
	TrackedFrame * tracker = nullptr;
	std::shared_ptr<TrackData> last_vehicle;
	double start_time;
	uint32_t current_frame_id = 0, wheel_count = 0;

	size_t TS = 0;
	virtual void startFrame(uint32_t frame_id) override {
		start_time = time();

		main_render_pass = 2;
		albedo_output = RenderTargetView();

		if (!id_buffer) id_buffer = createCBuffer("IDBuffer", sizeof(int));
		if (!prev_buffer) prev_buffer = createCBuffer("prev_rage_matrices", 4*sizeof(float4x4));
		if (!prev_wheel_buffer) prev_wheel_buffer = createCBuffer("prev_matWheelBuffer", 4 * sizeof(float4x4));
		if (!prev_rage_bonemtx) prev_rage_bonemtx = createCBuffer("prev_rage_bonemtx", BONE_MTX_SIZE);
		base_id = oid = 1;
		last_vehicle.reset();
		wheel_count = 0;
		tracker = trackNextFrame();

		avg_world = 0;
		avg_world_view = 0;
		avg_world_view_proj = 0;
		TS = 0;
	}
	virtual void endFrame(uint32_t frame_id) override {
		if (currentRecordingType() & DRAW) {
			mul(&prev_view_proj, avg_world.affine_inv(), avg_world_view_proj);
			mul(&prev_view, avg_world.affine_inv(), avg_world_view);
			current_frame_id++;

			// Copy the depth buffer for occlusion testing
			copyTarget("prev_depth", "depth");

			LOG(INFO) << "T = " << time() - start_time << "   S = " << TS;
		}
	}
	RenderTargetView albedo_output;
	virtual DrawType startDraw(const DrawInfo & info) override {
		if ((currentRecordingType() & DRAW) && info.outputs.size() && info.outputs[0].W == defaultWidth() && info.outputs[0].H == defaultHeight() && info.outputs.size() >= 2 && info.type == DrawInfo::INDEX && info.instances == 0) {
			//{
			//	typedef USHORT(WINAPI *CaptureStackBackTraceType)(__in ULONG, __in ULONG, __out PVOID*, __out_opt PULONG);
			//	static CaptureStackBackTraceType func = (CaptureStackBackTraceType)(GetProcAddress(LoadLibrary("kernel32.dll"), "RtlCaptureStackBackTrace"));

			//	if (func != NULL) {
			//		const int kMaxCallers = 62;

			//		void* callers[kMaxCallers];
			//		int count = (func)(0, kMaxCallers, callers, NULL);
			//		if (count > 11)
			//			LOG(INFO) << "Trace " << std::hex << callers[6] << " " << callers[7] << " " << callers[8] << " " << callers[9] << " " << callers[10] << " " << callers[11];
			//	}
			//}



			bool has_rage_matrices = rage_matrices.has(info.vertex_shader);
			ObjectType type = UNKNOWN;
			{
				auto i = object_type.find(info.vertex_shader);
				if (i != object_type.end())
					type = i->second;
			}
			if (has_rage_matrices && main_render_pass > 0) {
				std::shared_ptr<GPUMemory> wp = rage_matrices.fetch(this, info.vertex_shader, info.vs_cbuffers, true);
				if (main_render_pass == 2) {
					// Starting the main render pass
					albedo_output = info.outputs[0];
					main_render_pass = 1;
				}
				// This is just a placehoder for now (I might change the draw type bahavior soon anyway)
				if (main_render_pass == 1) {
					uint32_t id = 0;
					if (wp && wp->size() >= 3 * sizeof(float4x4)) {
						// Fetch the rage matrices gWorld, gWorldView, gWorldViewProj
						const float4x4 * rage_mat = (const float4x4 *)wp->data();
						float4x4 prev_rage[4] = { rage_mat[0], rage_mat[1], rage_mat[2], rage_mat[3] };
						mul(&prev_rage[2], rage_mat[0], prev_view_proj);
						mul(&prev_rage[1], rage_mat[0], prev_view);

						// Sum up the world and world_view_proj matrices to later compute the view_proj matrix
						if (type != PEDESTRIAN) {
							// There is a BUG in GTA V that doesn't draw Franklyn correctly in first person view (rage_mat are wrong)
							add(&avg_world, avg_world, rage_mat[0]);
							add(&avg_world_view, avg_world_view, rage_mat[1]);
							add(&avg_world_view_proj, avg_world_view_proj, rage_mat[2]);
						}

						if (type == WHEEL && last_vehicle) {
							std::shared_ptr<GPUMemory> wm = wheel_matrices.fetch(this, info.vertex_shader, info.vs_cbuffers, true);
							if (wm && wm->size() >= 2 * sizeof(float4x4)) {
								const size_t s = wheel_count * WHEEL_SIZE + VEHICLE_SIZE, e = (wheel_count + 1) * WHEEL_SIZE + VEHICLE_SIZE;
								if (e > last_vehicle->current_data.size())
									last_vehicle->current_data.resize( e );
								memcpy(&last_vehicle->current_data[0] + s, wm->data(), WHEEL_SIZE);

								// Set the previous wheel matrix
								if (e <= last_vehicle->prev_data.size())
									prev_wheel_buffer->set(std::vector<uint8_t>(last_vehicle->prev_data.begin() + s, last_vehicle->prev_data.begin() + e));
								else
									prev_wheel_buffer->set(std::vector<uint8_t>(last_vehicle->current_data.begin() + s, last_vehicle->current_data.begin() + e));
								bindCBuffer(prev_wheel_buffer);
							}
							id = last_vehicle->id;
							wheel_count++;
						} else if (tracker) {
							Vec3f v = { rage_mat[0].d[3][0], rage_mat[0].d[3][1], rage_mat[0].d[3][2] };
							TrackedFrame::Object * object = (*tracker)(v, Quaternion::fromMatrix(rage_mat[0]));
							if (object) {
								std::shared_ptr<TrackData> track = std::dynamic_pointer_cast<TrackData>(object->private_data);
								if (!track) // Create a track if the object is new
									object->private_data = track = std::make_shared<TrackData>();
								// Advance a tracked frame
								if (track->last_frame < current_frame_id) {
									// Avoid objects poping in and out
									if (track->last_frame != current_frame_id - 1)
										track->current_data.clear();

									// Advance time
									track->last_frame = current_frame_id;
									track->prev_data.swap( track->current_data );

									// Update the rage matrix
									track->current_data.resize(RAGE_MAT_SIZE);
									memcpy(&track->current_data[0], rage_mat, RAGE_MAT_SIZE);

									track->set_bm = 0;
								}
								// Update the bone_mtx
								if ((type == PEDESTRIAN || type == BONE_MTX) && track->set_bm == 0) {
									std::shared_ptr<GPUMemory> bm = rage_bonemtx.fetch(this, info.vertex_shader, info.vs_cbuffers, true);
									if (bm) {
										track->current_data.resize(RAGE_MAT_SIZE + BONE_MTX_SIZE, 0);
										memcpy(&track->current_data[RAGE_MAT_SIZE], bm->data(), BONE_MTX_SIZE);
										track->set_bm = 1;
										TS += BONE_MTX_SIZE;
									}
								}

								// Set the prior projection view
								if (track->prev_data.size() >= RAGE_MAT_SIZE)
									memcpy(prev_rage, &track->prev_data[0], RAGE_MAT_SIZE);

								// Set the prior bone mtx
								if (type == PEDESTRIAN || type == BONE_MTX) {
									if (track->prev_data.size() >= RAGE_MAT_SIZE + BONE_MTX_SIZE)
										prev_rage_bonemtx->set(&track->prev_data[RAGE_MAT_SIZE], BONE_MTX_SIZE, 0);
									else if (track->current_data.size() >= RAGE_MAT_SIZE + BONE_MTX_SIZE)
										prev_rage_bonemtx->set(&track->current_data[RAGE_MAT_SIZE], BONE_MTX_SIZE, 0);
									bindCBuffer(prev_rage_bonemtx);
								}

								track->id = id = MAX_UNTRACKED_OBJECT_ID + object->id;
								if (type == VEHICLE) {
									last_vehicle = track;
									wheel_count = 0;
								}
							}
						}
						prev_buffer->set((float4x4*)prev_rage, 4, 0);
						bindCBuffer(prev_buffer);

						id_buffer->set(id);
						bindCBuffer(id_buffer);
						return RIGID;
					}
				}
			}
		} else if (main_render_pass == 1) {
			// End of the main render pass
			copyTarget("albedo", albedo_output);
			main_render_pass = 0;
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
		if (tracker)
			return toJSON(tracker->info);
		return "";
	}
	virtual bool reset () {
		return true;
	}
	virtual bool stop() { return stopTracker(); }
};
REGISTER_CONTROLLER(GTA5);


BOOL WINAPI DllMain(HINSTANCE hInst, DWORD reason, LPVOID) {
	if (reason == DLL_PROCESS_ATTACH) {
		LOG(INFO) << "GTA5 turned on";
		initGTA5State(hInst);
	}

	if (reason == DLL_PROCESS_DETACH) {
		releaseGTA5State(hInst);
		LOG(INFO) << "GTA5 turned off";
	}
	return TRUE;
}
