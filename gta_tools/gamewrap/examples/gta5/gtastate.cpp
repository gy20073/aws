#include "gtastate.h"
#include "log.h"
#include "scripthook/main.h"
#include "scripthook/types.h"
#include "scripthook/natives.h"
#include <string>
#include <ostream>
#include <mutex>
#include <Windows.h>

const float TRACKING_RADIUS = 0.1f;
const float TARCKING_QUAT = 0.1f;

struct Tracker {
	static std::mutex is_tracking;
	static TrackedFrame current, returned;
	static uint64_t current_id, returned_id;
	static bool stop_tracking, currently_tracking;

	static void Main() {
		current_id = returned_id = 1;
		stop_tracking = false;
		currently_tracking = true;
		while (!stop_tracking) {
			{
				std::lock_guard<std::mutex> lock(is_tracking);
				current.fetch();
				current_id += 1;
			}
			WAIT(0);
		}
		currently_tracking = false;
		TERMINATE();
	}
	static TrackedFrame * nextFrame() {
		if (!current_id) return nullptr;
		std::lock_guard<std::mutex> lock(is_tracking);
		uint64_t delta = current_id - returned_id;
		if (delta > 0) {
			for (int i = 0; i < N_OBJECTS; i++) {
				if (returned.objects[i].id == current.objects[i].id) {
					returned.objects[i].age = current.objects[i].age = returned.objects[i].age + (uint32_t)delta;
					returned.objects[i].p = current.objects[i].p;
					returned.objects[i].q = current.objects[i].q;
					// Let's associate the private data with the returned object only [no swapping here]
				} else if (current.objects[i].id) {
					returned.objects[i] = current.objects[i];
				} else if (returned.objects[i].id) {
					returned.objects[i].id = 0;
					returned.objects[i].private_data.reset();
				}
			}
			returned.object_map.swap(current.object_map);
			returned.info = current.info;
			returned_id = current_id;
		}
		return &returned;
	}
	static bool stop() {
		stop_tracking = true;
		return !currently_tracking;
	}
};
bool stopTracker() {
	return Tracker::stop();
}
std::mutex Tracker::is_tracking;
TrackedFrame Tracker::current;
TrackedFrame Tracker::returned;
uint64_t Tracker::current_id = 0, Tracker::returned_id = 0;
bool Tracker::stop_tracking = false;
bool Tracker::currently_tracking = false;

TrackedFrame * trackNextFrame() {
	return Tracker::nextFrame();
}

void initGTA5State(HMODULE hInstance) {
	scriptRegister(hInstance, Tracker::Main);
}
void releaseGTA5State(HMODULE hInstance) {
	scriptUnregister(hInstance);
}

TrackedFrame::TrackedFrame() :object_map(TRACKING_RADIUS) {}

uint32_t ID(uint32_t id, TrackedFrame::ObjectType t) {
	//return ((uint32_t)t) << 24 | ((id << 8)>>8);
	return ((uint32_t)id);
}
void TrackedFrame::fetch() {
	static std::mutex fetching;
	std::lock_guard<std::mutex> lock(fetching);

	static int entity_buf[1 << 14];

	// Clear the tracker
	object_map.clear();
	for (int i = 0; i < N_OBJECTS; i++)
		objects[i].id = 0;

	// Track all new objects
	typedef int(*WorldGet)(int*, int);
	WorldGet worldGet[] = { &worldGetAllVehicles, &worldGetAllPeds , &worldGetAllObjects , &worldGetAllPickups };
	//ObjectType type[] = { PED, OBJECT, PICKUP, VEHICLE };
	//for (int it = 0; it * sizeof(type[0]) < sizeof(type); it++) {
	//	int n = worldGet[it](entity_buf, sizeof(entity_buf) / sizeof(entity_buf[0]));
	//	ObjectType t = type[it];
	//	for (int i = 0; i < n; i++) {
	//		const int e = entity_buf[i];
	//		Quaternion q;
	//		ENTITY::GET_ENTITY_QUATERNION(e, &q.x, &q.y, &q.z, &q.w);
	//		Vector3 p = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(e, 0.0, 0.0, 0.0);

	//		// Add the entry
	//		uint32_t k = (e >> 8) & (N_OBJECTS - 1);
	//		if (objects[k].id)
	//			LOG(WARN) << "Tracker has duplicate objects";
	//		objects[k] = { ID(e, t), 0, {p.x, p.y, p.z}, q, nullptr };
	//		object_map.insert({ objects[k].p.x, objects[k].p.y }, k);
	//	}
	//}

	int num_vehicles = worldGet[0](entity_buf, sizeof(entity_buf) / sizeof(entity_buf[0]));
	for (int i = 0; i < num_vehicles; i++) {
		const int e = entity_buf[i];
		Quaternion q;
		ENTITY::GET_ENTITY_QUATERNION(e, &q.x, &q.y, &q.z, &q.w);
		Vector3 p = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(e, 0.0, 0.0, 0.0);
		ObjectType t = UNKNOWN;
		Hash model = ENTITY::GET_ENTITY_MODEL(e);
		if (VEHICLE::IS_THIS_MODEL_A_CAR(model)) t = CAR;
		else if (VEHICLE::IS_THIS_MODEL_A_BIKE(model)) t = MOTORBIKE;
		else if (VEHICLE::IS_THIS_MODEL_A_BICYCLE(model)) t = BICYCLE;
		else if (VEHICLE::IS_THIS_MODEL_A_QUADBIKE(model)) t = QUADBIKE;
		else if (VEHICLE::IS_THIS_MODEL_A_BOAT(model)) t = BOAT;
		else if (VEHICLE::IS_THIS_MODEL_A_PLANE(model)) t = PLANE;
		else if (VEHICLE::IS_THIS_MODEL_A_HELI(model)) t = HELICOPTER;
		else if (VEHICLE::IS_THIS_MODEL_A_TRAIN(model)) t = TRAIN;
		else if (VEHICLE::_IS_THIS_MODEL_A_SUBMERSIBLE(model)) t = SUBMERSIBLE;

		// Add the entry
		uint32_t k = (e >> 8) & (N_OBJECTS - 1);
		if (objects[k].id)
			LOG(WARN) << "Tracker has duplicate objects";
		objects[k] = { ID(e, t), 0,{ p.x, p.y, p.z }, q, nullptr };
		object_map.insert({ objects[k].p.x, objects[k].p.y }, k);
	}

	int num_ped = worldGet[1](entity_buf, sizeof(entity_buf) / sizeof(entity_buf[0]));
	for (int i = 0; i < num_ped; i++) {
		const int e = entity_buf[i];
		Quaternion q;
		ENTITY::GET_ENTITY_QUATERNION(e, &q.x, &q.y, &q.z, &q.w);
		Vector3 p = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(e, 0.0, 0.0, 0.0);
		ObjectType t = PEDESTRIAN;
		Hash model = ENTITY::GET_ENTITY_MODEL(e);
		if (PED::IS_PED_IN_ANY_VEHICLE(e, TRUE)) t = RIDER;
		if (PED::GET_PED_TYPE(e) == 28) t = ANIMAL;

		// Add the entry
		uint32_t k = (e >> 8) & (N_OBJECTS - 1);
		if (objects[k].id)
			LOG(WARN) << "Tracker has duplicate objects";
		objects[k] = { ID(e, t), 0,{ p.x, p.y, p.z }, q, nullptr };
		object_map.insert({ objects[k].p.x, objects[k].p.y }, k);
	}

	Player p = PLAYER::PLAYER_ID();
	Ped pp = PLAYER::PLAYER_PED_ID();
	Vehicle vec = PLAYER::GET_PLAYERS_LAST_VEHICLE();
	info.time_since_player_drove_against_traffic = PLAYER::GET_TIME_SINCE_PLAYER_DROVE_AGAINST_TRAFFIC(p);
	info.time_since_player_drove_on_pavement = PLAYER::GET_TIME_SINCE_PLAYER_DROVE_ON_PAVEMENT(p);
	info.time_since_player_hit_ped = PLAYER::GET_TIME_SINCE_PLAYER_HIT_PED(p);
	info.time_since_player_hit_vehicle = PLAYER::GET_TIME_SINCE_PLAYER_HIT_VEHICLE(p);
	Vector3 x = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(pp, 0.0, 0.0, 0.0);
	info.position = { x.x, x.y, x.z };
	x = ENTITY::GET_ENTITY_FORWARD_VECTOR(vec);
	info.forward_vector = { x.x, x.y, x.z };
	info.heading = ENTITY::GET_ENTITY_HEADING(vec);
	Cam cam = CAM::GET_RENDERING_CAM();
	Vector3 c_pos = CAM::GET_CAM_COORD(cam);
	info.cam_pos = { c_pos.x,c_pos.y,c_pos.z };
	Vector3 c_rot = CAM::GET_CAM_ROT(cam, 0);
	info.cam_rot = { c_rot.x,c_rot.y,c_rot.z };
	info.cam_near_clip = CAM::GET_CAM_NEAR_CLIP(cam);
	info.cam_field_of_view = CAM::GET_CAM_FOV(cam);
}

TrackedFrame::Object * TrackedFrame::operator[](uint32_t id) {
	uint32_t i = (id >> 8) & (N_OBJECTS-1);
	if (objects[i].id == id)
		return objects + i;
	return nullptr;
}

const TrackedFrame::Object * TrackedFrame::operator[](uint32_t id) const {
	uint32_t i = (id >> 8) & (N_OBJECTS - 1);
	if (objects[i].id == id)
		return objects + i;
	return nullptr;
}

TrackedFrame::Object * TrackedFrame::operator()(const Vec3f & p, const Quaternion & q) {
	Object * r = nullptr;
	float d = TRACKING_RADIUS*TRACKING_RADIUS;
	object_map.find({ p.x, p.y }, [&](size_t i) {
		float dd = D2(objects[i].p, p);
		if (dd < d && D2(objects[i].q, q) < TARCKING_QUAT) {
			d = dd;
			r = objects + i;
		}
	});
	return r;
}

const TrackedFrame::Object * TrackedFrame::operator()(const Vec3f & p, const Quaternion & q) const {
	const Object * r = nullptr;
	float d = TRACKING_RADIUS*TRACKING_RADIUS;
	object_map.find({ p.x, p.y }, [&](size_t i) {
		float dd = D2(objects[i].p, p);
		if (dd < d && D2(objects[i].q, q) < TARCKING_QUAT) {
			d = dd;
			r = objects+i;
		}
	});
	return r;
}

TrackedFrame::ObjectType TrackedFrame::Object::type() const {
	return (ObjectType)((id >> 28) & 0xf);
}

uint32_t TrackedFrame::Object::handle() const {
	return id & ((1<<28)-1);
}

TrackedFrame::PrivateData::~PrivateData() {}
