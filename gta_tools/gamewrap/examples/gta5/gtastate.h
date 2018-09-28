#pragma once
#include <memory>
#include <unordered_map>
#include "util.h"

#ifndef _WINDEF_
struct HINSTANCE__; // Forward or never
typedef HINSTANCE__* HINSTANCE;
typedef HINSTANCE HMODULE;
#endif
void initGTA5State(HMODULE hInstance);
void releaseGTA5State(HMODULE hInstance);


struct GameInfo {
	int time_since_player_hit_vehicle;
	int time_since_player_hit_ped;
	int time_since_player_drove_on_pavement;
	int time_since_player_drove_against_traffic;
	Vec3f position, forward_vector;
	float heading;
	Vec3f cam_pos, cam_rot;
	float cam_near_clip, cam_field_of_view;
};
TOJSON(GameInfo, time_since_player_hit_vehicle, time_since_player_hit_ped, 
	time_since_player_drove_on_pavement, time_since_player_drove_against_traffic,
	position, forward_vector, heading,cam_pos,cam_rot, cam_near_clip, cam_field_of_view)

// N_OBJECTS Maximum number of frames, needs to be a power of 2
#define N_OBJECTS (1<<13)
struct TrackedFrame {
	enum ObjectType {
		NONE         = 0,
		CAR          = 1,
		MOTORBIKE    = 2,
		BICYCLE		 = 3,
		QUADBIKE	 = 4,
		BOAT		 = 5,
		PLANE		 = 6,
		HELICOPTER	 = 7,
		TRAIN		 = 8,
		SUBMERSIBLE	 = 9,
		PEDESTRIAN	 = 10,
		RIDER		 = 11,
		BUILDING	 = 12,
		FENCE		 = 13,
		POLE		 = 14,
		ROADLINE	 = 15,
		ROAD		 = 16,
		SIDEWALK	 = 17,
		VEGETATION	 = 18,
		WALL		 = 19,
		TRAFFICSIGN  = 20,
		LANE		 = 21,
		ANIMAL       = 22,
		UNKNOWN      = 255,

	};
	struct PrivateData {
		virtual ~PrivateData();
	};
	struct Object {
		uint32_t id = 0;
		uint32_t age = 0;
		Vec3f p;
		Quaternion q;
		std::shared_ptr<PrivateData> private_data;
		ObjectType type() const;
		uint32_t handle() const;
	};
protected:
	friend struct Tracker;
	Object objects[N_OBJECTS];
	NNSearch2D<size_t> object_map;
	void fetch();

public:
	TrackedFrame();
	GameInfo info;
	Object * operator[](uint32_t id);
	const Object * operator[](uint32_t id) const;
	Object * operator()(const Vec3f & v, const Quaternion & q);
	const Object * operator()(const Vec3f & v, const Quaternion & q) const;
};

TrackedFrame * trackNextFrame();
bool stopTracker();
