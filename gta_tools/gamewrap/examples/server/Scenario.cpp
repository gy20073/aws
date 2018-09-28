#include "Scenario.h"
#include "lib/utils.h"
#include "lib/rapidjson/writer.h"
#include "defaults.h"
#include <time.h>
#include "lib/natives.h"
#include "lib/types.h"
#include "lib/enums.h"
#include "lib/main.h"


char* Scenario::weatherList[14] = { "CLEAR", "EXTRASUNNY", "CLOUDS", "OVERCAST", "RAIN", "CLEARING", "THUNDER", "SMOG", "FOGGY", "XMAS", "SNOWLIGHT", "BLIZZARD", "NEUTRAL", "SNOW" };
char* Scenario::vehicleList[4] = { "blista", "voltic", "packer", "oracle"};

void Scenario::parseScenarioConfig(const Value& sc, bool setDefaults) {
	const Value& location = sc["location"];
	const Value& time = sc["time"];
	const Value& weather = sc["weather"];
	const Value& vehicle = sc["vehicle"];
	const Value& drivingMode = sc["drivingMode"];

	if (location.IsArray()) {
		if (!location[0].IsNull()) x = location[0].GetFloat();
		else if (setDefaults) x = 5000 * ((float)rand() / RAND_MAX) - 2500;

		if (!location[1].IsNull()) y = location[1].GetFloat(); 
		else if (setDefaults) y = 8000 * ((float)rand() / RAND_MAX) - 2000;
	}
	else if (setDefaults) {
		x = 5000 * ((float)rand() / RAND_MAX) - 2500;
		y = 8000 * ((float)rand() / RAND_MAX) - 2000;
	}
	printf("Location: %f %f", x, y);

	if (time.IsArray()) {
		if (!time[0].IsNull()) hour = time[0].GetInt();
		else if (setDefaults) hour = rand() % 24;

		if (!time[1].IsNull()) minute = time[1].GetInt();
		else if (setDefaults) minute = rand() % 60;
	}
	else if (setDefaults) {
		hour = rand() % 24;
		minute = rand() % 60;
	}

	if (!weather.IsNull()) _weather = weather.GetString();
	else if (setDefaults) _weather = weatherList[rand() % 14];

	if (!vehicle.IsNull()) _vehicle = vehicle.GetString();
	else if (setDefaults) _vehicle = vehicleList[rand() % 3];

	if (drivingMode.IsArray()) {
		if (!drivingMode[0].IsNull()) _drivingMode = drivingMode[0].GetInt();
		else if (setDefaults)  _drivingMode = rand() % 4294967296;
		if (drivingMode[1].IsNull()) _setSpeed = drivingMode[1].GetFloat(); 
		else if (setDefaults) _setSpeed = 15.0; // 1.0*(rand() % 20);
	}
	else if (setDefaults) {
		_drivingMode = -1;
	}
}

void Scenario::parseDatasetConfig(const Value& dc, bool setDefaults) {
	if (!dc["rate"].IsNull()) rate = dc["rate"].GetInt();
	else if (setDefaults) rate = _RATE_;
	
	if (!dc["frame"].IsNull()) {
		if (!dc["frame"][0].IsNull()) width = dc["frame"][0].GetInt();
		else if (setDefaults) width = _WIDTH_;

		if (!dc["frame"][1].IsNull()) height = dc["frame"][1].GetInt();
		else if (setDefaults) height = _HEIGHT_;
	}
	else if (setDefaults) {
		width = _WIDTH_;
		height = _HEIGHT_;
	}	

	if (!dc["vehicles"].IsNull()) vehicles = dc["vehicles"].GetBool();
	else if (setDefaults) vehicles = _VEHICLES_;

	if (!dc["peds"].IsNull()) peds = dc["peds"].GetBool();
	else if (setDefaults) peds = _PEDS_;

	if (!dc["trafficSigns"].IsNull()) trafficSigns = dc["trafficSigns"].GetBool();
	else if (setDefaults) trafficSigns = _TRAFFIC_SIGNS_;

	if (!dc["direction"].IsNull()) {
		if (!dc["direction"][0].IsNull()) dir.x = dc["direction"][0].GetFloat();
		else if (setDefaults) direction = _DIRECTION_;

		if (!dc["direction"][1].IsNull()) dir.y = dc["direction"][1].GetFloat();
		else if (setDefaults) direction = _DIRECTION_;

		if (!dc["direction"][2].IsNull()) dir.z = dc["direction"][2].GetFloat();
		else if (setDefaults) direction = _DIRECTION_;
	}
	else if (setDefaults) direction = _DIRECTION_;

	if (!dc["throttle"].IsNull()) throttle = dc["throttle"].GetBool();
	else if (setDefaults) throttle = _THROTTLE_;
	if (!dc["brake"].IsNull()) brake = dc["brake"].GetBool();
	else if (setDefaults) brake = _BRAKE_;
	if (!dc["steering"].IsNull()) steering = dc["steering"].GetBool();
	else if (setDefaults) steering = _STEERING_;
	if (!dc["speed"].IsNull()) speed = dc["speed"].GetBool();
	else if (setDefaults) speed = _SPEED_;
	if (!dc["yawRate"].IsNull()) yawRate = dc["yawRate"].GetBool();
	else if (setDefaults) yawRate = _YAW_RATE_;
	if (!dc["drivingMode"].IsNull()) drivingMode = dc["drivingMode"].GetBool();
	else if (setDefaults) drivingMode = _DRIVING_MODE_;
	if (!dc["location"].IsNull()) location = dc["location"].GetBool();
	else if (setDefaults) location = _LOCATION_;
	if (!dc["time"].IsNull()) time = dc["time"].GetBool();
	else if (setDefaults) time = _TIME_;
	collision = _COLLISION_;


	//Create JSON DOM
	d.SetObject();
	Document::AllocatorType& allocator = d.GetAllocator();
	Value a(kArrayType);

	if (vehicles) d.AddMember("vehicles", a, allocator);
	if (peds) d.AddMember("peds", a, allocator);
	if (trafficSigns) d.AddMember("trafficSigns", a, allocator);
	if (direction) d.AddMember("direction", a, allocator);
	if (throttle) d.AddMember("throttle", 0.0, allocator);
	if (brake) d.AddMember("brake", 0.0, allocator);
	if (steering) d.AddMember("steering", 0.0, allocator);
	if (speed) d.AddMember("speed", 0.0, allocator);
	if (yawRate) d.AddMember("yawRate", 0.0, allocator);
	if (drivingMode) d.AddMember("drivingMode", 0, allocator);
	if (location) d.AddMember("location", a, allocator);
	if (time) d.AddMember("time", 0, allocator);
	if (collision) d.AddMember("collision", 0, allocator);
	// camera intrinsic parameters (everything needed for the projection
	d.AddMember("cam_coords", a, allocator);
	d.AddMember("cam_rotation", a, allocator);
	d.AddMember("cam_near_clip", 0.0, allocator);
	d.AddMember("cam_field_of_view", 0.0, allocator);

}

void Scenario::buildScenario() {
	Vector3 pos, rotation;
	Hash vehicleHash;
	float heading;

	GAMEPLAY::SET_RANDOM_SEED(std::time(NULL));
	while (!PATHFIND::_0xF7B79A50B905A30D(-8192.0f, 8192.0f, -8192.0f, 8192.0f)) WAIT(0);
	PATHFIND::GET_CLOSEST_VEHICLE_NODE_WITH_HEADING(x, y, 0, &pos, &heading, 0, 0, 0);

	LOG(INFO) << "22";
	ENTITY::DELETE_ENTITY(&vehicle);
	vehicleHash = GAMEPLAY::GET_HASH_KEY((char*)_vehicle);
	LOG(INFO) << "221";
	STREAMING::REQUEST_MODEL(vehicleHash);
	LOG(INFO) << "222";
	// while (!STREAMING::HAS_MODEL_LOADED(vehicleHash)) WAIT(0);
	LOG(INFO) << "223";
	while (!ENTITY::DOES_ENTITY_EXIST(vehicle)) {
		vehicle = VEHICLE::CREATE_VEHICLE(vehicleHash, pos.x, pos.y, pos.z, heading, FALSE, FALSE);
		WAIT(0);
	}

	VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(vehicle);

	LOG(INFO) << "33";
	while (!ENTITY::DOES_ENTITY_EXIST(ped)) {
		ped = PLAYER::PLAYER_PED_ID();
		WAIT(0);
	}
	LOG(INFO) << "44";
	player = PLAYER::PLAYER_ID();
	PLAYER::START_PLAYER_TELEPORT(player, pos.x, pos.y, pos.z, heading, 0, 0, 0);
	while (PLAYER::IS_PLAYER_TELEPORT_ACTIVE()) WAIT(0);
	LOG(INFO) << "55";
	PED::SET_PED_INTO_VEHICLE(ped, vehicle, -1);
	STREAMING::SET_MODEL_AS_NO_LONGER_NEEDED(vehicleHash);

	TIME::SET_CLOCK_TIME(hour, minute, 0);

	GAMEPLAY::SET_WEATHER_TYPE_NOW_PERSIST((char*)_weather);
	LOG(INFO) << "66";
	rotation = ENTITY::GET_ENTITY_ROTATION(vehicle, 1);
	CAM::DESTROY_ALL_CAMS(TRUE);
	camera = CAM::CREATE_CAM("DEFAULT_SCRIPTED_CAMERA", TRUE);
	if (strcmp(_vehicle, "packer") == 0) CAM::ATTACH_CAM_TO_ENTITY(camera, vehicle, 0, 2.35, 1.7, TRUE);
	else CAM::ATTACH_CAM_TO_ENTITY(camera, vehicle, 0, 0.5, 0.8, TRUE);
	CAM::SET_CAM_FOV(camera, 60);
	CAM::SET_CAM_ACTIVE(camera, TRUE);
	CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, rotation.z, 1);
	CAM::SET_CAM_INHERIT_ROLL_VEHICLE(camera, TRUE);
	CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, TRUE, TRUE);
	LOG(INFO) << "77";
	AI::CLEAR_PED_TASKS(ped);
	if (_drivingMode >= 0) AI::TASK_VEHICLE_DRIVE_WANDER(ped, vehicle, _setSpeed, _drivingMode);
}

void Scenario::start(const Value& sc, const Value& dc) {
	if (is_slow_motion) 
		GAMEPLAY::SET_TIME_SCALE(slow_rate);
	if (running) return;
	//Parse options
	srand(std::time(NULL));
	parseScenarioConfig(sc, true);
	parseDatasetConfig(dc, true);
	LOG(INFO) << "S4";

	//Build scenario
	buildScenario();
	LOG(INFO) << "S5";

	running = true;
	lastSafetyCheck = std::clock();
}

void Scenario::config(const Value& sc, const Value& dc) {
	if (!running) return;

	running = false;

	//Parse options
	srand(std::time(NULL));
	parseScenarioConfig(sc, false);
	parseDatasetConfig(dc, false);

	//Build scenario
	buildScenario();

	running = true;
	lastSafetyCheck = std::clock();
}

void Scenario::run() {
	if (running) {
		std::clock_t now = std::clock();

		Vector3 rotation = ENTITY::GET_ENTITY_ROTATION(vehicle, 1);
		CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, rotation.z, 1);

		if (_drivingMode < 0) {
			CONTROLS::_SET_CONTROL_NORMAL(27, 71, currentThrottle); //[0,1]
			CONTROLS::_SET_CONTROL_NORMAL(27, 72, currentBrake); //[0,1]
			CONTROLS::_SET_CONTROL_NORMAL(27, 59, currentSteering); //[-1,1]
		}
		
		float delay = ((float)(now - lastSafetyCheck)) / CLOCKS_PER_SEC;
		if (delay > 10) {
			lastSafetyCheck = std::clock();
			//Avoid bad things such as getting killed by the police, robbed, dying in car accidents or other horrible stuff
			PLAYER::SET_EVERYONE_IGNORE_PLAYER(player, TRUE);
			PLAYER::SET_POLICE_IGNORE_PLAYER(player, TRUE);
			PLAYER::CLEAR_PLAYER_WANTED_LEVEL(player); // Never wanted

			// Put on seat belt
			PED::SET_PED_CONFIG_FLAG(ped, 32, FALSE);

			// Invincible vehicle
			VEHICLE::SET_VEHICLE_TYRES_CAN_BURST(vehicle, FALSE);
			VEHICLE::SET_VEHICLE_WHEELS_CAN_BREAK(vehicle, FALSE);
			VEHICLE::SET_VEHICLE_HAS_STRONG_AXLES(vehicle, TRUE);

			VEHICLE::SET_VEHICLE_CAN_BE_VISIBLY_DAMAGED(vehicle, FALSE);
			ENTITY::SET_ENTITY_INVINCIBLE(vehicle, TRUE);
			ENTITY::SET_ENTITY_PROOFS(vehicle, 1, 1, 1, 1, 1, 1, 1, 1);

			// Player invincible
			PLAYER::SET_PLAYER_INVINCIBLE(player, TRUE);

			// Driving characteristics
			PED::SET_DRIVER_AGGRESSIVENESS(ped, 0.0);
			PED::SET_DRIVER_ABILITY(ped, 100.0);
		}
	}
	scriptWait(0);
}

void Scenario::stop() {
	GAMEPLAY::SET_TIME_SCALE(1.0);
	if (!running) return;
	running = false;
	CAM::DESTROY_ALL_CAMS(TRUE);
	CAM::RENDER_SCRIPT_CAMS(FALSE, TRUE, 500, FALSE, FALSE);
	AI::CLEAR_PED_TASKS(ped);
	setCommands(0.0, 0.0, 0.0);
}

void Scenario::setCommands(float throttle, float brake, float steering) {
	currentThrottle = throttle;
	currentBrake = brake;
	currentSteering = steering;
}

StringBuffer Scenario::generateMessage() {
	StringBuffer buffer;
	buffer.Clear();
	Writer<StringBuffer> writer(buffer);
	
	// GAMEPLAY::SET_TIME_SCALE(0);

	if (vehicles) setVehiclesList();
	if (peds) setPedsList();
	if (trafficSigns); //TODO
	if (direction) setDirection();
	if (throttle) setThrottle();
	if (brake) setBrake();
	if (steering) setSteering();
	if (speed) setSpeed();
	if (yawRate) setYawRate();
	if (drivingMode); //TODO
	if (location) setLocation();
	if (time) setTime();
	if (collision) setCollision();
	// camera intrinsic parameters
	setCamIntrinsic();

	d.Accept(writer);
	return buffer;
}

void Scenario::setVehiclesList() {
	const int ARR_SIZE = 1024;
	if (firstVehicle){
		vehicles_list = new Vehicle[ARR_SIZE];
		firstVehicle = false;
	}
	Value _vehicles(kArrayType);
	Document::AllocatorType& allocator = d.GetAllocator();

	Vector3 FUR; //Front Upper Right
	Vector3 BLL; //Back Lower Lelft
	Vector3 dim; //Vehicle dimensions
	Vector3 upVector, rightVector, forwardVector, position; //Vehicle position
	Hash model;
	Vector3 min;
	Vector3 max;
	Vector3 speedVector;
	float heading, speed;
	int classid;

	Vector3 currentPos = ENTITY::GET_ENTITY_COORDS(vehicle, false);
	Vector3 currentForwardVector = ENTITY::GET_ENTITY_FORWARD_VECTOR(vehicle);
	
	// needed for projection
	Vector3 cam_rot = CAM::GET_CAM_ROT(camera, 0);
	Vector3 cam_pos = CAM::GET_CAM_COORD(camera);
	float fov = CAM::GET_CAM_FOV(camera);
	float near_clip = CAM::GET_CAM_NEAR_CLIP(camera);
	
	int count = worldGetAllVehicles(vehicles_list, ARR_SIZE);
	if (count > ARR_SIZE)
		count = ARR_SIZE;
	for (int i = 0; i < count; i++) {
		if (vehicles_list[i] == vehicle) continue; //Don't process own car!
		if (ENTITY::IS_ENTITY_ON_SCREEN(vehicles_list[i])) {
			//Check if it is in screen
			ENTITY::GET_ENTITY_MATRIX(vehicles_list[i], &rightVector, &forwardVector, &upVector, &position); //Blue or red pill
			if (SYSTEM::VDIST2(currentPos.x, currentPos.y, currentPos.z, position.x, position.y, position.z) < 22500) { //150 m.
			// if(true){
				if (true){
					bool visible = ENTITY::HAS_ENTITY_CLEAR_LOS_TO_ENTITY(vehicle, vehicles_list[i], 19);
					model = ENTITY::GET_ENTITY_MODEL(vehicles_list[i]);
					int vec_hash = (int)model;
					int vec_class = VEHICLE::GET_VEHICLE_CLASS(vehicles_list[i]);
					GAMEPLAY::GET_MODEL_DIMENSIONS(model, &min, &max);

					speedVector = ENTITY::GET_ENTITY_SPEED_VECTOR(vehicles_list[i], false);
					speed = ENTITY::GET_ENTITY_SPEED(vehicles_list[i]);
					if (speed > 0) {
						heading = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(speedVector.x - currentForwardVector.x, speedVector.y - currentForwardVector.y);
					}
					else {
						heading = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(forwardVector.x - currentForwardVector.x, forwardVector.y - currentForwardVector.y);
					}

					if (VEHICLE::IS_THIS_MODEL_A_CAR(model)) classid = 0;
					else if (VEHICLE::IS_THIS_MODEL_A_BIKE(model)) classid = 1;
					else if (VEHICLE::IS_THIS_MODEL_A_BICYCLE(model)) classid = 2;
					else if (VEHICLE::IS_THIS_MODEL_A_QUADBIKE(model)) classid = 3;
					else if (VEHICLE::IS_THIS_MODEL_A_BOAT(model)) classid = 4;
					else if (VEHICLE::IS_THIS_MODEL_A_PLANE(model)) classid = 5;
					else if (VEHICLE::IS_THIS_MODEL_A_HELI(model)) classid = 6;
					else if (VEHICLE::IS_THIS_MODEL_A_TRAIN(model)) classid = 7;
					else if (VEHICLE::_IS_THIS_MODEL_A_SUBMERSIBLE(model)) classid = 8;
					else classid = 9; //unknown (ufo?)

					
					//Calculate size
					dim.x = 0.5*(max.x - min.x);
					dim.y = 0.5*(max.y - min.y);
					dim.z = 0.5*(max.z - min.z);
					

					FUR.x = position.x + dim.y*rightVector.x + dim.x*forwardVector.x + dim.z*upVector.x;
					FUR.y = position.y + dim.y*rightVector.y + dim.x*forwardVector.y + dim.z*upVector.y;
					FUR.z = position.z + dim.y*rightVector.z + dim.x*forwardVector.z + dim.z*upVector.z;
					//GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(FUR.x, FUR.y, FUR.z, &(FUR.z), 0);
					//FUR.z += 2 * dim.z;

					BLL.x = position.x - dim.y*rightVector.x - dim.x*forwardVector.x - dim.z*upVector.x;
					BLL.y = position.y - dim.y*rightVector.y - dim.x*forwardVector.y - dim.z*upVector.y;
					BLL.z = position.z - dim.y*rightVector.z - dim.x*forwardVector.z - dim.z*upVector.z;
					//GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(BLL.x, BLL.y, 1000.0, &(BLL.z), 0);

					Vector3 pp1 = BLL;
					Vector3 pp2;
					Vector3 pp3;
					Vector3 pp4;
					Vector3 pp5 = FUR;
					Vector3 pp6;
					Vector3 pp7;
					Vector3 pp8;

					pp2.x = pp1.x + 2 * dim.y*rightVector.x;
					pp2.y = pp1.y + 2 * dim.y*rightVector.y;
					pp2.z = pp1.z + 2 * dim.y*rightVector.z;

					pp3.x = pp2.x + 2 * dim.z*upVector.x;
					pp3.y = pp2.y + 2 * dim.z*upVector.y;
					pp3.z = pp2.z + 2 * dim.z*upVector.z;

					pp4.x = pp1.x + 2 * dim.z*upVector.x;
					pp4.y = pp1.y + 2 * dim.z*upVector.y;
					pp4.z = pp1.z + 2 * dim.z*upVector.z;

					pp6.x = pp5.x - 2 * dim.y*rightVector.x;
					pp6.y = pp5.y - 2 * dim.y*rightVector.y;
					pp6.z = pp5.z - 2 * dim.y*rightVector.z;

					pp7.x = pp6.x - 2 * dim.z*upVector.x;
					pp7.y = pp6.y - 2 * dim.z*upVector.y;
					pp7.z = pp6.z - 2 * dim.z*upVector.z;

					pp8.x = pp5.x - 2 * dim.z*upVector.x;
					pp8.y = pp5.y - 2 * dim.z*upVector.y;
					pp8.z = pp5.z - 2 * dim.z*upVector.z;

					bool correct_forward = true;  // actual right
					bool correct_right = true;  // actual forward
					bool correct_up = true;
					// correction in forward direction (actual right)
					if (correct_forward){
						float forward_shift = 0.5*(max.x + min.x);
						pp1.x = pp1.x + forward_shift * forwardVector.x;
						pp1.y = pp1.y + forward_shift * forwardVector.y;
						pp1.z = pp1.z + forward_shift * forwardVector.z;
						pp2.x = pp2.x + forward_shift * forwardVector.x;
						pp2.y = pp2.y + forward_shift * forwardVector.y;
						pp2.z = pp2.z + forward_shift * forwardVector.z;
						pp3.x = pp3.x + forward_shift * forwardVector.x;
						pp3.y = pp3.y + forward_shift * forwardVector.y;
						pp3.z = pp3.z + forward_shift * forwardVector.z;
						pp4.x = pp4.x + forward_shift * forwardVector.x;
						pp4.y = pp4.y + forward_shift * forwardVector.y;
						pp4.z = pp4.z + forward_shift * forwardVector.z;
						pp5.x = pp5.x + forward_shift * forwardVector.x;
						pp5.y = pp5.y + forward_shift * forwardVector.y;
						pp5.z = pp5.z + forward_shift * forwardVector.z;
						pp6.x = pp6.x + forward_shift * forwardVector.x;
						pp6.y = pp6.y + forward_shift * forwardVector.y;
						pp6.z = pp6.z + forward_shift * forwardVector.z;
						pp7.x = pp7.x + forward_shift * forwardVector.x;
						pp7.y = pp7.y + forward_shift * forwardVector.y;
						pp7.z = pp7.z + forward_shift * forwardVector.z;
						pp8.x = pp8.x + forward_shift * forwardVector.x;
						pp8.y = pp8.y + forward_shift * forwardVector.y;
						pp8.z = pp8.z + forward_shift * forwardVector.z;
					}
					// correction in right direction (actual forward)
					if (correct_right){
						float right_shift = 0.5*(max.y + min.y);
						pp1.x = pp1.x + right_shift * rightVector.x;
						pp1.y = pp1.y + right_shift * rightVector.y;
						pp1.z = pp1.z + right_shift * rightVector.z;
						pp2.x = pp2.x + right_shift * rightVector.x;
						pp2.y = pp2.y + right_shift * rightVector.y;
						pp2.z = pp2.z + right_shift * rightVector.z;
						pp3.x = pp3.x + right_shift * rightVector.x;
						pp3.y = pp3.y + right_shift * rightVector.y;
						pp3.z = pp3.z + right_shift * rightVector.z;
						pp4.x = pp4.x + right_shift * rightVector.x;
						pp4.y = pp4.y + right_shift * rightVector.y;
						pp4.z = pp4.z + right_shift * rightVector.z;
						pp5.x = pp5.x + right_shift * rightVector.x;
						pp5.y = pp5.y + right_shift * rightVector.y;
						pp5.z = pp5.z + right_shift * rightVector.z;
						pp6.x = pp6.x + right_shift * rightVector.x;
						pp6.y = pp6.y + right_shift * rightVector.y;
						pp6.z = pp6.z + right_shift * rightVector.z;
						pp7.x = pp7.x + right_shift * rightVector.x;
						pp7.y = pp7.y + right_shift * rightVector.y;
						pp7.z = pp7.z + right_shift * rightVector.z;
						pp8.x = pp8.x + right_shift * rightVector.x;
						pp8.y = pp8.y + right_shift * rightVector.y;
						pp8.z = pp8.z + right_shift * rightVector.z;
					}
					// correction in up direction
					if (correct_up){
						float up_shift = 0.5*(max.z + min.z);
						pp1.x = pp1.x + up_shift * upVector.x;
						pp1.y = pp1.y + up_shift * upVector.y;
						pp1.z = pp1.z + up_shift * upVector.z;
						pp2.x = pp2.x + up_shift * upVector.x;
						pp2.y = pp2.y + up_shift * upVector.y;
						pp2.z = pp2.z + up_shift * upVector.z;
						pp3.x = pp3.x + up_shift * upVector.x;
						pp3.y = pp3.y + up_shift * upVector.y;
						pp3.z = pp3.z + up_shift * upVector.z;
						pp4.x = pp4.x + up_shift * upVector.x;
						pp4.y = pp4.y + up_shift * upVector.y;
						pp4.z = pp4.z + up_shift * upVector.z;
						pp5.x = pp5.x + up_shift * upVector.x;
						pp5.y = pp5.y + up_shift * upVector.y;
						pp5.z = pp5.z + up_shift * upVector.z;
						pp6.x = pp6.x + up_shift * upVector.x;
						pp6.y = pp6.y + up_shift * upVector.y;
						pp6.z = pp6.z + up_shift * upVector.z;
						pp7.x = pp7.x + up_shift * upVector.x;
						pp7.y = pp7.y + up_shift * upVector.y;
						pp7.z = pp7.z + up_shift * upVector.z;
						pp8.x = pp8.x + up_shift * upVector.x;
						pp8.y = pp8.y + up_shift * upVector.y;
						pp8.z = pp8.z + up_shift * upVector.z;
					}

#ifdef DEBUG
					GRAPHICS::DRAW_LINE(pp1.x, pp1.y, pp1.z, pp2.x, pp2.y, pp2.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp1.x, pp1.y, pp1.z, pp4.x, pp4.y, pp4.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp2.x, pp2.y, pp2.z, pp3.x, pp3.y, pp3.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp3.x, pp3.y, pp3.z, pp4.x, pp4.y, pp4.z, 0, 255, 0, 200);

					GRAPHICS::DRAW_LINE(pp5.x, pp5.y, pp5.z, pp6.x, pp6.y, pp6.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp5.x, pp5.y, pp5.z, pp8.x, pp8.y, pp8.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp6.x, pp6.y, pp6.z, pp7.x, pp7.y, pp7.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp7.x, pp7.y, pp7.z, pp8.x, pp8.y, pp8.z, 0, 255, 0, 200);

					GRAPHICS::DRAW_LINE(pp1.x, pp1.y, pp1.z, pp7.x, pp7.y, pp7.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp2.x, pp2.y, pp2.z, pp8.x, pp8.y, pp8.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp3.x, pp3.y, pp3.z, pp5.x, pp5.y, pp5.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp4.x, pp4.y, pp4.z, pp6.x, pp6.y, pp6.z, 0, 255, 0, 200);
#endif

					float x_tmp, y_tmp;

					Value xxx(kArrayType); 
					Value yyy(kArrayType);
					Value zzz(kArrayType);

					float interpRatio = 0.5;

					
					
					Value _vehicle(kObjectType);


					_vehicle.AddMember("speed", speed, allocator).AddMember("heading", heading, allocator).AddMember("classID", classid, allocator);
					_vehicle.AddMember("hash", vec_hash, allocator).AddMember("class", vec_class, allocator);
					_vehicle.AddMember("visible", visible, allocator);
					_vehicle.AddMember("arrID", (int)vehicles_list[i], allocator); 

					Value _vector(kArrayType);
					_vector.PushBack(position.x, allocator).PushBack(position.y, allocator).PushBack(position.z, allocator);
					_vehicle.AddMember("your_pos", _vector, allocator);

					
					xxx.PushBack(pp1.x, allocator).PushBack(pp2.x, allocator).PushBack(pp3.x, allocator).PushBack(pp4.x, allocator).PushBack(pp5.x, allocator)
						.PushBack(pp6.x, allocator).PushBack(pp7.x, allocator).PushBack(pp8.x, allocator);
					yyy.PushBack(pp1.y, allocator).PushBack(pp2.y, allocator).PushBack(pp3.y, allocator).PushBack(pp4.y, allocator).PushBack(pp5.y, allocator)
						.PushBack(pp6.y, allocator).PushBack(pp7.y, allocator).PushBack(pp8.y, allocator);
					zzz.PushBack(pp1.z, allocator).PushBack(pp2.z, allocator).PushBack(pp3.z, allocator).PushBack(pp4.z, allocator).PushBack(pp5.z, allocator)
						.PushBack(pp6.z, allocator).PushBack(pp7.z, allocator).PushBack(pp8.z, allocator);

					_vehicle.AddMember("xxx", xxx, allocator);
					_vehicle.AddMember("yyy", yyy, allocator);
					_vehicle.AddMember("zzz", zzz, allocator);
					

					_vehicles.PushBack(_vehicle, allocator);
					 
					

					

				}
			}
		}
	}
	d["vehicles"] = _vehicles;
}

void Scenario::setPedsList(){
	const int ARR_SIZE = 1024;
	if (firstPed){
		peds_list = new Ped[ARR_SIZE];
		firstPed = false;
	}
	
	Value _peds(kArrayType);  
	Document::AllocatorType& allocator = d.GetAllocator();

	Vector3 FUR; //Front Upper Right
	Vector3 BLL; //Back Lower Lelft
	Vector3 dim; //Vehicle dimensions
	Vector3 upVector, rightVector, forwardVector, position; //Vehicle position
	Hash model;
	Vector3 min;
	Vector3 max;
	Vector3 speedVector;
	float heading, speed;
	int classid;

	Vector3 currentPos = ENTITY::GET_ENTITY_COORDS(vehicle, false);
	Vector3 currentForwardVector = ENTITY::GET_ENTITY_FORWARD_VECTOR(vehicle);

	int count = worldGetAllPeds(peds_list, ARR_SIZE);
	if (count > ARR_SIZE)
		count = ARR_SIZE;
	for (int i = 0; i < count; i++) {
		if (PED::IS_PED_IN_ANY_VEHICLE(peds_list[i], TRUE)) continue; //Don't process peds in vehicles!
		if (ENTITY::IS_ENTITY_ON_SCREEN(peds_list[i])) {
			//Check if it is in screen
			ENTITY::GET_ENTITY_MATRIX(peds_list[i], &rightVector, &forwardVector, &upVector, &position); //Blue or red pill
			if (SYSTEM::VDIST2(currentPos.x, currentPos.y, currentPos.z, position.x, position.y, position.z) < 22500) { //150 m.
				if (true){
					bool visible = ENTITY::HAS_ENTITY_CLEAR_LOS_TO_ENTITY(ped, peds_list[i], 19);
					if (PED::GET_PED_TYPE(peds_list[i]) == 28) {
						// classid = 11; //animal
						continue;
					}
					else classid = 10;

					model = ENTITY::GET_ENTITY_MODEL(peds_list[i]);
					GAMEPLAY::GET_MODEL_DIMENSIONS(model, &min, &max);

					speedVector = ENTITY::GET_ENTITY_SPEED_VECTOR(peds_list[i], false);
					speed = ENTITY::GET_ENTITY_SPEED(peds_list[i]);
					if (speed > 0) {
						heading = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(speedVector.x - currentForwardVector.x, speedVector.y - currentForwardVector.y);
					}
					else {
						heading = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(forwardVector.x - currentForwardVector.x, forwardVector.y - currentForwardVector.y);
					}

					

					//Calculate size
					dim.x = 0.5*(max.x - min.x);
					dim.y = 0.5*(max.y - min.y);
					dim.z = 0.5*(max.z - min.z);

					FUR.x = position.x + dim.y*rightVector.x + dim.x*forwardVector.x + dim.z*upVector.x;
					FUR.y = position.y + dim.y*rightVector.y + dim.x*forwardVector.y + dim.z*upVector.y;
					FUR.z = position.z + dim.y*rightVector.z + dim.x*forwardVector.z + dim.z*upVector.z;
					//GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(FUR.x, FUR.y, 1000.0, &(FUR.z), 0);
					//FUR.z += 2 * dim.z;

					BLL.x = position.x - dim.y*rightVector.x - dim.x*forwardVector.x - dim.z*upVector.x;
					BLL.y = position.y - dim.y*rightVector.y - dim.x*forwardVector.y - dim.z*upVector.y;
					BLL.z = position.z - dim.y*rightVector.z - dim.x*forwardVector.z - dim.z*upVector.z;
					//GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(BLL.x, BLL.y, 1000.0, &(BLL.z), 0);

					Vector3 pp1 = BLL;
					Vector3 pp2;
					Vector3 pp3;
					Vector3 pp4;
					Vector3 pp5 = FUR;
					Vector3 pp6;
					Vector3 pp7;
					Vector3 pp8;

					pp2.x = pp1.x + 2 * dim.y*rightVector.x;
					pp2.y = pp1.y + 2 * dim.y*rightVector.y;
					pp2.z = pp1.z + 2 * dim.y*rightVector.z;

					pp3.x = pp2.x + 2 * dim.z*upVector.x;
					pp3.y = pp2.y + 2 * dim.z*upVector.y;
					pp3.z = pp2.z + 2 * dim.z*upVector.z;

					pp4.x = pp1.x + 2 * dim.z*upVector.x;
					pp4.y = pp1.y + 2 * dim.z*upVector.y;
					pp4.z = pp1.z + 2 * dim.z*upVector.z;

					pp6.x = pp5.x - 2 * dim.y*rightVector.x;
					pp6.y = pp5.y - 2 * dim.y*rightVector.y;
					pp6.z = pp5.z - 2 * dim.y*rightVector.z;

					pp7.x = pp6.x - 2 * dim.z*upVector.x;
					pp7.y = pp6.y - 2 * dim.z*upVector.y;
					pp7.z = pp6.z - 2 * dim.z*upVector.z;

					pp8.x = pp5.x - 2 * dim.z*upVector.x;
					pp8.y = pp5.y - 2 * dim.z*upVector.y;
					pp8.z = pp5.z - 2 * dim.z*upVector.z;

					
					Value xxx(kArrayType);
					Value yyy(kArrayType);
					Value zzz(kArrayType);

					Value _ped(kObjectType);


					_ped.AddMember("speed", speed, allocator).AddMember("heading", heading, allocator).AddMember("classID", classid, allocator);
					_ped.AddMember("visible", visible, allocator);
					_ped.AddMember("arrID", (int)peds_list[i], allocator);

					Value _vector(kArrayType);
					_vector.PushBack(position.x, allocator).PushBack(position.y, allocator).PushBack(position.z, allocator);
					_ped.AddMember("your_pos", _vector, allocator);


					xxx.PushBack(pp1.x, allocator).PushBack(pp2.x, allocator).PushBack(pp3.x, allocator).PushBack(pp4.x, allocator).PushBack(pp5.x, allocator)
						.PushBack(pp6.x, allocator).PushBack(pp7.x, allocator).PushBack(pp8.x, allocator);
					yyy.PushBack(pp1.y, allocator).PushBack(pp2.y, allocator).PushBack(pp3.y, allocator).PushBack(pp4.y, allocator).PushBack(pp5.y, allocator)
						.PushBack(pp6.y, allocator).PushBack(pp7.y, allocator).PushBack(pp8.y, allocator);
					zzz.PushBack(pp1.z, allocator).PushBack(pp2.z, allocator).PushBack(pp3.z, allocator).PushBack(pp4.z, allocator).PushBack(pp5.z, allocator)
						.PushBack(pp6.z, allocator).PushBack(pp7.z, allocator).PushBack(pp8.z, allocator);

					_ped.AddMember("xxx", xxx, allocator);
					_ped.AddMember("yyy", yyy, allocator);
					_ped.AddMember("zzz", zzz, allocator);


					_peds.PushBack(_ped, allocator);


					#ifdef DEBUG
					GRAPHICS::DRAW_LINE(pp1.x, pp1.y, pp1.z, pp2.x, pp2.y, pp2.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp1.x, pp1.y, pp1.z, pp4.x, pp4.y, pp4.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp2.x, pp2.y, pp2.z, pp3.x, pp3.y, pp3.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp3.x, pp3.y, pp3.z, pp4.x, pp4.y, pp4.z, 0, 255, 0, 200);

					GRAPHICS::DRAW_LINE(pp5.x, pp5.y, pp5.z, pp6.x, pp6.y, pp6.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp5.x, pp5.y, pp5.z, pp8.x, pp8.y, pp8.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp6.x, pp6.y, pp6.z, pp7.x, pp7.y, pp7.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp7.x, pp7.y, pp7.z, pp8.x, pp8.y, pp8.z, 0, 255, 0, 200);

					GRAPHICS::DRAW_LINE(pp1.x, pp1.y, pp1.z, pp7.x, pp7.y, pp7.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp2.x, pp2.y, pp2.z, pp8.x, pp8.y, pp8.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp3.x, pp3.y, pp3.z, pp5.x, pp5.y, pp5.z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(pp4.x, pp4.y, pp4.z, pp6.x, pp6.y, pp6.z, 0, 255, 0, 200);
					#endif

				}
			}
		}
	}		
	d["peds"] = _peds;
}


void Scenario::setThrottle(){
	int a = 0x8FC;
	d["throttle"] = getFloatValue(vehicle, 0x90C); // 0x8FC
	// d["throttle"] = CONTROLS::GET_CONTROL_NORMAL(0, 71); // VehicleAccelerate = 71 USEFUL
	// d["throttle"] = VEHICLE::GET_VEHICLE_ACCELERATION(vehicle);
}

void Scenario::setBrake(){
	d["brake"] = getFloatValue(vehicle, 0x910); // 0x900
	// d["brake"] = CONTROLS::GET_CONTROL_NORMAL(0, 72); // VehicleBrake = 72
}

void Scenario::setSteering(){
	d["steering"] = getFloatValue(vehicle, 0x904) / -0.7; // 0x8F4 
	// d["steering"] = CONTROLS::GET_CONTROL_NORMAL(0, 59); // VehicleMoveLeftRight = 59,
}

void Scenario::setSpeed(){
	d["speed"] = ENTITY::GET_ENTITY_SPEED(vehicle);
}

void Scenario::setYawRate(){
	Vector3 rates = ENTITY::GET_ENTITY_ROTATION_VELOCITY(vehicle);
	d["yawRate"] = rates.z*180.0 / 3.14159265359;
}

void Scenario::setLocation(){
	Document::AllocatorType& allocator = d.GetAllocator();
	Vector3 pos = ENTITY::GET_ENTITY_COORDS(vehicle, false);
	Value location(kArrayType);
	location.PushBack(pos.x, allocator).PushBack(pos.y, allocator).PushBack(pos.z, allocator);
	d["location"] = location;
}

void Scenario::setTime(){
	d["time"] = TIME::GET_CLOCK_HOURS();
}

void Scenario::setDirection(){
	int direction;
	float distance;
	Document::AllocatorType& allocator = d.GetAllocator();
	PATHFIND::GENERATE_DIRECTIONS_TO_COORD(dir.x, dir.y, dir.z, TRUE, &direction, &vehicle, &distance);
	Value _direction(kArrayType);
	_direction.PushBack(direction, allocator).PushBack(distance, allocator);
	d["direction"] = _direction;
}



void Scenario::setCollision(){
	d["collision"] = ENTITY::HAS_ENTITY_COLLIDED_WITH_ANYTHING(vehicle);
}

void Scenario::setRotation(){
	Document::AllocatorType& allocator = d.GetAllocator();
	Vector3 rot = ENTITY::GET_ENTITY_ROTATION(vehicle, 1);

	Value rotation(kArrayType);
	rotation.PushBack(rot.x, allocator).PushBack(rot.y, allocator).PushBack(rot.z, allocator);
	d["rotation"] = rotation;
}

void Scenario::setCamIntrinsic() {
	Document::AllocatorType& allocator = d.GetAllocator();
	// cam coord
	Vector3 cam_coords = CAM::GET_CAM_COORD(camera);
	Value cam_coord_list(kArrayType);
	cam_coord_list.PushBack(cam_coords.x, allocator).PushBack(cam_coords.y, allocator).PushBack(cam_coords.z, allocator);
	d["cam_coords"] = cam_coord_list;
	// cam rotation
	Vector3 cam_rot = CAM::GET_CAM_ROT(camera, 0);
	Value cam_rot_list(kArrayType);
	cam_rot_list.PushBack(cam_rot.x, allocator).PushBack(cam_rot.y, allocator).PushBack(cam_rot.z, allocator);
	d["cam_rotation"] = cam_rot_list;
	// near clip
	float near_clip = CAM::GET_CAM_NEAR_CLIP(camera);
	d["cam_near_clip"] = near_clip;
	// field of view
	d["cam_field_of_view"] = CAM::GET_CAM_FOV(camera);
	
}
