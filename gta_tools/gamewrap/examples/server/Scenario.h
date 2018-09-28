#pragma once

#include <stdlib.h>
#include <ctime>

// #include "lib/script.h"
#include "lib/utils.h"

#include "lib/rapidjson/document.h"
#include "lib/rapidjson/stringbuffer.h"
// #include "lib/rapidjson/error/en.h"

#include "log.h"

#include <Psapi.h>

using namespace rapidjson;

//#define DEBUG 1

class Scenario {
private:
	bool is_slow_motion = false;
	float slow_rate = 0.5;

	static char* weatherList[14];
	static char* vehicleList[4];

	Vehicle vehicle = NULL;
	Player player = NULL;
	Ped ped = NULL;
	Cam camera = NULL;
	Vector3 dir;

	float x, y;
	int hour, minute;
	const char* _weather;
	const char* _vehicle;
	int width, height;

	bool vehicles;
	bool peds;
	bool trafficSigns; //TODO
	bool direction;
	bool throttle;
	bool brake;
	bool steering;
	bool speed;
	bool yawRate;
	bool drivingMode; //TODO
	bool location;
	bool time;
	bool collision;

	float currentThrottle = 0.0;
	float currentBrake = 0.0;
	float currentSteering = 0.0;

	std::clock_t lastSafetyCheck;
	int _drivingMode;
	float _setSpeed;

	bool running = false;
	Document d;

public:
	int rate;

	void start(const Value& sc, const Value& dc);
	void stop();
	void config(const Value& sc, const Value& dc);
	void setCommands(float throttle, float brake, float steering);
	void run();

	StringBuffer generateMessage();

private:
	void parseScenarioConfig(const Value& sc, bool setDefaults);
	void parseDatasetConfig(const Value& dc, bool setDefaults);
	void buildScenario();

	void setVehiclesList();
	void setPedsList();
	void setTrafficSignsList();
	void setDirection();
	void setThrottle();
	void setBrake();
	void setSteering();
	void setSpeed();
	void setYawRate();
	void setDrivingMode();
	void setLocation();
	void setTime();
	void setCollision();
	void setRotation();
	void setCamIntrinsic();
	
	bool firstVehicle = true;
	bool firstPed = true;

	Vehicle* vehicles_list;
	Ped* peds_list;

	// for camera matrix
	Vector3 cam_right;
	Vector3 cam_forward;
	Vector3 cam_up;
	Vector3 cam_pos;

	
};