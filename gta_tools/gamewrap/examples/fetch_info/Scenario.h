#pragma once

#include <stdlib.h>
#include <ctime>
#include <chrono>

// #include "lib/script.h"
#include "lib/utils.h"

#include "lib/rapidjson/document.h"
#include "lib/rapidjson/stringbuffer.h"

#include "Rewarders\Rewarder.h"
#include "Rewarders\LaneRewarder.h"
#include "Rewarders\GeneralRewarder.h"

#include <Psapi.h>

using namespace rapidjson;

//#define DEBUG 1

class Scenario {
private:
	bool is_slow_motion = true;
	float slow_rate = 1.0;

	static char* weatherList[14];
	static char* vehicleList[4];

	Vehicle vehicle = NULL;
	Player player = NULL;
	Ped ped = NULL;
	Cam camera = NULL;
	Cam birdcamera = NULL;
	Vector3 dir;

	float x, y;
	int hour, minute;
	const char* _weather;
	const char* _vehicle;
	int width, height;

	bool inexperienced=false;
	bool vehicles;
	bool peds;
	bool trafficSigns; //TODO
	bool direction;
	bool reward;
	bool throttle;
	bool brake;
	bool steering;
	bool speed;
	bool yawRate;
	bool drivingMode; //TODO
	bool location;
	bool time;
	bool collision;
	bool roadinfo;

	float currentThrottle = 0.0;
	float currentBrake = 0.0;
	float currentSteering = 0.0;

	GeneralRewarder* rewarder;
	std::clock_t lastSafetyCheck;
	int _drivingMode;
	float _setSpeed;

	bool running = false;
	bool bird = false;
	Document d;
	LaneRewarder* lanerewarder;

public:
	int rate;

	void start(const Value& sc, const Value& dc);
	void stop();
	void setCommands(float throttle, float brake, float steering, int manual);
	void config(const Value& sc, const Value& dc);
	void run();
	StringBuffer generateMessage();

	void initRewarder();

private:
	void parseScenarioConfig(const Value& sc, bool setDefaults);
	void parseDatasetConfig(const Value& dc, bool setDefaults);
	void buildScenario();

	void setVehiclesList();
	void setPedsList();
	void setTrafficSignsList();
	void setDirection();
	void setReward();
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
	void setTimestamp();
	void setroadinfo();
	// void setCalibration();
	// Eigen::Vector2f get_2d_from_3d(const Eigen::Vector3f& vertex, const Eigen::Vector3f& cam_coords, const Eigen::Vector3f& cam_rotation, float cam_near_clip, float cam_field_of_view, double* depth);
	// Eigen::Vector3f rotate(Eigen::Vector3f a, Eigen::Vector3f theta);

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