#include "ros/ros.h"
#include <iostream>
#include "unistd.h"
#include "rpc/client.h"
#include <string>
#include <path_follower/Trajectory2D.h>
#include <path_follower/state_Dynamic.h>
#include <path_follower/TrajectoryPoint2D.h>
#include <time.h>
#include <std_msgs/Float64.h>
//#include <path_follower/vehicleinfo.h>
using namespace std;

#define PORT (8086)
static char sep = '&';
static char stop = '|';

path_follower::state_Dynamic state;
path_follower::Trajectory2D tra;
bool if_get = false;

void CmdCallback(const path_follower::state_Dynamic msg)
{
  state = msg;
}

void SteeringCallback(const path_follower::Trajectory2D msg)
{
  tra = msg;
  if_get = true;
}

void Dynamic_tostring(std::string* str){
    *str = std::to_string(state.vx) + sep + std::to_string(state.vy) + sep + std::to_string(state.X) + sep + std::to_string(state.Y) + sep + std::to_string(state.psi) + sep + std::to_string(state.wz) + stop;
}
void Trajectory2D_tostring(std::string* str1){
    int i=1;
    //double data;
    //ROS_INFO_STREAM_ONCE("6.2");
    //data = tra.point[i].t;
    //ROS_INFO_STREAM_ONCE("6.5");
    *str1 = std::to_string(tra.point[i].t) + sep + std::to_string(tra.point[i].x) + sep + std::to_string(tra.point[i].y) + sep + std::to_string(tra.point[i].theta) + sep + std::to_string(tra.point[i].kappa) + sep
            + std::to_string(tra.point[i].kappa_dot) + sep + std::to_string(tra.point[i].v) + sep + std::to_string(tra.point[i].a) + sep + std::to_string(tra.point[i].jerk) + sep + std::to_string(tra.point[i].delta_theta) + sep
                + std::to_string(tra.point[i].d) + sep + std::to_string(tra.point[i].a_lat) + stop + "\n";
}


int main(int argc, char **argv) {
    
    std_msgs::Float64 time1, time2;
    ros::init(argc, argv, "client");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("state_estimate",1,CmdCallback); 
    ros::Subscriber sub_steering_cmd = n.subscribe("final_trajectory",1,SteeringCallback);
    ros::Publisher pub1 = n.advertise<std_msgs::Float64>("time1",1);
    ros::Publisher pub2 = n.advertise<std_msgs::Float64>("time2",1);
    rpc::client my_client("128.32.164.36", PORT);
    ros::Rate loop_rate(10);
    int i;
    std::string num;
    int length;

    while(ros::ok())
    {
        ros::spinOnce();
        if(if_get){
            clock_t start1, start2, stop1, stop2;
            std::string str1 = "",str2 = "";
            Dynamic_tostring(&str1);
            Trajectory2D_tostring(&str2);
            
            //send string
            std::string send = str1 + str2;
            string infoma = " ";
            
            //connection
	    start1 = clock();
            std::string str = my_client.call("echo", send).as<std::string>();
	    stop1 = clock();
            ROS_INFO_STREAM_ONCE("8");

            //cout
            std::cout << "Send successfully!" << "\n" << send << std::endl;

            //get the number of cars
	    start2 = clock();
            std::string info = my_client.call("getinfo").as<std::string>();
	    stop2 = clock();
            std::cout << "Get successfully!" << "\n" << info << std::endl;

            //for(int a = 0; a < 5; a++)
            //{
            //	if(info[a] == '&')
            //		{i = a;}
            //}
            //num = info.substr(0,i);
            //i = atoi(const_cast<const char *>(num.c_str()));

            //get other infos
            //double  *p=new double[6*i];
            //std::string  *q=new std::string[6*i];
            //int  *index=new int[6*i + 1];
            //length = info.length();
            //int k = 0;

            //for(int a = 0; a < length; a++)
            //{
            //	if(info[a] == '&')
            //		{ 
            //			index[k] = a;
            //			k++;
            //		}
            //}

            //for(int a = 0; a < 6*i; a++)
            //{
            //	q[a] = info.substr(index[a]+1,index[a+1]);
            //	p[a] = atof(const_cast<const char *>(q[a].c_str()));
            //}
	double p = atof(const_cast<const char *>(info.c_str()));
	double timeone = ((double)stop1 - (double)start1) / double(CLOCKS_PER_SEC);
	double timetwo = ((double)stop2 - (double)start2) / double(CLOCKS_PER_SEC);
	time1.data = timeone;
	time2.data = timetwo;
	pub1.publish(time1);
	pub2.publish(time2);


            //delete[]  p;
            //delete[]  q;
            //delete[]  index;

        }
        else{
            continue;
        }

        loop_rate.sleep();
    }
    return 0;
}

