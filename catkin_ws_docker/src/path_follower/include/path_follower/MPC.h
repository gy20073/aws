/********************************************************
  Autonomous Driving Software
  Copyright (c) 2017 MSC Lab, UC Berkeley 
  All rights reserved.

 ********************************************************/

#ifndef MPC_LQR_H
#define MPC_LQR_H

#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <algorithm>

#include <dbw_mkz_msgs/SteeringReport.h>
#include <path_follower/SteeringCmd.h>
#include <path_follower/ApplanixPose.h>
#include <path_follower/Trajectory2D.h>
#include <path_follower/TrajectoryPoint2D.h>
#include <path_follower/state_Dynamic.h>
#include <path_follower/vlrException.h>
#include <path_follower/Vehicle.h>
#include <path_follower/scaledTime.h>
#include <dynamic_reconfigure/server.h>
#include <qpOASES/qpOASES.hpp>
USING_NAMESPACE_QPOASES

#define NUM_STATES 6
#define NUM_CONTROLS 2
#define NUM_EXT_STATES (NUM_STATES+2*NUM_CONTROLS)
#define NUM_EEXT_STATES (NUM_STATES+2*NUM_CONTROLS+1)
#define EPSILON 1e-5

double p_hertz = 20;
double ros_hertz = 20;
double dt = 1.0 / p_hertz;
int p_horizon = 20;
int c_horizon = 20;
int i_horizon = 1;
bool kappa_flag = false;
bool nearest_flag = true;
double p_q_lon = 0.2;
double p_q_lat = 0.3;
double p_q_theta = 0.3;
double p_q_u = 0.1;
double p_q_v = 0.01;
double p_q_theta_dot = 0.8;
double p_r_udot = 0.01;
double p_r_delta = 0.1;
double p_rd_udot = 0.01;
double p_rd_delta = 12.0;

namespace vlr {

    void Array2realt(Eigen::ArrayXd A, real_t *t) {
        int k = A.size();
        for(int i=0; i < k; i++)
            t[i] = A[i];
    }

    class MPCController {

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            MPCController();
            ~MPCController();
            void run();
            void controlLoop();
            void changeParams();

        private:
            template <class T> void getParam(std::string key, T& var);
            void trajectoryHandler(const path_follower::Trajectory2D trajectory);
            void currentstateHandler(const path_follower::state_Dynamic current_state);
            void steeringHandler(const dbw_mkz_msgs::SteeringReport msg);
            void getDesiredStates();
            void getDesiredStates_NearestP();

            void mpc(const Eigen::Matrix<double, NUM_STATES, 1> &s0, const Eigen::Matrix<double, NUM_CONTROLS, Eigen::Dynamic> &u0, const Eigen::Matrix<double,
             NUM_STATES, Eigen::Dynamic> &s_star, const Eigen::Matrix<double, NUM_CONTROLS, 1> &u_prev1, const Eigen::Matrix<double, NUM_CONTROLS, 1> &u_prev2, Eigen::Matrix<double, NUM_CONTROLS, Eigen::Dynamic> *u_out);

            void dynamics(const Eigen::Matrix<double, NUM_STATES, 1> &s, const Eigen::Matrix<double, NUM_CONTROLS, 1> &u, Eigen::Matrix<double, NUM_STATES, 1> *s_dot,
                Eigen::Matrix<double, NUM_STATES, NUM_STATES> *A = 0, Eigen::Matrix<double, NUM_STATES, NUM_CONTROLS> *B = 0);

            void simulateEuler(const Eigen::Matrix<double, NUM_STATES, 1> &s, const Eigen::Matrix<double, NUM_CONTROLS, 1> &u,
                Eigen::Matrix<double, NUM_STATES, 1> *s_next, Eigen::Matrix<double, NUM_STATES, NUM_STATES> *A = 0, Eigen::Matrix<double, NUM_STATES, NUM_CONTROLS> *B = 0);
           
            void simulateRK4(const Eigen::Matrix<double, NUM_STATES, 1> &s, const Eigen::Matrix<double, NUM_CONTROLS, 1> &u,
                Eigen::Matrix<double, NUM_STATES, 1> *s_next, Eigen::Matrix<double, NUM_STATES, NUM_STATES> *A = 0, Eigen::Matrix<double, NUM_STATES, NUM_CONTROLS> *B = 0);

            double getdistance(path_follower::TrajectoryPoint2D Traj,  Eigen::Matrix<double, NUM_STATES, 1> State);

        private:
            ros::NodeHandle nh_;
            ros::Subscriber current_state_sub_, trajectory_sub_, steering_sub_;
            ros::Publisher steering_pub_, twist_pub_, applanix_pub_, time_pub_;
            path_follower::ApplanixPose applanix_;  
            path_follower::Trajectory2D traj_;
            path_follower::SteeringCmd steering_cmd_;
            geometry_msgs::TwistStamped twist_;
            std_msgs::Float64 time_;

            bool received_applanix_state_;
            bool received_trajectory_;
            bool start_time_flag_, mpc_flag;
            int mpc_pub_flag;
            double steering_current_;
            double start_time, start_time_mpc, end_time_mpc;
            vehicle_state p_vs;

            Eigen::Matrix<double, NUM_STATES, 1> state_;
            Eigen::Matrix<double, NUM_CONTROLS, Eigen::Dynamic> controls_;
            Eigen::Matrix<double, NUM_STATES, Eigen::Dynamic> des_states_;
            Eigen::Matrix<double, NUM_STATES, 1> s_close;
            Eigen::Matrix<double, NUM_CONTROLS, 1> u_prev2;
            Eigen::Matrix<double, NUM_CONTROLS, 1> u_prev1;

            Eigen::Matrix<double, NUM_STATES, NUM_STATES> p_Q;
            Eigen::Matrix<double, NUM_CONTROLS, NUM_CONTROLS> p_R;
            Eigen::Matrix<double, NUM_CONTROLS, NUM_CONTROLS> p_R_delta;   
    };

}// namespace vlr
#endif 
