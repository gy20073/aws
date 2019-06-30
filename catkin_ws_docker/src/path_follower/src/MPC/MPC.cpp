/********************************************************
  Autonomous Driving Software
  Copyright (c) 2017 MSC Lab, UC Berkeley 
  All rights reserved.

 ********************************************************/

#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
#include "path_follower/MPC.h"

#pragma GCC push_options
#pragma GCC optimize("O2")

using namespace std;
using namespace Eigen;

vlr::MPCController* mpc=NULL;

int main(int argc, char** argv) {
    ros::init(argc, argv, "MPC");  
    try {
        mpc = new vlr::MPCController;
    }
    catch (vlr::Ex<>& e) {
        std::cout << e.what() << std::endl;
    }
    try {
        mpc->run();
    }
    catch (vlr::Ex<>& e) {
        std::cout << e.what() << std::endl;
    }
    delete mpc;
    return 0;
}

namespace vlr {
    MPCController::MPCController() : nh_(), received_applanix_state_(false), received_trajectory_(false), 
            start_time_flag_(false), mpc_flag(true), mpc_pub_flag(0) {
        p_Q.setZero();
        p_R.setZero();
        p_R_delta.setZero();
        p_vs.set_mkz_params();
        changeParams();
        des_states_.resize(NUM_STATES, p_horizon + 1);
        controls_.resize(NUM_CONTROLS, p_horizon);
        controls_.setZero();
        current_state_sub_ = nh_.subscribe("state_estimate", 1, &MPCController::currentstateHandler, this);
        trajectory_sub_ = nh_.subscribe("final_trajectory", 1, &MPCController::trajectoryHandler, this);
        steering_sub_ = nh_.subscribe("/vehicle/steering_report", 1, &MPCController::steeringHandler, this);
        steering_pub_ = nh_.advertise<path_follower::SteeringCmd>("path_follower/steering_cmd", 1);
        twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("twist_cmd", 1);
        applanix_pub_ = nh_.advertise<path_follower::ApplanixPose>("Applanix", 1);
        time_pub_ = nh_.advertise<std_msgs::Float64>("Time", 1);
     }

    MPCController::~MPCController() {}

    void MPCController::run() {
        ros::Rate r(ros_hertz);
        while (ros::ok()) {
            controlLoop();
            ros::spinOnce();
            r.sleep();
        }
    }

    void MPCController::changeParams() {
        p_Q(0, 0) = p_q_lon;
        p_Q(1, 1) = p_q_lat;
        p_Q(2, 2) = p_q_theta;
        p_Q(3, 3) = p_q_u;
        p_Q(4, 4) = p_q_v;
        p_Q(5, 5) = p_q_theta_dot;
        p_R(0, 0) = p_r_udot;
        p_R(1, 1) = p_r_delta;
        p_R_delta(0, 0) = p_rd_udot;
        p_R_delta(1, 1) = p_rd_delta;
    }

    void MPCController::currentstateHandler(const path_follower::state_Dynamic current_state) {
        applanix_.vel_north = current_state.vx;
        applanix_.vel_east  = current_state.vy;
        applanix_.smooth_x  = current_state.X;
        applanix_.smooth_y  = current_state.Y;
        applanix_.yaw       = current_state.psi; 
        applanix_.rate_yaw  = current_state.wz;
        applanix_.timestamp = Time::current();
        applanix_pub_.publish(applanix_);
        received_applanix_state_ = true; 
        if (!start_time_flag_ && received_applanix_state_) {
            start_time = Time::current(); 
            start_time_flag_ = true;
        }

        state_(0) = applanix_.smooth_x + cos(applanix_.yaw) * p_vs.param.b;
        state_(1) = applanix_.smooth_y + sin(applanix_.yaw) * p_vs.param.b;  
        state_(2) = applanix_.yaw;
        state_(5) = applanix_.rate_yaw;
        state_(3) = applanix_.vel_north;
        state_(4) = applanix_.vel_east + state_(5) * p_vs.param.b;
    }

    void MPCController::steeringHandler(const dbw_mkz_msgs::SteeringReport msg) {
        steering_current_ = msg.steering_wheel_angle;
    }

    void MPCController::trajectoryHandler(const path_follower::Trajectory2D trajectory) {
        traj_ = trajectory;
        received_trajectory_ = true;
    }

    void MPCController::controlLoop() {

        if (!received_trajectory_) 
            return; 
        received_trajectory_ = false;  
        if (!received_applanix_state_) 
            return;
        received_applanix_state_ = false;
        if (traj_.header.stamp.toSec() < applanix_.timestamp - 10.0 ) {
            controls_.setZero();
            cout << "No commands in a while, resetting controller..." << endl;
            return;
        }
        if (nearest_flag)
            getDesiredStates_NearestP();
        else 
            getDesiredStates();
        
        controls_.block(0, 0, NUM_CONTROLS, p_horizon - i_horizon) = controls_.block(0, i_horizon, NUM_CONTROLS, p_horizon - i_horizon);  
        if (mpc_flag) {
            if (mpc_pub_flag > 0) { 
                if (mpc_pub_flag > 1)
                    u_prev2 = u_prev1;
                u_prev1 = controls_.col(0);
            }
            else {
                u_prev1.setZero();
                u_prev2.setZero();
            }

            mpc(state_, controls_, des_states_, u_prev1, u_prev2, &controls_);

            mpc_pub_flag++; 
    	    for (int i = 0; i < i_horizon; i++) {
            	double steering_angle = controls_(1, i) * p_vs.param.steering_ratio;
            	steering_cmd_.enable = true;
            	steering_cmd_.steering_wheel_angle_cmd = steering_angle;
            	steering_cmd_.steering_wheel_angle_velocity = (steering_angle - steering_current_) * p_hertz;
           		steering_pub_.publish(steering_cmd_);
            	twist_.twist.linear.x = des_states_(3, i);
            	twist_pub_.publish(twist_);
    		    if (i < i_horizon - 1)
            	    sleep(dt);
	        }
	    }
        else {
            double steering_angle = 0;
            steering_cmd_.enable = true;
            steering_cmd_.steering_wheel_angle_cmd = steering_angle;
            steering_cmd_.steering_wheel_angle_velocity = 0;
            steering_pub_.publish(steering_cmd_);
            twist_.twist.linear.x = state_(3) - dt * 2;
            twist_pub_.publish(twist_);
        }

    }

    double MPCController::getdistance(path_follower::TrajectoryPoint2D TrajPoint,  Eigen::Matrix<double, NUM_STATES, 1> State) {
        double ra_to_cg = p_vs.param.b;
        double theta = TrajPoint.theta;
        double x = TrajPoint.x + cos(theta) * ra_to_cg;
        double y = TrajPoint.y + sin(theta) * ra_to_cg;
        double dist = sqrt(pow(x-State(0),2)+ pow(y-State(1),2));
        return dist;
    }

    void MPCController::getDesiredStates() {
        double t = applanix_.timestamp, alpha;
        double ra_to_cg = p_vs.param.b;
        int j = 0;
        for (int i = 0; i < p_horizon + 1; i++) {
            if (i==0 && (traj_.point[(int)traj_.point.size()-1].t + start_time) < t)
                mpc_flag = false;
            while ((traj_.point[j + 1].t + start_time) < t && j < (int)traj_.point.size() - 2)
                j++;
            path_follower::TrajectoryPoint2D& p1 = traj_.point[j];   
            path_follower::TrajectoryPoint2D& p2 = traj_.point[j + 1];   
            while (p2.theta - p1.theta > M_PI)
                p2.theta -= 2 * M_PI;
            while (p2.theta - p1.theta < -M_PI)
                p2.theta += 2 * M_PI;
            alpha = (t - p1.t -start_time) / (p2.t - p1.t);
            if (alpha > 1) alpha = 1;
            if (alpha < 0) alpha = 0;
            des_states_(0, i) = (1 - alpha) * p1.x + alpha * p2.x;
            des_states_(1, i) = (1 - alpha) * p1.y + alpha * p2.y;
            des_states_(2, i) = (1 - alpha) * p1.theta + alpha * p2.theta;
            des_states_(0, i) += cos(des_states_(2, i)) * ra_to_cg;
            des_states_(1, i) += sin(des_states_(2, i)) * ra_to_cg;
            des_states_(3, i) = (1 - alpha) * p1.v + alpha * p2.v;
            des_states_(4, i) = 0.0;
            if(!kappa_flag)
                des_states_(5, i) =  (1 - alpha) * p1.kappa + alpha * p2.kappa;   
            else
                des_states_(5, i) =  des_states_(3,i)*((1 - alpha) * p1.kappa + alpha * p2.kappa);   
            t += dt;
        }
    }

    void MPCController::getDesiredStates_NearestP() {
        double t = Time::current(), alpha;
        double ra_to_cg = p_vs.param.b;
        int node = traj_.point.size()-1;
        for (int i=(int)traj_.point.size()-1; i>=0; i--) { 
            if (getdistance(traj_.point[node], state_)> getdistance(traj_.point[i], state_))
                node = i;
        }   
        start_time = Time::current();
        int j = node;
        for (int i = 0; i < p_horizon + 1; i++) {
            if (i==0 && (traj_.point[(int)traj_.point.size()-1].t - traj_.point[node].t) < (start_time-t))
                mpc_flag = false;
            while ((traj_.point[j + 1].t - traj_.point[node].t) < (t - start_time) && j < (int)traj_.point.size() - 2)
                j++;
            path_follower::TrajectoryPoint2D& p1 = traj_.point[j];       
            path_follower::TrajectoryPoint2D& p2 = traj_.point[j + 1]; 
            while (p2.theta - p1.theta > M_PI)
                p2.theta -= 2 * M_PI;
            while (p2.theta - p1.theta < -M_PI)
                p2.theta += 2 * M_PI;
            alpha = (t - p1.t -start_time+ traj_.point[node].t) / (p2.t - p1.t);
            if (alpha > 1) alpha = 1;
            if (alpha < 0) alpha = 0;
            des_states_(0, i) = (1 - alpha) * p1.x + alpha * p2.x;
            des_states_(1, i) = (1 - alpha) * p1.y + alpha * p2.y;
            des_states_(2, i) = (1 - alpha) * p1.theta + alpha * p2.theta;
            des_states_(0, i) += cos(des_states_(2, i)) * ra_to_cg;
            des_states_(1, i) += sin(des_states_(2, i)) * ra_to_cg;
            des_states_(3, i) = (1 - alpha) * p1.v + alpha * p2.v;
            des_states_(4, i) = 0.0;
            if (!kappa_flag)
                des_states_(5, i) = (1 - alpha) * p1.kappa + alpha * p2.kappa;   
            else
                des_states_(5, i) = des_states_(3, i) * ((1 - alpha) * p1.kappa + alpha * p2.kappa);
            t += dt;
        }
    }

    void MPCController::mpc( const Eigen::Matrix<double, NUM_STATES, 1> &s0, 
                                const Eigen::Matrix<double, NUM_CONTROLS, Dynamic> &u0,
                                const Eigen::Matrix<double, NUM_STATES, Dynamic> &s_star, 
                                const Eigen::Matrix<double, NUM_CONTROLS, 1> &u_prev1, 
                                const Eigen::Matrix<double, NUM_CONTROLS, 1> &u_prev2,
                                Eigen::Matrix<double, NUM_CONTROLS, Dynamic> *u_out) {
        USING_NAMESPACE_QPOASES

        Eigen::Matrix<double, NUM_STATES, Dynamic> s(NUM_STATES, p_horizon + 1);
        Eigen::Matrix<double, NUM_STATES, NUM_STATES> A[p_horizon];
        Eigen::Matrix<double, NUM_STATES, NUM_CONTROLS> B[p_horizon];
        Eigen::Matrix<double, NUM_EXT_STATES , NUM_EXT_STATES > P, Q, Ae[p_horizon], Qe[p_horizon];
        Eigen::Matrix<double, NUM_EXT_STATES, NUM_CONTROLS> Be[p_horizon];
        Eigen::Matrix<double, NUM_STATES, 1> s_next;
        Eigen::Matrix<double, NUM_CONTROLS, NUM_CONTROLS> R;
        Eigen::Matrix<double, NUM_CONTROLS, 1>lb_unit, ub_unit, lb0_unit, ub0_unit;
        Eigen::Matrix<double, NUM_EXT_STATES, 1> x0;
        Eigen::Matrix<double, NUM_EXT_STATES, 1> de[p_horizon];
        Eigen::Matrix<double, 1, NUM_CONTROLS*2> Au_unit;
        Eigen::Matrix<double, 1, 1> lb_A_unit, ub_A_unit;
        Eigen::MatrixXd Sx(NUM_EXT_STATES *(p_horizon+1), NUM_EXT_STATES);
        Eigen::MatrixXd Rbar(NUM_CONTROLS * c_horizon, NUM_CONTROLS * c_horizon);
        Eigen::MatrixXd H(NUM_CONTROLS * c_horizon, NUM_CONTROLS * c_horizon);
        Eigen::MatrixXd g(NUM_CONTROLS * c_horizon, 1);
        Eigen::MatrixXd Sx_x0(NUM_EXT_STATES*(p_horizon+1), 1);
        Eigen::MatrixXd lb(NUM_CONTROLS * c_horizon,1), ub(NUM_CONTROLS * c_horizon,1);
        Eigen::MatrixXd De(NUM_EXT_STATES * p_horizon, 1);
        Eigen::MatrixXd Xref(NUM_EXT_STATES* (p_horizon+1), 1);
        Eigen::MatrixXd Qbar(NUM_EXT_STATES * (p_horizon+1), NUM_EXT_STATES *(p_horizon+1));
        Eigen::MatrixXd Sd(NUM_EXT_STATES*(p_horizon+1), NUM_EXT_STATES*p_horizon);
        Eigen::MatrixXd Su(NUM_EXT_STATES * (p_horizon+1), NUM_CONTROLS * p_horizon);
        Eigen::MatrixXd Sutr_Qbar(NUM_EXT_STATES * (p_horizon+1), NUM_EXT_STATES *(p_horizon+1));
        Eigen::MatrixXd Sc(NUM_CONTROLS * p_horizon, NUM_CONTROLS * c_horizon);
        Eigen::MatrixXd Sctr(NUM_CONTROLS * c_horizon, NUM_CONTROLS * p_horizon);
        Eigen::MatrixXd Au(c_horizon-1, NUM_CONTROLS * c_horizon);
        Eigen::MatrixXd lb_A(c_horizon-1,1), ub_A(c_horizon-1,1);
        Eigen::MatrixXd uOpt_matrix(NUM_CONTROLS * p_horizon, 1);
      
        Sx.setZero();
        Rbar.setZero();
        Su.setZero();
        Sd.setZero();  
        Rbar.setZero();
        Qbar.setZero();
        Sc.setZero();
        Sctr.setZero();
        s.col(0) = s0;
        x0.setZero();
        x0.block(0,0,NUM_STATES, 1) = s0;
        x0.block(NUM_STATES,0,NUM_CONTROLS,1) = u_prev1;
        x0.block(NUM_STATES+NUM_CONTROLS,0,NUM_CONTROLS,1) = u_prev2;
        QProblem example(NUM_CONTROLS * c_horizon, c_horizon-1);
        Options options;
        example.setOptions( options );
        real_t uOpt[c_horizon * NUM_CONTROLS];
        int_t nWSR = 100;

        for (int i = 0; i < p_horizon; i++) {
            simulateRK4(s.col(i), u0.col(i), &s_next, &A[i], &B[i]);
            s.col(i + 1) = s_next;
        }
        Q.setZero();
        Q.block(0, 0, NUM_STATES, NUM_STATES) = p_Q;
        R = Rotation2D<double> (s_star(2, p_horizon)).toRotationMatrix();
        Q.block(0, 0, 2, 2) = R.transpose() * p_Q.block(0, 0, 2, 2) * R;
        Q.block(NUM_STATES, NUM_STATES, NUM_CONTROLS, NUM_CONTROLS) = p_R_delta;
        Q.block(NUM_STATES + NUM_CONTROLS, NUM_STATES, NUM_CONTROLS, NUM_CONTROLS) = -p_R_delta;
        Q.block(NUM_STATES, NUM_STATES + NUM_CONTROLS, NUM_CONTROLS, NUM_CONTROLS) = -p_R_delta;
        Q.block(NUM_STATES + NUM_CONTROLS, NUM_STATES + NUM_CONTROLS, NUM_CONTROLS, NUM_CONTROLS) = p_R_delta;
        P = Q;

 
        for (int i = p_horizon - 1; i >= 0; i--) {
            Qe[i] = Q;
            R = Rotation2D<double> (s_star(2, i)).toRotationMatrix();
            Qe[i].block(0, 0, 2, 2) = R.transpose() * p_Q.block(0, 0, 2, 2) * R;
            Ae[i].setZero();
            Ae[i].block(NUM_STATES + NUM_CONTROLS, NUM_STATES, NUM_CONTROLS, NUM_CONTROLS) = Eigen::Matrix<double, NUM_CONTROLS, NUM_CONTROLS>::Identity();
            Be[i].setZero();
            Be[i].block(NUM_STATES, 0, NUM_CONTROLS, NUM_CONTROLS) = Eigen::Matrix<double, NUM_CONTROLS, NUM_CONTROLS>::Identity();
            Ae[i].block(0, 0, NUM_STATES, NUM_STATES) = A[i];
            Be[i].block(0, 0, NUM_STATES, NUM_CONTROLS) = B[i];
        }
        Sx.block(0, 0, NUM_EXT_STATES, NUM_EXT_STATES) = Eigen::Matrix<double, NUM_EXT_STATES, NUM_EXT_STATES>::Identity(); 
        for (int i=0; i<p_horizon; i++)
            Sx.block(NUM_EXT_STATES * (i+1), 0, NUM_EXT_STATES, NUM_EXT_STATES) = Ae[i] * Sx.block(NUM_EXT_STATES * i, 0, NUM_EXT_STATES, NUM_EXT_STATES);
        for (int column =0; column < p_horizon; column++) {
            Su.block(NUM_EXT_STATES *(column+1), NUM_CONTROLS * column, NUM_EXT_STATES, NUM_CONTROLS) = Be[column];
            Sd.block(NUM_EXT_STATES*(column+1), NUM_EXT_STATES*column, NUM_EXT_STATES,NUM_EXT_STATES) = Eigen::Matrix<double, NUM_EXT_STATES, NUM_EXT_STATES>::Identity();
            for (int row = column +1; row < p_horizon; row++) {
                Su.block(NUM_EXT_STATES *(row+1), NUM_CONTROLS * column, NUM_EXT_STATES, NUM_CONTROLS) = Ae[row] * Su.block(NUM_EXT_STATES * row, NUM_CONTROLS* column, NUM_EXT_STATES, NUM_CONTROLS);    
                Sd.block(NUM_EXT_STATES * (row+1), NUM_EXT_STATES* column, NUM_EXT_STATES, NUM_EXT_STATES) = Ae[row] * Sd.block(NUM_EXT_STATES * row, NUM_EXT_STATES* column, NUM_EXT_STATES, NUM_EXT_STATES);
            }
        }
        for (int i=0; i<p_horizon;i++) {
            de[i].setZero();
            de[i].block(0,0,NUM_STATES,1) =  s.col(i+1) - A[i] * s.col(i) -B[i] * u0.col(i);
            De.block(NUM_EXT_STATES*i, 0,NUM_EXT_STATES,1) = de[i];
        }

        Xref.setZero();
        for (int i=0;i<(p_horizon+1);i++)
            Xref.block(NUM_EXT_STATES*i,0,NUM_STATES,1) = s_star.col(i);
        for (int i=0; i< (p_horizon+1); i++) { 
            if (i< p_horizon) {
            	if (i<c_horizon)
            		Rbar.block(NUM_CONTROLS * i, NUM_CONTROLS * i, NUM_CONTROLS, NUM_CONTROLS) = p_R;
                Qbar.block(NUM_EXT_STATES * i, NUM_EXT_STATES * i, NUM_EXT_STATES, NUM_EXT_STATES) = Qe[i];
            }
            else
                Qbar.block(NUM_EXT_STATES * i, NUM_EXT_STATES * i, NUM_EXT_STATES, NUM_EXT_STATES) = P;
        } 
        for (int i = 0; i<p_horizon; i++) {
        	if (i < c_horizon)
        		Sc.block(NUM_CONTROLS * i, NUM_CONTROLS * i, NUM_CONTROLS, NUM_CONTROLS) = Eigen::Matrix<double, NUM_CONTROLS, NUM_CONTROLS>::Identity();
			else
				Sc.block(NUM_CONTROLS * i, NUM_CONTROLS * c_horizon - 1, NUM_CONTROLS, 1) << 1, 1;
		}
		Sctr = Sc.transpose();
        Sutr_Qbar = Su.transpose() * Qbar;
      
 start_time_mpc = Time::current();

        H = 2 * (Sctr * Sutr_Qbar * Su * Sc + Rbar);
        g = 2 * Sctr * Sutr_Qbar *( Sx * x0 + Sd * De - Xref);

 

        ub_unit <<  20.0,                           // u_dot: m/s^2
                    p_vs.param.max_wheel_angle;     // wheel_angle: rad
        lb_unit <<  -20.0,                          // u_dot: m/s^2
                    -p_vs.param.max_wheel_angle;    // wheel_angle: rad
        ub0_unit << 20.0,
                    min(p_vs.param.max_wheel_angle, steering_current_/p_vs.param.steering_ratio + 10.0/(p_hertz/i_horizon));
        lb0_unit << -20.0,
                    max(-p_vs.param.max_wheel_angle, steering_current_/p_vs.param.steering_ratio - 10.0/p_hertz/i_horizon);
        ub_A_unit << 10.0/(p_hertz/i_horizon);
        lb_A_unit << -10.0/(p_hertz/i_horizon);
end_time_mpc = Time::current();
	    time_.data = end_time_mpc - start_time_mpc;
	    time_pub_.publish(time_);
        Au.setZero();

        Au_unit << 0.0, -1.0, 0.0, 1.0;
        for (int i=0; i<c_horizon; i++) {
            if (i == 0) {
                lb.block(NUM_CONTROLS * i, 0, NUM_CONTROLS,1) = lb0_unit;
                ub.block(NUM_CONTROLS * i, 0, NUM_CONTROLS,1) = ub0_unit; 
            }
            else {
                lb.block(NUM_CONTROLS * i, 0, NUM_CONTROLS, 1) = lb_unit;
                ub.block(NUM_CONTROLS * i, 0, NUM_CONTROLS, 1) = ub_unit;
            }
            if (i<c_horizon-1) {
                lb_A.block(i,0,1,1) = lb_A_unit;
                ub_A.block(i,0,1,1) = ub_A_unit;
                Au.block(i, i*NUM_CONTROLS, 1, NUM_CONTROLS*2) = Au_unit;
            }
        }


        Eigen::MatrixXd Ht = H.transpose();
        Eigen::MatrixXd Aut = Au.transpose();
        ArrayXd aH_  = Map<const ArrayXd>(Ht.data(), Ht.size());
        ArrayXd ag_  = Map<const ArrayXd>(g.data(), g.size()); 
        ArrayXd alb_ = Map<const ArrayXd>(lb.data(), lb.size());
        ArrayXd aub_ = Map<const ArrayXd>(ub.data(), ub.size());
        ArrayXd aAu_ = Map<const ArrayXd>(Aut.data(), Aut.size());
        ArrayXd alb_A_ = Map<const ArrayXd>(lb_A.data(), lb_A.size());
        ArrayXd aub_A_ = Map<const ArrayXd>(ub_A.data(), ub_A.size());
        real_t H_[aH_.size()];  
        real_t g_[ag_.size()];
        real_t lb_[alb_.size()];
        real_t ub_[aub_.size()];
        real_t Au_[aAu_.size()];
        real_t lb_A_[alb_A_.size()];
        real_t ub_A_[aub_A_.size()];
        Array2realt(aH_, H_);
        Array2realt(ag_, g_);
        Array2realt(alb_, lb_);
        Array2realt(aub_, ub_);
        Array2realt(aAu_, Au_);
        Array2realt(alb_A_, lb_A_);
        Array2realt(aub_A_, ub_A_);

        example.init( H_, g_, Au_, lb_, ub_, lb_A_, ub_A_, nWSR);
        example.getPrimalSolution( uOpt );

        s_next = s0;
        for (int i = 0; i < p_horizon; i++) {
            if (i < c_horizon)
            	u_out->col(i) << uOpt[i*2], uOpt[i*2+1];
    	    else
    		    u_out->col(i) << uOpt[(c_horizon - 1)*2], uOpt[(c_horizon - 1)*2+1];
            uOpt_matrix.block(i*2,0,2,1) = u_out->col(i);
            simulateRK4(s_next, u_out->col(i), &s_next);
            if (i < i_horizon)
        	   s_close = s_next;
	    }
    }

    void MPCController::dynamics( const Eigen::Matrix<double, NUM_STATES, 1> &s, 
                                  const Eigen::Matrix<double, NUM_CONTROLS, 1> &u_, 
                                  Eigen::Matrix<double, NUM_STATES, 1> *s_dot, 
                                  Eigen::Matrix<double, NUM_STATES, NUM_STATES> *A, 
                                  Eigen::Matrix<double, NUM_STATES, NUM_CONTROLS> *B) {
        double u = s(3), v = s(4), cos_th = cos(s(2)), sin_th = sin(s(2));
        double th_dot = s(5), u_dot = u_(0);
        double del = u_(1), tan_del = tan(u_(1)), cos_del = cos(u_(1));
        double Fyf, Fyr, Caf = p_vs.param.ftire_stiffness, 
        Car = p_vs.param.rtire_stiffness, m = p_vs.param.mass, 
        a = p_vs.param.a, b = p_vs.param.b, I = p_vs.param.iz;
        Fyf = -Caf * (atan2(v + th_dot * a, max(u, dgc::dgc_mph2ms(5))) - del);
        Fyr = -Car * atan2(v - th_dot * b, max(u, dgc::dgc_mph2ms(5)));

        (*s_dot)(0) = u * cos_th - v * sin_th;
        (*s_dot)(1) = u * sin_th + v * cos_th;
        (*s_dot)(2) = th_dot;
        (*s_dot)(3) = u_dot;
        (*s_dot)(4) = - th_dot * v + (Fyf * cos_del + Fyr) / m;
        (*s_dot)(5) = 1 / I * (a * Fyf * cos_del - b * Fyr);
        //(*s_dot)(4) = tan_del * (u_dot - th_dot * v) + (Fyf / cos_del + Fyr) / m - th_dot * u;
        //(*s_dot)(5) = m * a / I * tan_del * (u_dot - th_dot * v) + a * Fyf / (I * cos_del) - b * Fyr / I;

        if (A != 0) {
            Eigen::Matrix<double, NUM_STATES, 1> s2 = s;
            Eigen::Matrix<double, NUM_STATES, 1> s_dot1, s_dot2;
            for (int i = 0; i < NUM_STATES; i++) {
                s2(i) += EPSILON;
                dynamics(s2, u_, &s_dot1, 0, 0);
                s2(i) -= 2 * EPSILON;
                dynamics(s2, u_, &s_dot2, 0, 0);
                s2(i) += EPSILON;
                A->col(i) = (s_dot1 - s_dot2) / (2 * EPSILON);
            }
        }

        if (B != 0) {
            Eigen::Matrix<double, NUM_CONTROLS, 1> u2 = u_;
            Eigen::Matrix<double, NUM_STATES, 1> s_dot1, s_dot2;
            for (int i = 0; i < NUM_CONTROLS; i++) {
                u2(i) += EPSILON;
                dynamics(s, u2, &s_dot1, 0, 0);
                u2(i) -= 2 * EPSILON;
                dynamics(s, u2, &s_dot2, 0, 0);
                u2(i) += EPSILON;
                B->col(i) = (s_dot1 - s_dot2) / (2 * EPSILON);
            }
        }
    }

    void MPCController::simulateEuler(const Eigen::Matrix<double, NUM_STATES, 1> &s, 
                                      const Eigen::Matrix<double, NUM_CONTROLS, 1> &u, 
                                      Eigen::Matrix<double, NUM_STATES, 1> *s_next,
                                      Eigen::Matrix<double, NUM_STATES, NUM_STATES> *A, 
                                      Eigen::Matrix<double, NUM_STATES, NUM_CONTROLS> *B) {
        Eigen::Matrix<double, NUM_STATES, 1> s_dot;
        dynamics(s, u, &s_dot, A, B);
        (*s_next) = s + s_dot / p_hertz;

        if (A) {
            (*A) /= p_hertz;
            (*A) += Eigen::Matrix<double, NUM_STATES, NUM_STATES>::Identity();
        }

        if (B) (*B) /= p_hertz;
    }


    void MPCController::simulateRK4(const Eigen::Matrix<double, NUM_STATES, 1> &s, 
                                    const Eigen::Matrix<double, NUM_CONTROLS, 1> &u, 
                                    Eigen::Matrix<double, NUM_STATES, 1> *s_next,
                                    Eigen::Matrix<double, NUM_STATES, NUM_STATES> *A, 
                                    Eigen::Matrix<double, NUM_STATES, NUM_CONTROLS> *B) {
        Eigen::Matrix<double, NUM_STATES, 1> k1, k2, k3, k4;
        dynamics(s, u, &k1);
        dynamics(s + 0.5 * dt * k1, u, &k2);
        dynamics(s + 0.5 * dt * k2, u, &k3);
        dynamics(s + dt * k3, u, &k4);
        (*s_next) = s + dt * (k1 / 6.0 + k2 / 3.0 + k3 / 3.0 + k4 / 6.0);

        if (A != 0) {
            Eigen::Matrix<double, NUM_STATES, 1> s2 = s;
            Eigen::Matrix<double, NUM_STATES, 1> sn1, sn2;
            for (int i = 0; i < NUM_STATES; i++) {
                s2(i) += EPSILON;
                simulateRK4(s2, u, &sn1, 0, 0);
                s2(i) -= 2 * EPSILON;
                simulateRK4(s2, u, &sn2, 0, 0);
                s2(i) += EPSILON;
                A->col(i) = (sn1 - sn2) / (2 * EPSILON);
            }
        }

        if (B != 0) {
            Eigen::Matrix<double, NUM_CONTROLS, 1> u2 = u;
            Eigen::Matrix<double, NUM_STATES, 1> sn1, sn2;
            for (int i = 0; i < NUM_CONTROLS; i++) {
                u2(i) += EPSILON;
                simulateRK4(s, u2, &sn1, 0, 0);
                u2(i) -= 2 * EPSILON;
                simulateRK4(s, u2, &sn2, 0, 0);
                u2(i) += EPSILON;
                B->col(i) = (sn1 - sn2) / (2 * EPSILON);
            }
        }
    }

    template <class T> void MPCController::getParam(std::string key, T& var) {
        if(!nh_.getParam(key, var))
            throw VLRException("Cannot read parameter " + key + std::string("."));
    }
} //namespace vlr
#pragma pop_options
