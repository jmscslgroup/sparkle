
/* ----- Copyright Rahul Bhadani
 * Maintainer Email: rahulbhadani@email.arizona.edu
 * ----------*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

class BaseController
{
	protected:
		ros::NodeHandle *nh;
		ros::Publisher cmd_vel_pub;
		ros::Publisher cmd_accel_pub;
		std::string ego_vel_topic;
		std::string headway_topic;
		std::string relative_vel_topic;
		std::string leader_vel_topic;
		std::string ego_odom_topic;
		std::string leader_odom_topic;
		bool use_odom; // if true, odometry data is used for vehicle's position 
		bool use_leadervel; // flag to decide whether to to use leader vehicle directly from the leader_vel_topic or use estimate based on relative velocity + ego velocity
		bool publish_cmdaccel; // flag to decide whether to publish commanded acceleration
		bool publish_cmdvel; // flag to decide whether to publish commanded velocity
		
		ros::Subscriber sub_vel, sub_leadervel, sub_headway, sub_relative_vel, sub_ego_odom, sub_leader_odom;
  		geometry_msgs::Twist state_vel, state_leadervel, state_relative_vel;
  		std_msgs::Float64 state_headway;
		nav_msgs::Odometry state_leader_odom;
		nav_msgs::Odometry state_ego_odom;


	public:
		BaseController(ros::NodeHandle *nh);

};


class MicromodelController: BaseController
{
	protected:
		double h1, h2, h3;
		double w1, w2, w3;
		double d1, d2, d3;
		double max_accel, max_decel;
		double v_fast;
		double historical_value;
		double params_1, params_1_1, params_3, params_3_1;
		double reduction;
		double increase;
		double weight_1, weight_2, weight_3;
		double h0;
		double s0;

		bool gap_variant;
		double historical;
		
		double old_cmdvel;
		double new_cmdvel;
		double new_cmdaccel;

		ros::Time old_time;
		ros::Time new_time;
		bool newMessage;

		double v_des, v_des2;
		bool fast_flag; // d1 in Amaury's implementation
		bool adjust_flag; // d2 in Amaury's implementation
		
		bool first_flag; //flag to denote if it is the first message
		std::vector<double> time_avg_target;
		double t_length;

	public:
		MicromodelController(ros::NodeHandle *nh);
		void callback_vel(const geometry_msgs::Twist& v_msg);
		void callback_leadervel(const geometry_msgs::Twist& lv_msg);
		void callback_relative_vel(const geometry_msgs::Twist& rv_msg);
		void callback_headway(const std_msgs::Float64& h_msg);
		void callback_ego_odom(const nav_msgs::Odometry& o_msg);
		void callback_leader_odom(const nav_msgs::Odometry& o_msg);

		void publish();
		void control_algorithm();
};

#endif // CONTROLLER_H
