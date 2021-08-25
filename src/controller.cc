#include "sparkle/controller.hh"
double min(double a, double b)
{
	if (a >= b) { return b; }
	else { return a; }
}

double max(double a, double b)
{
	if (a >= b) { return a; }
	else { return b; }
}

BaseController::BaseController(ros::NodeHandle *nh): nh(nh)
{
	nh->param("ego_vel_topic", ego_vel_topic, std::string("vel"));
	nh->param("relative_vel_topic", relative_vel_topic, std::string("rel_vel"));
	nh->param("leader_vel_topic", leader_vel_topic, std::string("leader_vel"));
	nh->param("headway_topic", headway_topic, std::string("lead_dist"));
	nh->param("use_leadervel", use_leadervel, false);
	nh->param("use_odom", use_odom, false);
	nh->param("publish_cmdaccel", publish_cmdaccel, false);
	nh->param("publish_cmdvel", publish_cmdvel, false);
	nh->param("ego_odom", ego_odom_topic, std::string("ego_odom"));
	nh->param("leader_odom", leader_odom_topic, std::string("leader_odom"));
}

MicromodelController::MicromodelController(ros::NodeHandle *nh): BaseController(nh)
{
	time_avg_target = std::vector<double>(1280);
	nh->param("th1", h1, 0.4); nh->param("th2", h2, 0.8); nh->param("th3", h2, 1.5);	
	nh->param("w1", w1, 4.5);  nh->param("w2", w2, 5.25); nh->param("w3", w3, 6.5);	
	nh->param("d1", d1, 0.5);  nh->param("d2", d2, 1.0);  nh->param("d3", d3, 1.5);	

	max_accel = 1.5; max_decel = -3.0;
	v_fast = 30.0; historical_value = 6.0;
	params_1 = 2.0; params_3 = 5.0; params_3_1 = 1.3; params_1_1 = 1.1;
	reduction = 0.8;
	increase = 0.01;
	weight_1 = 1.0/3.0; weight_2 = 1.0/3.0; weight_3 = 1.0/3.0;
	h0=2.5;
	s0=2.0;

	nh->param("gap_variant", gap_variant, true);
	historical = 0;

	time_avg_target = {0};
	t_length = 0;

	fast_flag = true; adjust_flag = true; first_flag = false;

	old_cmdvel = new_cmdvel = 0.0;
	old_time = ros::Time::now();

	cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
	cmd_accel_pub = nh->advertise<geometry_msgs::Twist>("cmd_accel",1);
	sub_vel = nh->subscribe(ego_vel_topic, 1, &MicromodelController::callback_vel, this);
	sub_leadervel = nh->subscribe(leader_vel_topic, 1, &MicromodelController::callback_leadervel, this);
	sub_relative_vel = nh->subscribe(relative_vel_topic, 1, &MicromodelController::callback_relative_vel, this);
	sub_headway = nh->subscribe(headway_topic, 1, &MicromodelController::callback_headway, this);
	sub_ego_odom = nh->subscribe(ego_odom_topic, 1, &MicromodelController::callback_ego_odom, this);
	sub_leader_odom = nh->subscribe(leader_odom_topic, 1, &MicromodelController::callback_leader_odom, this);
	
	geometry_msgs::Twist temp_vel;
	temp_vel.linear.x = 0.0; temp_vel.linear.y = 0.0; temp_vel.linear.z = 0.0;
	state_vel = temp_vel;
	state_leadervel = temp_vel;
	state_relative_vel = temp_vel;

}

void MicromodelController::callback_ego_odom(const nav_msgs::Odometry& o_msg) { state_ego_odom = o_msg; }
void MicromodelController::callback_leader_odom(const nav_msgs::Odometry& o_msg) { state_leader_odom = o_msg; }
void MicromodelController::callback_vel(const geometry_msgs::Twist& v_msg) { state_vel = v_msg; }
void MicromodelController::callback_leadervel(const geometry_msgs::Twist& lv_msg) { state_leadervel = lv_msg; }
void MicromodelController::callback_relative_vel(const geometry_msgs::Twist& rv_msg) { state_relative_vel = rv_msg; }
void MicromodelController::callback_headway(const std_msgs::Float64& h_msg) { state_headway = h_msg; }

void MicromodelController::control_algorithm()
{
	new_time = ros::Time::now();
	ros::Duration diff = new_time - old_time;
	old_time = new_time;
		
	double elapsedTime = diff.toSec();
        double nanoSecs = diff.toNSec()*1e-9;

        elapsedTime = elapsedTime + nanoSecs;
	

	double egovel = sqrt(((state_vel.linear.x)*(state_vel.linear.x)) + ((state_vel.linear.y)*(state_vel.linear.y)));
	if(!first_flag)
        {
		egovel = 0.0;
	}
	ROS_INFO_STREAM("True velocity of the ego is "<<egovel);
	double relvel = 0.0;
	double headway = 0.0;
	

	double leadervel = 0.0;
	
	if(use_leadervel)
	{	
		leadervel= sqrt(((state_leadervel.linear.x)*(state_leadervel.linear.x)) + ((state_leadervel.linear.y)*(state_leadervel.linear.y)));
		ROS_INFO_STREAM("True leader Velocity is "<<leadervel);
	}
	else
	{
		relvel = state_relative_vel.linear.z;
		leadervel = relvel + egovel;
		ROS_INFO_STREAM("Estimated leader Velocity is "<<leadervel);
	}
	
	double sum_weights = 0.0;
	for (int i = time_avg_target.size() - 2; i >=0; --i)
	{
		time_avg_target.at(i+1) = time_avg_target.at(i);
	}
	time_avg_target.at(0) = leadervel;
	if(t_length < 1280)
	{
		t_length = t_length + 1; 
	}
	
	if(use_odom)
	{
		ROS_INFO_STREAM("We are going to use true odomtry for relative velocity and space gapp");
		relvel = leadervel - egovel;

		ROS_INFO_STREAM("Relative velocity is " << relvel);
		double ego_posX = state_ego_odom.pose.pose.position.x;
		double ego_posY = state_ego_odom.pose.pose.position.y;
		double leader_posX = state_leader_odom.pose.pose.position.x;
		double leader_posY = state_leader_odom.pose.pose.position.y;
		headway =  sqrt( (ego_posX - leader_posX )*( ego_posX - leader_posX ) +  (ego_posY - leader_posY )*( ego_posY - leader_posY ) ) - 4.5;// subtract by the length of the car.
		ROS_INFO_STREAM("Space Gap " <<  headway);
	}
	else
	{
		headway = state_headway.data;
		relvel = state_relative_vel.linear.x;
	}

	double dx = headway;
	ROS_INFO_STREAM("dx = " << dx);
	if (historical == 1)
	{
		ROS_INFO_STREAM("Historical = 1");
		v_des = historical_value*0.8;
		if (gap_variant == 1)
		{
			ROS_INFO_STREAM("Gap Variant = 1");
			sum_weights = weight_1 + weight_2 + weight_3;
			v_des = (weight_1*(min((1.0 / h0) * max(dx - s0, 0.0), v_fast)) + weight_2 * leadervel+ weight_3 * v_des) / sum_weights;
			ROS_INFO_STREAM("V des is " << v_des);
		}
	}
	else
	{
		ROS_INFO_STREAM("Historical = 0");
		double sum = 0.0;
		for (int i = 0; i < time_avg_target.size() ; ++i)
		{
			sum = sum + time_avg_target.at(i);
		}
		v_des = sum/t_length;
		if (gap_variant == 1)
		{
			sum_weights = weight_1 + weight_2 + weight_3;
			v_des = (weight_1*(min((1 / h0) * max(dx - s0, 0), v_fast)) + weight_2 * leadervel+ weight_3 * v_des) / sum_weights;
			ROS_INFO_STREAM("V des is " << v_des);
		}
	}

	if(!first_flag)
	{
		v_des2 = v_des;
		first_flag = true;
	}

	if((fast_flag == false) && (adjust_flag == false))
	{
		if (v_des2*reduction >= v_des*reduction)
		{
			v_des2 = v_des*reduction;
		}
		else
		{
			v_des2 = v_des2*reduction;
		}
		adjust_flag = true;		
	}
	else
	{
		if(v_des2* (1 + increase*elapsedTime) >= v_des)
		{
			v_des2 = v_des;	
		}
		else
		{
			v_des2 = v_des2* (1 + increase*elapsedTime);
		}
	}
	ROS_INFO_STREAM("vdes2 " << v_des2);	
	// Filter for free flow
	double dv_minus = min(relvel, 0.0);
	double v = min(max(leadervel, 0.0), v_des);
	ROS_INFO_STREAM("V = " << v);
	double dx_1 = 1.0 / (2.0 * d1) * (dv_minus*dv_minus) + max(w1, h1 * egovel);
	double dx_2 = 1.0 / (2.0 * d2) * (dv_minus*dv_minus) + max(w2, h2 * egovel);
	double dx_3 = 1.0 / (2.0 * d3) * (dv_minus*dv_minus) + max(w3, h3 * egovel);
	double dx_5 = max(w3 * params_3, params_3_1 * dx_3);
	double dx_4 = max(w3 * params_1, params_1_1 * dx_3);

	old_cmdvel = new_cmdvel;

	ROS_INFO_STREAM("dx_1 " << dx_1);
	ROS_INFO_STREAM("dx_2 " << dx_2);
	ROS_INFO_STREAM("dx_3 " << dx_3);
	ROS_INFO_STREAM("dx_4 " << dx_4);
	ROS_INFO_STREAM("dx_5 " << dx_5);

	if (dx >= dx_5)
	{
		ROS_INFO_STREAM("Region 5");
		new_cmdvel = min(v_des * (1.0 + (dx - dx_5) / (dx_5)), v_fast);
		ROS_INFO_STREAM("New Commanded Velocity is " << new_cmdvel);	
	}
	else if(dx >= dx_4)
	{
		ROS_INFO_STREAM("Region 4");
		new_cmdvel = v_des + min(0.0, (v_des2 - v_des) * (dx_5 - dx) / (dx_5 - dx_4));
		ROS_INFO_STREAM("New Commanded Velocity is " << new_cmdvel);	
	}
	else if(dx >= dx_3)
	{
		ROS_INFO_STREAM("Region 3");
		new_cmdvel = v_des2;
		ROS_INFO_STREAM("New Commanded Velocity is " << new_cmdvel);	
	}
	else if(dx >= dx_2)
	{
		ROS_INFO_STREAM("Region 2");	
		new_cmdvel = v + (v_des2 - v) * (dx - dx_2) / (dx_3 - dx_2);
		ROS_INFO_STREAM("New Commanded Velocity is " << new_cmdvel);	
	}
	else if(dx >= dx_1)
	{
		ROS_INFO_STREAM("Region 1");
		new_cmdvel = v * (dx - dx_1) / (dx_2 - dx_1);
		fast_flag = 1;
		adjust_flag = 0;
		ROS_INFO_STREAM("New Commanded Velocity is " << new_cmdvel);	
	}
	else
	{
		ROS_INFO_STREAM("Region 1");
		new_cmdvel = 0.0;
		fast_flag = 0;
		ROS_INFO_STREAM("New Commanded Velocity is " << new_cmdvel);	
	}
	
		
	new_cmdaccel = (new_cmdvel - old_cmdvel)/elapsedTime;
}

void MicromodelController::publish()
{
	MicromodelController::control_algorithm();
	if (publish_cmdvel)
	{
		geometry_msgs::Twist v;
		v.linear.x = new_cmdvel;
		cmd_vel_pub.publish(v);
	}
	if(publish_cmdaccel)
	{
		geometry_msgs::Twist a;
		a.linear.x = new_cmdaccel;
		cmd_accel_pub.publish(a);
	}
}
