// Author: Jonathan Sprinkle
// This (very simple) node reads a laser scan, and 
// publishes the distance to the nearest point
//
// TODO: ensure nearest few points are somewhat close
// TODO: what to return if no closest points?
// TODO: enable angle range we care about
// TODO: enable distance range we care about

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include <cassert>

#include <cstdio>
#include <cstdlib>

/*
   Updated version to account for a window size that varies
   based on the latest discontinuity in the data.
   */


// this global var holds the distance
std_msgs::Float64 angle;
std_msgs::Float64 dist;
std_msgs::Float64 dist_1;
geometry_msgs::Twist vel;

// memory for bad data
std_msgs::Float64 vel_lastGood;
std_msgs::Float64 dist_lastGood;
double weightedVel;

// memory array for filtered distance
typedef struct distanceInfo_tag {
	ros::Time stamp;
	double distance;
} distanceInfo;

std_msgs::Float64 accel;
ros::Time lastUpdate;
ros::Time startUp;
bool newMessage;
double angle_min;
double angle_max;

// This very simple callback looks through the data array, and then
// returns the value (not the index) of that distance
void distCallback( const std_msgs::Float64::ConstPtr& distance )
{

	dist.data = distance->data;
	ros::Time nowtime = ros::Time::now();

	newMessage = true;

	ros::Duration diff = nowtime - lastUpdate;
	
	double elapsedTime = diff.toSec();

	double nanoSecs = diff.toNSec()*1e-9;
	elapsedTime = elapsedTime + nanoSecs;
	lastUpdate = nowtime;

	vel.linear.z = (dist.data - dist_1.data)/nanoSecs;
	dist_1.data = dist.data;	
}

int main( int argc, char **argv )
{
	// initialize global vars
	dist.data = dist_1.data = dist_lastGood.data = 0.0;
	vel.linear.z = 0.0;
	accel.data = 0.0;
	weightedVel = 0.0;

	std::string dist_topic;
	//    std::string angle_topic;
	std::string vel_topic;

	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "relvel");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	//	ros::NodeHandle n;
	// set up the handle for this node, in order to publish information
	// the node handle is retrieved from the parameters in the .launch file,
	// but we have to initialize the handle to have the name in that file.
	ros::NodeHandle n("~");

	/**
	 * The advertise() function is how you tell ROS that you want to
	 * publish on a given topic name. This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing. After this advertise() call is made, the master
	 * node will notify anyone who is trying to subscribe to this topic name,
	 * and they will in turn negotiate a peer-to-peer connection with this
	 * node.  advertise() returns a Publisher object which allows you to
	 * publish messages on that topic through a call to publish().  Once
	 * all copies of the returned Publisher object are destroyed, the topic
	 * will be automatically unadvertised.
	 *
	 * The second parameter to advertise() is the size of the message queue
	 * used for publishing messages.  If messages are published more quickly
	 * than we can send them, the number here specifies how many messages to
	 * buffer up before throwing some away.
	 */
	n.param("dist_topic", dist_topic, std::string("dist"));
	n.param("vel_topic", vel_topic, std::string("relvel"));

	ROS_INFO_STREAM("Node namespace is " << ros::this_node::getNamespace());
	ROS_INFO_STREAM("Node name is " << ros::this_node::getName( ) );


	ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>(vel_topic, 1);

	// we also want to subscribe to the signaller
	ros::Subscriber sub1 = n.subscribe(dist_topic, 1, &distCallback);

	ROS_INFO_STREAM("Looking for dist in topic " << dist_topic);
	ROS_INFO_STREAM("Publishing estimated velocity as " << ros::this_node::getName( ) << "/" << vel_topic);

	startUp = ros::Time::now();
	ROS_INFO_STREAM("Start up time is " << startUp );

	ros::Rate loop_rate(20);
	lastUpdate = ros::Time();
	newMessage = false;

	while( ros::ok() )
	{
		if( newMessage )
		{
			vel_pub.publish(vel);
			newMessage = false;
		}
		ros::spinOnce( );
		loop_rate.sleep( );
	}

	return EXIT_SUCCESS;
}

