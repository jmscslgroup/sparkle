#include<ros/ros.h>

#include<ctime>
#include<cstdio>
#include<cstdlib>
#include<stdlib.h>
#include<pwd.h>
#include<stdio.h>
#include<iostream>
#include<sys/types.h>
#include<string>
#include<vector>
#include<math.h>

#include<std_msgs/String.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Point.h>
#include<gazebo_msgs/ModelState.h>
#include<gazebo_msgs/ModelStates.h>
#include<gazebo_msgs/SetModelState.h>
#include<gazebo_msgs/GetModelState.h>
#include<ignition/math/Vector3.hh>
#include<ignition/math/Box.hh>

using namespace std;

geometry_msgs::Point pose;
geometry_msgs::Quaternion orientation;
geometry_msgs::Twist twist;
gazebo_msgs::ModelState modelState;


bool newMessage = false;

//ros::ServiceClient client;

geometry_msgs::Quaternion rpyToQuaternion(double roll, double pitch, double yaw)
{
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);


    double w = cy * cp * cr + sy * sp * sr;
    double x = cy * cp * sr - sy * sp * cr;
    double y = sy * cp * sr + cy * sp * cr;
    double z = sy * cp * cr - cy * sp * sr;

    geometry_msgs::Quaternion quat;
    quat.x = x;
    quat.y = y;
    quat.z = z;
    quat.w = w;

    return quat;
}


class PubSub
{
    private:
        unique_ptr<ros::NodeHandle> node;
        ros::ServiceClient client;
        ros::Subscriber sub;
        ros::Publisher pub;

    public:
        PubSub()
        {
            this->node.reset(new ros::NodeHandle(""));
        }

    public:
        void setVelCallBack(const nav_msgs::OdometryConstPtr & _msg);
};

void setVelCallBack(const nav_msgs::OdometryConstPtr & _msg)
{

    string ns = ros::this_node::getNamespace();
    ns = ns.substr(1, ns.length()-1);
    
    pose.x = _msg->pose.pose.position.x;
    pose.y = _msg->pose.pose.position.y;
    pose.z = _msg->pose.pose.position.z;

    orientation = rpyToQuaternion(_msg->pose.pose.orientation.x,
                                                _msg->pose.pose.orientation.y,
                                                _msg->pose.pose.orientation.z);    
    newMessage = true;


    #if 0
    gazebo_msgs::GetModelState getModelState ;

    getModelState.request.model_name = ns;
   // getModelState.request.relative_entity_name = (std::string)"world" ;
    
    string serviceName = client.getService();
    if(!client.call(getModelState))
    {
        ROS_ERROR_STREAM("Client Service "<< serviceName<<" Call Failed");
    }
    

    ROS_INFO_STREAM("GETMODEL X:" << getModelState.response.pose.position.x);
    ROS_INFO_STREAM("GETMODEL Seq:" << getModelState.response.header.seq);
    ROS_INFO_STREAM("GETMODEL success result:" << getModelState.response.success);
    ROS_INFO_STREAM("GETMODEL status message:" << getModelState.response.status_message);
    #endif

    twist = _msg->twist.twist;

    ROS_INFO_STREAM("Namespace is "<<ns);
    modelState.model_name = ns;
    modelState.pose.position = pose ;//getModelState.response.pose.position;
    modelState.pose.orientation = orientation; //getModelState.response.pose.orientation;
   // modelState.twist = twist;
    modelState.reference_frame = "world";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SparkleSetPose");
    
    ros::NodeHandle nodeHandle("");
    
 //   client = nodeHandle.serviceClient<gazebo_msgs::GetModelState>("/gazeo/get_model_state", true);

 //   ros::ServiceClient client = nodeHandle.serviceClient<gazebo_msgs::SetModelState>("/gazeo/set_model_state");

    ros::Subscriber sub = nodeHandle.subscribe("setvel", 1, &setVelCallBack);
    
    ros::Publisher pub = nodeHandle.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);


    // run at 50Hz?
    ros::Rate loop_rate(100);
    ROS_INFO_STREAM("1. New Message = "<<newMessage);

    while( ros::ok() )
    {
        ROS_INFO_STREAM("New Message = "<<newMessage);
        if( newMessage )
        {
            pub.publish(modelState);
            ROS_INFO_STREAM("Publishing");
            newMessage = false;
        }
        ros::spinOnce( );
        loop_rate.sleep( );
    }

    #if 0
    double velx  = 0.0; //Linear X velocity
    double angularz = 0.0; //Angular Z steering Angle

    ros::Publisher vel_pub = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::Rate loop_rate(60);
    
    geometry_msgs::Twist velMsg;

    while(ros::ok())
    {
        nodeHandle.param("constVel", velx, 5.0);
        nodeHandle.param("strAngle", angularz,  0.0);
        velMsg.linear.x = velx;
        velMsg.angular.z = angularz;
        
        vel_pub.publish(velMsg);
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    #endif

    return EXIT_SUCCESS;
    
}
