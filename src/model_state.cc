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

#include "mymath.cc"
using namespace std;


class SetModelPose
{
    private:
        ros::NodeHandle* rosnode; //Create ROS Node Handle
        ros::Subscriber sub; // Subscriber for subscribing to topic that produces Odometry information to be set.
        ros::Publisher pub; // This will always be /gazebo/set_model_state
        
        // Check if subscriber has been created
        bool isSubscribed;
        
        // False if no message has receive, true if any message on `sub` has
        // been received, until `pub` publishes, and then it becomes false again
        bool newMessage;
        
        // A flag to enable sending twist along with poses.
        bool enableTwist;

        bool enablePos;
        // Get the namespace (value in the group tag)
        string ns;
        
        // Get the node name
        string nodename;
        
        // Variables to store messages
        geometry_msgs::Point pose;
        geometry_msgs::Quaternion orientation;
        geometry_msgs::Twist twist;
        gazebo_msgs::ModelState modelState;

    public:
        SetModelPose()
        {
            this->rosnode = new ros::NodeHandle(""); 
            this->newMessage = false;
            this->enableTwist = false;
            this->enablePos = false;
            this->isSubscribed = false;
            
            this->ns = ros::this_node::getNamespace();
            this->ns = this->ns.substr(1, this->ns.length()-1);
            
            this->nodename = ros::this_node::getName();
            
            this->pub = this->rosnode->advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);          
            // Initialize the message variable
            this->pose.x = this->pose.y = this->pose.z = 0.0;
            this->pose.z = 0.006510;
            this->orientation.x = this->orientation.y =  this->orientation.z = this->orientation.w = 0.0;
            memset(&(this->modelState), 0, sizeof(this->modelState));
            memset(&(this->modelState), 0, sizeof(this->modelState));
        }
        
        
        void createSubscriber(string odomVelTopic = "setvel")
        {
            this->sub = this->rosnode->subscribe(odomVelTopic, 1, &SetModelPose::setVelCallBack, this);
            this->isSubscribed = true;
        }
        

    public:
    
        bool subscribed()
        {
            return this->isSubscribed;
        }
        void setVelCallBack(const nav_msgs::OdometryConstPtr& _msg)
        {
            this->pose.x = _msg->pose.pose.position.x;
            this->pose.y = _msg->pose.pose.position.y;
            this->pose.z = _msg->pose.pose.position.z;
            this->orientation = _msg->pose.pose.orientation;
            this->newMessage = true;

            this->twist = _msg->twist.twist;
            

            this->modelState.model_name = this->ns;
            
            this->rosnode->param(this->nodename +"/"+"enablePos", this->enablePos, true);

            if (this->enablePos)
            {
                this->modelState.pose.position = this->pose ;//getModelState.response.pose.position;
                this->modelState.pose.orientation = this->orientation; //getModelState.response.pose.orientation;
            }
            this->rosnode->param(this->nodename +"/"+"enableTwist", this->enableTwist, false);
            
            
            if(this->enableTwist)
            {
                this->modelState.twist = this->twist;
            }
            this->modelState.reference_frame = "world";
        }
        
    public:
        void publishModelState(unsigned int rate)
        {
            ros::Rate loop_rate(rate);
            while( ros::ok() )
            {
                if( this->newMessage )
                {
                    this->pub.publish(this->modelState);
                    this->newMessage = false;
                }
                ros::spinOnce( );
                loop_rate.sleep( );
            }
        }
};


int main(int argc, char **argv)
{

    // Before you do anything, do the ros init
    ros::init(argc, argv, "SparkleSetPose");
    
    std::string enableTwist;
    std::string enablePos;
    

    //Create an object of type `SetModelPose`
    SetModelPose *sparkleModel = new SetModelPose();
    
    
    sparkleModel->createSubscriber();
    
    if (sparkleModel->subscribed())
    {
        sparkleModel->publishModelState(100);
    }
    return EXIT_SUCCESS;
    
}
