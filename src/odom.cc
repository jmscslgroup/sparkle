/*---------------------------------------------------
Author: Rahul Bhadani
Date: July 2019

This class implements a Gazebo plugin that publishes 
odometry information for Sparkle car model

---------------------------------------------------*/

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <cstdio>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <cstdlib>
#include <stdlib.h>
#include <pwd.h>
#include <stdio.h>
#include <sys/types.h>
#include<iostream>
#include<string>
#include<vector>
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

using namespace std;

namespace gazebo
{
    class SparkleOdom: public ModelPlugin
    {
        private: 
            //to read the name space from urdf file
            std::string robotNamespace;

            std::string tfScope;
            // Name of the odometry topic being published
            std::string odomTopic;
            
            ros::Publisher odom_pub;
            ros::Subscriber sub;
            boost::thread ros_spinner_thread_;
            ros::NodeHandle* rosnode_;
            nav_msgs::Odometry odom;
            //Gazebo
            event::ConnectionPtr updateConnection;
            //Pointer to the model entity
            physics::ModelPtr model;
            //Pointer to the world in which the model exists
            physics::WorldPtr world;
            ros::Time current_time;
    
        public:
            SparkleOdom()
            {
                this->robotNamespace = "";
                this->tfScope = "";
                this->odomTopic = "";
            
            }
            //Destructor
            ~SparkleOdom()
            {
                //Kill the boost thread
                pthread_kill(this->ros_spinner_thread_.native_handle(), 9);
                
                this->rosnode_->shutdown();
                delete this->rosnode_;

            }
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
            {
                this->model = _parent;
                this->world = _parent->GetWorld();
                this->robotNamespace = "";
                
                if(_sdf->HasElement("robotNamespace"))
                {
                    this->robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
                }
                ROS_INFO_STREAM("Robot loaded is "<<this->robotNamespace);
                this->odomTopic = this->robotNamespace + "/odom";
                this->tfScope = this->robotNamespace.substr(1,this->robotNamespace.size()-1);
                
                //Start up ros_node
                int argc = 0;
                char** argv = NULL;
                ros::init(argc, argv, "SparkleOdom", ros::init_options::NoSigintHandler |  ros::init_options::AnonymousName);
                rosnode_ = new ros::NodeHandle(this->robotNamespace);

                odom_pub = rosnode_->advertise<nav_msgs::Odometry>(odomTopic, 1);
                
                this->ros_spinner_thread_ = boost::thread( boost::bind( &SparkleOdom::OdomThread, this ) );
            }
            private:
                void OdomThread()
                {
                    ROS_INFO_STREAM("Sparkle Odom Publisher: Callback thread id=" << boost::this_thread::get_id());
                    this->sub = rosnode_->subscribe("/gazebo/model_states",1, &SparkleOdom::getOdom, this);

                    ROS_INFO_STREAM("Subscriber has topic " << this->sub.getTopic() ); 

                    ros::Rate loop_rate(100);
                    while (this->rosnode_->ok())
                    {
                        ros::spinOnce();
                        loop_rate.sleep( );
                    }
                }
                void getOdom(const gazebo_msgs::ModelStates::ConstPtr& msg)
                {
                    this->current_time = ros::Time::now();
                    vector<string> modelNames = msg->name;
                    
                   // ROS_INFO_STREAM("Size: "<< modelNames.size() );
                    int index=-1;

                    // figure out which index we are in the msg list of model states
                    for( int i=0; i<modelNames.size() && index<0; i++ )
                    {
                        if( this->robotNamespace == std::string("/"+modelNames[i]) )
                        {
                            index = i;
                            break;
                        }
                    }
                    if( index == -1 )
                    {
                        ROS_ERROR_STREAM("Unable to find odometry for model name " << this->robotNamespace << "=" <<       index);
                    }
                    else
                    {                 
                        // grab the odometry from the incoming msg and post it
                        this->odom.header.stamp = this->current_time;
                        this->odom.header.frame_id = this->tfScope + "/odom";
                        // we calculate our index in the pose msg by 
                        this->odom.child_frame_id = this->tfScope + "/base_link";
                        this->odom.pose.pose = msg->pose[index];
                        this->odom.twist.twist = msg->twist[index];                
                        this->odom_pub.publish(this->odom);
                    }
                }
    };
    GZ_REGISTER_MODEL_PLUGIN(SparkleOdom)
    
}
