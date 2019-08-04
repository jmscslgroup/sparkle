#include<functional>
#include<boost/bind.hpp>

#include<gazebo/gazebo.hh>
#include<gazebo/physics/physics.hh>
#include<gazebo/common/common.hh>

#include<cstdio>
#include<cstdlib>
#include<stdlib.h>
#include<pwd.h>
#include<stdio.h>
#include<iostream>
#include<sys/types.h>
#include<string>
#include<vector>

#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<ros/subscribe_options.h>

#include<tf/transform_broadcaster.h>

#include<std_msgs/String.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<gazebo_msgs/ModelState.h>
#include<gazebo_msgs/ModelStates.h>

#include<ignition/math/Vector3.hh>
#include<ignition/math/Box.hh>

#include<thread>
using namespace std;

namespace gazebo
{
    class Sparkle: public ModelPlugin
    {
        private:
            physics::ModelPtr model; //Model Pointer
            physics::WorldPtr world; //World Pointer

            string robotNamespace; //Name of the robot
            event::ConnectionPtr updateConnection;
            

            double velx; //X Velocity to be imparted to the robot model
            double vely; //Y Velocity to be imparted to the robot model
            double v; //Absolute velocity
            double angularz; //Yaw to be imparted to the robot model
            double vAngZ; //Yaw rate
            string setVelTopic; //On velocity topic to read velocity in order to impart velocity to the model in Gazebo

            unique_ptr<ros::NodeHandle> rosNode; //A node used for ROS Transport
            ros::Subscriber rosSubVel;
            ros::SubscribeOptions soVel;
            
            ros::Subscriber rosSubAngles;
            ros::SubscribeOptions soAngles;
            
            ros::CallbackQueue rosQueue; //Callbackqueue to process messages

            thread rosQueueThread; //Thread to run rosQueue
            
            ignition::math::Box boundingBox;
            
        public:
            Sparkle()
            {
                this->robotNamespace = "";
                this->velx = 0.0;
                this->vely = 0.0;
                this->v = 0.0;
                this->vAngZ = 0.0;
                this->angularz = 0.0;
                this->setVelTopic = "setvel";

            }
        public:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
            {
                this->model = _parent; //Store the pointer the model to which plugin is attached.
                this->world = _parent->GetWorld();
                this->boundingBox = this->model->BoundingBox();
                
                ROS_INFO_STREAM("Sparkle: Bounding box of model: x is "<< this->boundingBox.XLength()<<" y is "<<this->boundingBox.YLength());
                if(_sdf->HasElement("robotNamespace"))
                {
                    this->robotNamespace = _sdf->GetElement("robotNamespace")->Get<string>();
                    ROS_INFO_STREAM("Sparkle: Robot namespace is "<<this->robotNamespace);
                }
                else
                {
                    ROS_INFO_STREAM("Sparkle: Robot Namespace not found. Setting robotnamespace to 'catvehicle'");
                }

                if(_sdf->HasElement("setVelTopic"))
                {
                    this->setVelTopic = _sdf->GetElement("setVelTopic")->Get<string>();
                    ROS_INFO_STREAM("Sparkle: setVelTopic is set to "<<this->setVelTopic);
                }
                else
                {
                    ROS_INFO_STREAM("Sparkle: setVelTopic not found. Make sure you do not prefix setVelTopic with / or robotNamespace. Setting setVelTopic to 'setvel'.");
                    this->setVelTopic  = "setvel";
                }
                
                string plugin_name = this->robotNamespace + "_sparkle_plugin";

                if(!ros::isInitialized())
                {
                    int argc = 0;
                    char **argv = NULL;
                    ros::init(argc, argv, plugin_name);
                }
                plugin_name = plugin_name + "_node";
                this->rosNode.reset(new ros::NodeHandle("sparklePlugin")); //Create a ROS Node.
                this->soVel = ros::SubscribeOptions::create<nav_msgs::Odometry>(this->robotNamespace + "/" + this->setVelTopic, 1, boost::bind(&Sparkle::OnROSMsg, this, _1), ros::VoidPtr(), &this->rosQueue); //Create a named topic and subscribe to it
                
                this->rosSubVel = this->rosNode->subscribe(this->soVel);

                this->rosQueueThread = thread(bind(&Sparkle::QueueThread, this)); //Spin up the queue helper thread
            }
        public:
            void OnROSMsg(const nav_msgs::OdometryConstPtr & _msg)
            {
                this->velx = _msg->twist.twist.linear.x;
                this->vely = _msg->twist.twist.linear.z;                
                this->angularz = _msg->pose.pose.orientation.z; //This is yaw, NOT the Yaw rate.

                this->vAngZ = _msg->twist.twist.angular.z;

                this->v = sqrt(pow(this->velx,2) +  pow(this->vely, 2));

                //ROS_INFO_STREAM("Sparkle: I got the velocity message to be imparted to "<<this->robotNamespace << " is "<< this->velx);
                this->model->SetLinearVel(ignition::math::Vector3d((this->v)*cos(this->angularz), (this->v)*sin(this->angularz), 0.0));
                this->model->SetAngularVel(ignition::math::Vector3d(0, 0, this->vAngZ ));
            }

        private:
            void QueueThread()
            {
                static const double timeout = 0.1;
                while(this->rosNode->ok())
                {
                    this->rosQueue.callAvailable(ros::WallDuration(timeout));
                }
            }
            
    };
    GZ_REGISTER_MODEL_PLUGIN(Sparkle)
}
