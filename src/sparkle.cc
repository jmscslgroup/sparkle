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
#include<math.h>

#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<ros/subscribe_options.h>

#include<tf/transform_broadcaster.h>

#include<std_msgs/String.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<gazebo_msgs/ModelState.h>
#include<gazebo_msgs/ModelStates.h>
#include<gazebo_msgs/SetModelState.h>
#include<ignition/math/Vector3.hh>
#include<ignition/math/Box.hh>

#include<thread>
using namespace std;

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
            string controlMethod; // Control Method for updating Model State

            unique_ptr<ros::NodeHandle> rosNode; //A node used for ROS Transport
            ros::Subscriber rosSubVel;
            ros::SubscribeOptions soVel;
            
            ros::Subscriber rosSubAngles;
            ros::SubscribeOptions soAngles;

            ros::ServiceClient client;
            
            ros::Publisher modelPub;
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
                
                ROS_INFO_STREAM("Sparkle.cc: Bounding box of model: x is "<< this->boundingBox.XLength()<<" y is "<<this->boundingBox.YLength());
                if(_sdf->HasElement("robotNamespace"))
                {
                    this->robotNamespace = _sdf->GetElement("robotNamespace")->Get<string>();
                    ROS_INFO_STREAM("Sparkle.cc: Robot namespace is "<<this->robotNamespace);
                }
                else
                {
                    ROS_INFO_STREAM("Sparkle.cc: Robot Namespace not found. Setting robotnamespace to 'catvehicle'");
                }

                if(_sdf->HasElement("setVelTopic"))
                {
                    this->setVelTopic = _sdf->GetElement("setVelTopic")->Get<string>();
                    ROS_INFO_STREAM("Sparkle.cc: setVelTopic is set to "<<this->setVelTopic);
                }
                else
                {
                    ROS_INFO_STREAM("Sparkle.cc: setVelTopic not found. Make sure you do not prefix setVelTopic with / or robotNamespace. Setting setVelTopic to 'setvel'.");
                    this->setVelTopic  = "setvel";
                }
                
                if(_sdf->HasElement("controlMethod"))
                {
                    this->controlMethod = _sdf->GetElement("controlMethod")->Get<string>();
                }
                else
                {
                    this->controlMethod = "pose";
                }
                string plugin_name = this->robotNamespace + "_sparkle_plugin";

                if(!ros::isInitialized())
                {
                    int argc = 0;
                    char **argv = NULL;
                    ros::init(argc, argv, plugin_name);
                }
                plugin_name = plugin_name + "_node";
                this->rosNode.reset(new ros::NodeHandle(plugin_name)); //Create a ROS Node.
                this->soVel = ros::SubscribeOptions::create<nav_msgs::Odometry>(this->robotNamespace + "/" + this->setVelTopic, 1, boost::bind(&Sparkle::OnROSMsg, this, _1), ros::VoidPtr(), &this->rosQueue); //Create a named topic and subscribe to it
                
                this->rosSubVel = this->rosNode->subscribe(this->soVel);
                this->modelPub = this->rosNode->advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);

                this->client = this->rosNode->serviceClient<gazebo_msgs::SetModelState>("/gazebo/SetModelState");
                this->rosQueueThread = thread(bind(&Sparkle::QueueThread, this)); //Spin up the queue helper thread
            }
        public:
            void OnROSMsg(const nav_msgs::OdometryConstPtr & _msg)
            {
                ROS_INFO_STREAM("Got a message");
                if(this->controlMethod == "pose")
                {
                    geometry_msgs::Point pose;
                    geometry_msgs::Quaternion orientation;
                    geometry_msgs::Twist twist;

                    pose.x = _msg->pose.pose.position.x;
                    pose.y = _msg->pose.pose.position.y;
                    pose.z = _msg->pose.pose.position.z;
                    
                    orientation = rpyToQuaternion(_msg->pose.pose.orientation.x,
                                                _msg->pose.pose.orientation.y,
                                                _msg->pose.pose.orientation.z);
                    
                    ROS_INFO_STREAM("Orientation: x = " <<orientation.x << ", y = "<<orientation.y 
                                        <<", z = "<<orientation.z << ", w = "<<orientation.w);
                    

                    twist = _msg->twist.twist;

                    gazebo_msgs::ModelState modelState;
                    modelState.model_name = this->robotNamespace.substr(1, this->robotNamespace.length()-1);
                    modelState.pose.position = pose;
                    modelState.pose.orientation = orientation;
                    modelState.twist = twist;
                    modelState.reference_frame = "world";
                    
                    this->modelPub.publish(modelState);
                    
                    /*
                    gazebo_msgs::SetModelState srv;
                    srv.request.model_state = modelState;

                    if(this->client.call(srv))
                    {
                        ROS_INFO("Robot moved");
                    }
                    else
                    {
                        ROS_ERROR_STREAM("Robot moving Failed:"<< modelState.model_name);
                    }*/
                }
                else
                {

                    this->velx = _msg->twist.twist.linear.x;
                    this->vely = _msg->twist.twist.linear.z;                
                    this->angularz = _msg->pose.pose.orientation.z; //This is yaw, NOT the Yaw rate.

                    this->vAngZ = _msg->twist.twist.angular.z;

                    this->v = sqrt(pow(this->velx,2) +  pow(this->vely, 2));

                    this->model->SetLinearVel(ignition::math::Vector3d((this->v)*cos(this->angularz), (this->v)*sin(this->angularz), 0.0));
                    this->model->SetAngularVel(ignition::math::Vector3d(0, 0, this->vAngZ ));
                }
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
