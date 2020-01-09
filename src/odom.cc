/*---------------------------------------------------
Author: Rahul Bhadani
Date: December 2019

This class implements a Gazebo plugin that publishes 
odometry information for Sparkle car model

---------------------------------------------------*/
#include<ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <tf/transform_broadcaster.h>

#include <ignition/math/Box.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include<nav_msgs/Odometry.h>

#include<iostream>
#include<cstdlib>
#include<string>
using namespace std;

namespace gazebo
{
    class odometry: public ModelPlugin
    {
        private:

            string model_name; //Save the model name 
            string odometry_topic; // On what topic I should publish model's odometry data
            string tf_scope; 

            string model_state_topic; // What is my model state topic?
            
            physics::ModelPtr model; // Model Pointer
            ignition::math::Box bounding_box_model; //Bounding Box of the model
            ignition::math::Box collision_box_model; // Bounding Box of the collision of the model
            ignition::math::Pose3d model_poses;
            ignition::math::Vector3<double> model_linear_vel;
            ignition::math::Vector3<double> model_angular_vel;
            event::ConnectionPtr updateConnection;
            
            ros::Publisher odometry_publisher;
            ros::NodeHandle* rosnode;

            
        public:
            odometry()
            {
                model_name="";
                odometry_topic="/odom";
                
            }
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
            {
                if(!ros::isInitialized)
                {
                    ROS_FATAL_STREAM("ROS node for Gazebo not established. Plugin failed.");
                    return;
                }

                ROS_INFO_STREAM("Odometry ModelPlugin Loaded.");
                this->model = _parent;

                this->model_name = this->model->GetName();
                this->tf_scope = this->model_name;
                this->odometry_topic = "/" + this->model_name + "/odom";
                ROS_INFO_STREAM("[Odometry ModelPlugin] Name of the model is "<< this->model_name);
                ROS_INFO_STREAM("[Odometry ModelPlugin] Odometry topic set is  "<< this->odometry_topic);
               
                this->rosnode = new ros::NodeHandle(this->model_name);
                this->odometry_publisher =  this->rosnode->advertise<nav_msgs::Odometry>(odometry_topic, 1);

                this->bounding_box_model = this->model->BoundingBox();
                this->collision_box_model = this->model->CollisionBoundingBox();

                ROS_INFO_STREAM("[Odometry ModelPlugin] Bounding box of "<<this->model_name<<"= X:"<<this->bounding_box_model.XLength()
                        <<", Y:"<<this->bounding_box_model.YLength()
                        <<", Z:"<<this->bounding_box_model.ZLength());

                ROS_INFO_STREAM("[Odometry ModelPlugin] Collision Bounding box of "<<this->model_name<<"= X:"<<this->collision_box_model.XLength()
                        <<", Y:"<<this->collision_box_model.YLength()
                        <<", Z:"<<this->collision_box_model.ZLength());
                
                // Listen to the update event. This event is broadcast every simulation iteration.
                this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&odometry::OnUpdate, this));
            }
            
            // Called by the world update start event
            public: void OnUpdate()
            {
                ros::Time current_time = ros::Time::now();
                this->model_poses = this->model->RelativePose();
                this->model_linear_vel = this->model->RelativeLinearVel();
                this->model_angular_vel = this->model->RelativeAngularVel ();

                ignition::math::Vector3<double> pos = this->model_poses.Pos();
                ignition::math::Quaternion<double> quat = this->model_poses.Rot();

                ROS_DEBUG_STREAM(this->model_name<<"[Odometry ModelPlugin] Pos = X: "<<pos.X() <<" Y: "<<pos.Y() <<" Z: "<<pos.Z());
                ROS_DEBUG_STREAM(this->model_name<<"[Odometry ModelPlugin] Quat = X: "<<quat.X() <<" Y: "<<quat.Y() <<" Z: "<<quat.Z() << "W: "<<quat.W());
                
                ROS_DEBUG_STREAM(this->model_name<<"[Odometry ModelPlugin] Linear Vel = X: "<<this->model_linear_vel.X() <<" Y: "<<this->model_linear_vel.Y() <<" Z: "<<this->model_linear_vel.Z());
                ROS_DEBUG_STREAM(this->model_name<<"[Odometry ModelPlugin] Angular = X: "<<this->model_angular_vel.X() <<" Y: "<<this->model_angular_vel.Y() <<" Z: "<<this->model_angular_vel.Z());
                
                static tf::TransformBroadcaster br;
                tf::Transform transform;
                transform.setOrigin( tf::Vector3(pos.X(),
                            pos.Y(),
                            pos.Z()));
                tf::Quaternion q(quat.X(),
                        quat.Y(),
                        quat.Z(),
                        quat.W());
                    transform.setRotation(q);
                    br.sendTransform(tf::StampedTransform(transform, current_time,
                                this->tf_scope + "/odom",
                                this->tf_scope + "/base_link") );

                // grab the odometry from the incoming msg and post it
                nav_msgs::Odometry odom;
                odom.header.stamp = current_time;
                odom.header.frame_id = this->tf_scope + "/odom";
                // we calculate our index in the pose msg by
                odom.child_frame_id = this->tf_scope + "/base_link";
                odom.pose.pose.position.x = pos.X();
                odom.pose.pose.position.y = pos.Y();
                odom.pose.pose.position.z = pos.Z();
                odom.pose.pose.orientation.x = quat.X();
                odom.pose.pose.orientation.y = quat.Y();
                odom.pose.pose.orientation.z = quat.Z();
                odom.pose.pose.orientation.w = quat.W();

                odom.twist.twist.linear.x = this->model_linear_vel.X();
                odom.twist.twist.linear.y = this->model_linear_vel.Y();
                odom.twist.twist.linear.z = this->model_linear_vel.Z();
                odom.twist.twist.angular.x = this->model_angular_vel.X();
                odom.twist.twist.angular.y = this->model_angular_vel.Y();
                odom.twist.twist.angular.z = this->model_angular_vel.Z();

                odometry_publisher.publish(odom);
            }
        
    };
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(odometry)
}
