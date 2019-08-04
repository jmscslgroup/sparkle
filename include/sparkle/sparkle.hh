#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <cstdio>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <gazebo/physics/Joint.hh>
#include <cstdlib>
//#include <unistd.h>
#include<gazebo_msgs/ModelStates.h>
#include <ignition/math/Vector3.hh>
namespace gazebo
{
    class Sparkle : public ModelPlugin
    {
        public:
            Sparkle();
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        private:
            //void SimROSThread();;
            void SimROSThread();
            void changePos(const gazebo_msgs::ModelState::ConstPtr& msg);
            void changeVel(const geometry_msgs::Twist::ConstPtr& msg);
            physics::PhysicsEnginePtr physicsEngine;
            
            std::string posTopic;
            std::string velTopic;
            std::string robotNamespace;

            ros::Publisher modelPub;
            boost::thread ros_spinner_thread_;
            ros::NodeHandle* rosnode_;

            event::ConnectionPtr updateConnection;

            //Pointer to the model entity
            physics::ModelPtr model;
            //Pointer to the world in which the model exists
            physics::WorldPtr world;
            
            //rate at which to update the catsteering
            double updateRate;
            //Previous time when the catsteering was updated.
            ros::Time prevUpdateTime;

    };
}
