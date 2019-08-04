/*-------------------------------------------------------------------------


Author: Rahul Bhadani
Initial date: 1st Aug 2019

What: A Gazebo model-plugin to broadcast odom tf of the model

(C) 2015-2020 Rahul Bhadani, Arizona Board of Regents

Permission is hereby granted, without written agreement and without
license or royalty fees, to use, copy, modify, and distribute this
software and its documentation for any purpose, provided that the
above copyright notice and the following two paragraphs appear in
all copies of this software.

IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY
FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES
ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN
IF THE UNIVERSITY OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.

THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

---------------------------------------------------------------------------*/

#include <cstdio>
#include <stdlib.h>
#include <pwd.h>
#include <stdio.h>
#include <iostream>
#include <string>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>

using namespace std;


namespace gazebo
{
    class sparkletf: public ModelPlugin
    {
        //Read the namespace from urdf file
        std::string robotNamespace;

        //What's the tfScope
        std::string tfScope;

        //What's the odomTopic
        std::string odomTopic;

        //Pointer to the world 
        physics::WorldPtr world;

        //Pointer to the model entity
        physics::ModelPtr model;

        ros::Subscriber sub;
        ros::NodeHandle* rosnode;
        boost::thread ros_spinner_thread;

        public:

        sparkletf()
        {
            this->robotNamespace = "";
            this->tfScope = "";
            this->odomTopic = "";
        }

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            this->model = _parent;
            this->world = _parent->GetWorld();
            
            if(_sdf->HasElement("robotNamespace"))
            {
                this->robotNamespace = _sdf->GetElement("robotNamespace")->Get<string>();
            }
            
            this->tfScope = this->robotNamespace.substr(1, this->robotNamespace.size() - 1);

            //Start ROS NODE

            int argc = 0;
            char** argv = NULL;
            ros::init(argc, argv, "sparkleTF", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);

            this->rosnode = new ros::NodeHandle(this->robotNamespace);
            
            this->ros_spinner_thread = boost::thread(boost::bind( &sparkletf::rosThread, this));
            
            //end Load
        }
        void rosThread()
        {
            ROS_INFO_STREAM("SparkleTf: Call back thread id = " <<boost::this_thread::get_id());
            ros::NodeHandle nodeHandleForTf;
            this->sub = nodeHandleForTf.subscribe("/gazebo/model_states", 1, &sparkletf::publishTf, this);

            /*
            ros::Rate loop_rate(10);

            while(this->rosnode->ok())
            {
                ros::spinOnce();
                loop_rate.sleep();
            }
            */
        }

        void publishTf(const gazebo_msgs::ModelStates::ConstPtr& msg)
        {
            ros::Time current_time = ros::Time::now();
            vector<string> modelNames = msg->name;
            int index = -1;

            //Find out which index has model state
            for (int i = 0; i<modelNames.size() && index <0; ++i)
            {
                if(this->robotNamespace == std::string("/" + modelNames[i]))
                {
                    index = i;
                }
            }
        
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            
            ROS_INFO_STREAM("Index = "<<index);
        
            if(index == -1)
            {
                ROS_ERROR_STREAM("Unable to find odometry for the model name " << this->robotNamespace << " = " << index);
            }
            else
            {
                /*
                
                static tf::TransformBroadcaster br;
                tf::Transform transform;
                
                transform.setOrigin(tf::Vector3(msg->pose[index].position.x,
                    msg->pose[index].position.y,
                    msg->pose[index].position.z) );
                
                tf::Quaternion q(msg->pose[index].orientation.x,
                    msg->pose[index].orientation.y,
                    msg->pose[index].orientation.z,
                    msg->pose[index].orientation.w);
                
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, current_time, 
                    this->tfScope + "/odom",
                    this->tfScope + "/base_link"));*/
                ROS_INFO_STREAM("Published a transform");
            }
        }
    };
    GZ_REGISTER_MODEL_PLUGIN(sparkletf)
}
