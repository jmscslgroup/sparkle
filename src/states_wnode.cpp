/*---------------------------------------------------
Author: Rahul Bhadani
Date: October 2021

This class implements a Gazebo World plugin to update models

---------------------------------------------------*/

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
#include<map>
#include<tuple>

#include "mymath.cc"


class UpdateStates
{
	private:
		ros::NodeHandle* rosnode; //Create ROS Node Handle
        std::vector <ros::Subscriber> v_subs; // A vector of subscribers to topics that produces Odometry information.
        ros::Publisher pub; // This will always be /gazebo/set_model_state
        
        std::vector <bool> v_isSubscribed;// check if subscribers have been created

        
        bool enableTwist;
        bool enablePos;
        // Get the node name
        std::string nodename;
        std::string odomVelTopic;

        std::vector<std::string> namespaces;
        std::vector<std::string> nodenames;

        gazebo_msgs::ModelState modelState;
        
        // get the list model(car) nodes
        std::vector<std::string> robotNodeList;

        // Key = /robot/setvel
        std::map<std::string, std::tuple<bool, geometry_msgs::Point, geometry_msgs::Quaternion, geometry_msgs::Twist, int>> states;

        

    public:
        UpdateStates(std::string robot_model_name = "bicycle")
        {
            this->nodename = ros::this_node::getName();
            std::vector<std::string> nodeList; //Placeholder to save the list of all nodes
            ros::master::getNodes(nodeList); // Get the list of all nodes currently available
            int n_nodes = nodeList.size(); // Get the number of nodes currently available
            
            std::vector<unsigned int>  robot_indices; // What indices on node list corresponds to robot `robot_model_name`
            for (int kk = 0; kk < n_nodes; kk++)
            {
                std::string temp_name  = nodeList.at(kk);

                if(temp_name.find(robot_model_name) != std::string::npos) // if current node is a robot name then
                {
                    ROS_INFO_STREAM("Node is "<< temp_name);
                    robot_indices.push_back(kk); // then push the index to `robot_indices`
                    nodenames.push_back(temp_name);
                    // retrieve the namespace of the robot
                    namespaces.push_back(temp_name.substr(0, temp_name.substr(1, temp_name.length()-1).find(std::string("/")) +1 ));
                }
            }

            this->rosnode = new ros::NodeHandle("");
            this->enableTwist = false;
            this->enablePos = false;
            
            this->v_isSubscribed = std::vector<bool>(robot_indices.size());

            for (int f=0; f < this->v_isSubscribed.size() ; ++f)
            {
                this->v_isSubscribed.at(f) = false;
            }

            this->pub = this->rosnode->advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);


        }
        
        void setVelCallBack(const boost::shared_ptr<nav_msgs::Odometry const>& _msg, std::string topic_name)
        {
            std::tuple<bool, geometry_msgs::Point, geometry_msgs::Quaternion, geometry_msgs::Twist, int> state_entry = this->states.at(topic_name);
            if( std::get<0>(state_entry) )
            {
                ROS_INFO_STREAM("Returned");
                return;
            }

            geometry_msgs::Point pose;
            geometry_msgs::Quaternion orientation;
            geometry_msgs::Twist twist;
            pose.x = _msg->pose.pose.position.x;
            pose.y = _msg->pose.pose.position.y;
            ROS_INFO_STREAM("Topic_name: "<<topic_name<<", x="<<pose.x<<" y="<<pose.y);
            pose.z = _msg->pose.pose.position.z;
            orientation = _msg->pose.pose.orientation;

            twist = _msg->twist.twist;
            
            this->states.emplace(topic_name, std::make_tuple( true, pose, orientation, twist, 22) );
            for(const auto& elem : this->states)
            {
                geometry_msgs::Point tpose = std::get<1>(elem.second);
                ROS_INFO_STREAM("key = "<< elem.first<<", status = "<<std::get<0>(elem.second) << " x = "<< tpose.x<<", dummy="<<std::get<4>(elem.second));
            }

        }

        void createSubscribers(std::string odomVelTopic = std::string("setvel"))
        {
            this->odomVelTopic = odomVelTopic;
            for(int pp = 0; pp < this->namespaces.size(); ++pp)
            {
                ros::SubscribeOptions so;
                so.init<nav_msgs::Odometry>(this->namespaces.at(pp) + "/"+ odomVelTopic, 1, boost::bind(&UpdateStates::setVelCallBack, this, _1, this->namespaces.at(pp) + "/"+ odomVelTopic) );
                this->v_subs.push_back( this->rosnode->subscribe(so) );
                this->v_isSubscribed.at(pp) = true;

                geometry_msgs::Point pose;
                pose.x = 442.0;
                geometry_msgs::Quaternion orientation;
                geometry_msgs::Twist twist;
                this->states.emplace(this->namespaces.at(pp) + "/"+ odomVelTopic, std::make_tuple(false, pose, orientation, twist, 28) );
            }
        }

        bool subscribed()
        {
            unsigned int n_subs = 0;
            for(int yy  = 0; yy<this->namespaces.size(); ++yy)
            {
                if(this->v_isSubscribed.at(yy))
                    n_subs++;
            }
            if(n_subs == this->namespaces.size())
            {
                return true;
            }
            else
            {
                ROS_INFO_STREAM("Number of subscribes are "<<n_subs <<" which is less than total number of robots.");
                return false;
            }

        }

        void publishModelStates()
        {
            unsigned int n_states = 0;
            std::vector<gazebo_msgs::ModelState> modelStates;
            
            geometry_msgs::Point dummy_pose;
            geometry_msgs::Quaternion dummy_orientation;
            geometry_msgs::Twist dummy_twist;

            this->rosnode->param(this->nodename +"/"+"enablePos", this->enablePos, true);
            this->rosnode->param(this->nodename +"/"+"enableTwist", this->enableTwist, false);
            
            for(int yy  = 0; yy < this->namespaces.size(); ++yy)
            {
                std::tuple<bool, geometry_msgs::Point, geometry_msgs::Quaternion, geometry_msgs::Twist, int> state_entry = this->states.at(this->namespaces.at(yy) + "/"+ this->odomVelTopic);
                if( std::get<0>(state_entry) )
                {
                    n_states++;
                    gazebo_msgs::ModelState t_modelstate;
                    t_modelstate.model_name = this->namespaces.at(yy);
                    if (this->enablePos)
                    {
                        t_modelstate.pose.position = std::get<1>(state_entry);
                        t_modelstate.pose.orientation = std::get<2>(state_entry);
                    }
                    if(this->enableTwist)
                    {
                        t_modelstate.twist = std::get<3>(state_entry);
                    }
                    t_modelstate.reference_frame = "world";
                    modelStates.push_back(t_modelstate);

                }
            }
            ROS_INFO_STREAM("Number of vehicles = "<<n_states);
            if(n_states == this->namespaces.size())
            {
                for(int yy  = 0; yy<this->namespaces.size(); ++yy)
                {
                    this->pub.publish(modelStates.at(yy));
                    this->states.emplace(this->namespaces.at(yy) + "/"+ this->odomVelTopic, std::make_tuple(false, dummy_pose, dummy_orientation, dummy_twist, 34) );
                }
            }

        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "WorldPoses");
    
    // Create an object of type `UpdateStates`
    UpdateStates *multiModelUpdate = new UpdateStates();
    multiModelUpdate->createSubscribers();

    ros::Rate loop_rate(100);
    if (multiModelUpdate->subscribed() )
    {
        while( ros::ok() )
        {
            multiModelUpdate->publishModelStates();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return EXIT_SUCCESS;
}
