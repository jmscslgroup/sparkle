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
	public:
		ros::NodeHandle* rosnode; //Create ROS Node Handle
    private:
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
        //std::map<std::string, std::tuple<bool, px, py, pz, qx, qy, qz, qw, vx, vy, vz, ax, ay, az, int>> states;
        // std::map<std::string, std::tuple<bool, double, double, double, double, double, double, double, double, double, double, double, double, double, int>> states;

        std::vector<bool> status_list;
        std::vector<double> posx_list;
        std::vector<double> posy_list;
        std::vector<double> posz_list;
        std::vector<double> Qx_list;
        std::vector<double> Qy_list;
        std::vector<double> Qz_list;
        std::vector<double> Qw_list;
        std::vector<double> linearx_list;
        std::vector<double> lineary_list;
        std::vector<double> linearz_list;
        std::vector<double> angularx_list;
        std::vector<double> angulary_list;
        std::vector<double> angularz_list;
        


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

            ROS_INFO_STREAM("Robot Indices size = "<<robot_indices.size());
            ROS_INFO_STREAM("Namespace size = "<<namespaces.size());
            
            this->v_isSubscribed = std::vector<bool>(robot_indices.size());
            this->status_list = std::vector<bool>(robot_indices.size());
            this->posx_list = std::vector<double>(robot_indices.size());
            this->posy_list = std::vector<double>(robot_indices.size());
            this->posz_list = std::vector<double>(robot_indices.size());
            this->Qx_list = std::vector<double>(robot_indices.size());
            this->Qy_list = std::vector<double>(robot_indices.size());
            this->Qz_list = std::vector<double>(robot_indices.size());
            this->Qw_list = std::vector<double>(robot_indices.size());
            this->linearx_list = std::vector<double>(robot_indices.size());
            this->lineary_list = std::vector<double>(robot_indices.size());
            this->linearz_list = std::vector<double>(robot_indices.size());
            this->angularx_list = std::vector<double>(robot_indices.size());
            this->angulary_list = std::vector<double>(robot_indices.size());
            this->angularz_list = std::vector<double>(robot_indices.size());


            for (int hh=0; hh < this->v_isSubscribed.size() ; ++hh)
            {
                this->v_isSubscribed.at(hh) = false;
                this->posx_list.at(hh) = 0.0;
                this->status_list.at(hh) = false;
                this->posx_list.at(hh) = 0.0;
                this->posy_list.at(hh) = 0.0;
                this->posz_list.at(hh) = 0.0;
                this->Qx_list.at(hh) = 0.0;
                this->Qy_list.at(hh) = 0.0;
                this->Qz_list.at(hh) = 0.0;
                this->Qw_list.at(hh) = 0.0;
                this->linearx_list.at(hh) = 0.0;
                this->lineary_list.at(hh) = 0.0;
                this->linearz_list.at(hh) = 0.0;
                this->angularx_list.at(hh) = 0.0;
                this->angulary_list.at(hh) = 0.0;
                this->angularz_list.at(hh) = 0.0;
            }

            this->pub = this->rosnode->advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);


        }
        
        void setVelCallBack(const boost::shared_ptr<nav_msgs::Odometry const>& _msg, std::string topic_name)
        {
            // std::tuple<bool, double, double, double, double, double, double, double, double, double, double, double, double, double, int> state_entry = this->states.at(topic_name);
            // if( std::get<0>(state_entry) )
            // {
            //     ROS_INFO_STREAM("Returned");
            //     return;
            // }


            for(int hh = 0; hh < this->namespaces.size(); ++hh)
            {
                if(this->namespaces.at(hh) == topic_name.substr(0, topic_name.substr(1, topic_name.length()-1).find(std::string("/")) +1 ))
                {
                    if(this->status_list.at(hh))
                    {
                        //ROS_INFO_STREAM("Returned");
                    }
                }
            }
            geometry_msgs::Point pose;
            geometry_msgs::Quaternion orientation;
            geometry_msgs::Twist twist;
            pose.x = _msg->pose.pose.position.x;
            pose.y = _msg->pose.pose.position.y;
            //ROS_INFO_STREAM("Topic_name: "<<topic_name<<", x="<<pose.x<<" y="<<pose.y);
            pose.z = _msg->pose.pose.position.z;
            orientation = _msg->pose.pose.orientation;

            twist = _msg->twist.twist;
            // ROS_INFO_STREAM("Size of the map is "<<this->states.size());
            // this->states.emplace(topic_name, std::make_tuple( true, _msg->pose.pose.position.x, 
            //                                                         _msg->pose.pose.position.y,
            //                                                         _msg->pose.pose.position.z,
            //                                                         _msg->pose.pose.orientation.x,
            //                                                         _msg->pose.pose.orientation.y, 
            //                                                         _msg->pose.pose.orientation.z,
            //                                                         _msg->pose.pose.orientation.w,
            //                                                         _msg->twist.twist.linear.x,
            //                                                         _msg->twist.twist.linear.y, 
            //                                                         _msg->twist.twist.linear.z, 
            //                                                         _msg->twist.twist.angular.x,
            //                                                         _msg->twist.twist.angular.y,
            //                                                         _msg->twist.twist.angular.z, 
            //                                                         22) );


            //for(const auto& elem : this->states)

            for(int hh = 0; hh < this->namespaces.size(); ++hh)
            {
                if(this->namespaces.at(hh) == topic_name.substr(0, topic_name.substr(1, topic_name.length()-1).find(std::string("/")) +1 ))
                {
                    this->status_list.at(hh) = true;
                    this->posx_list.at(hh) = _msg->pose.pose.position.x;
                    this->posy_list.at(hh) = _msg->pose.pose.position.y;
                    this->posz_list.at(hh) = _msg->pose.pose.position.z;
                    this->Qx_list.at(hh) = _msg->pose.pose.orientation.x;
                    this->Qy_list.at(hh) = _msg->pose.pose.orientation.y; 
                    this->Qz_list.at(hh) = _msg->pose.pose.orientation.z;
                    this->Qw_list.at(hh) = _msg->pose.pose.orientation.w;
                    this->linearx_list.at(hh) = _msg->pose.pose.orientation.z;
                    this->lineary_list.at(hh) = _msg->twist.twist.linear.x;
                    this->linearz_list.at(hh) = _msg->twist.twist.linear.y; 
                    this->angularx_list.at(hh) = _msg->twist.twist.linear.z; 
                    this->angulary_list.at(hh) = _msg->twist.twist.angular.x;
                    this->angularz_list.at(hh) = _msg->twist.twist.angular.y;
                }
            }
            
           /* for(int hh = 0; hh < this->namespaces.size(); ++hh)
            {
                ROS_INFO_STREAM(this->namespaces.at(hh) << " ] X  = "<< this->posx_list.at(hh));
                ROS_INFO_STREAM(this->namespaces.at(hh) << " ] Status  = "<< this->status_list.at(hh));
            }
            */
            // auto it = this->states.begin();

            // for(int i = 0; i < static_cast<int>(this->states.size()); i++)
            // {
            //     double x = std::get<1>(it->second);
            //     ROS_INFO_STREAM("key = "<< it->first<<", status = "<<std::get<0>(it->second) << " x = "<< x<<", dummy="<<std::get<14>(it->second));
            //     it++;
            //     ROS_INFO_STREAM(this->namespaces.at(i) << " ] X  = "<< this->posx_list.at(i));
            // }

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

                // geometry_msgs::Point pose;
                // pose.x = 442.0;
                // geometry_msgs::Quaternion orientation;
                // geometry_msgs::Twist twist;
                // this->states.emplace(this->namespaces.at(pp) + "/"+ odomVelTopic, std::make_tuple(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 28) );
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
                //ROS_INFO_STREAM("Number of subscribes are "<<n_subs <<" which is less than total number of robots.");
                return false;
            }

        }

        void publishModelStates()
        {
            unsigned int n_states = 0;
            std::vector<gazebo_msgs::ModelState> modelStates;
            
            this->rosnode->param(this->nodename +"/"+"enablePos", this->enablePos, true);
            this->rosnode->param(this->nodename +"/"+"enableTwist", this->enableTwist, false);
            
            // for(int yy  = 0; yy < this->namespaces.size(); ++yy)
            // {
            //     std::tuple<bool, double, double, double, double, double, double, double, double, double, double, double, double, double, int> state_entry = this->states.at(this->namespaces.at(yy) + "/"+ this->odomVelTopic);
            //     if( std::get<0>(state_entry) )
            //     {
            //         n_states++;
            //         gazebo_msgs::ModelState t_modelstate;
            //         t_modelstate.model_name = this->namespaces.at(yy);
            //         if (this->enablePos)
            //         {
            //             t_modelstate.pose.position.x = std::get<1>(state_entry);
            //             t_modelstate.pose.position.y = std::get<2>(state_entry);
            //             t_modelstate.pose.position.z = std::get<3>(state_entry);
            //             t_modelstate.pose.orientation.x = std::get<4>(state_entry);
            //             t_modelstate.pose.orientation.y = std::get<5>(state_entry);
            //             t_modelstate.pose.orientation.z = std::get<6>(state_entry);
            //             t_modelstate.pose.orientation.w = std::get<7>(state_entry);
            //         }
            //         if(this->enableTwist)
            //         {
            //             t_modelstate.twist.linear.x = std::get<8>(state_entry);
            //             t_modelstate.twist.linear.y = std::get<9>(state_entry);
            //             t_modelstate.twist.linear.z = std::get<10>(state_entry);
            //             t_modelstate.twist.angular.x = std::get<11>(state_entry);
            //             t_modelstate.twist.angular.y = std::get<12>(state_entry);
            //             t_modelstate.twist.angular.z = std::get<13>(state_entry);
                        
            //         }
            //         t_modelstate.reference_frame = "world";
            //         modelStates.push_back(t_modelstate);

            //     }
            // }
            
            for(int yy  = 0; yy < this->namespaces.size(); ++yy)
            {
                if(status_list.at(yy))
                {
                    n_states++;
                    gazebo_msgs::ModelState t_modelstate;
                    t_modelstate.model_name = this->namespaces.at(yy);
                    t_modelstate.model_name = t_modelstate.model_name.substr(1, t_modelstate.model_name.length()-1);
                    if (this->enablePos)
                    {
                        t_modelstate.pose.position.x = this->posx_list.at(yy);
                        t_modelstate.pose.position.y = this->posy_list.at(yy);
                        t_modelstate.pose.position.z = this->posz_list.at(yy);
                        t_modelstate.pose.orientation.x = this->Qx_list.at(yy);
                        t_modelstate.pose.orientation.y = this->Qy_list.at(yy);
                        t_modelstate.pose.orientation.z = this->Qz_list.at(yy);
                        t_modelstate.pose.orientation.w = this->Qw_list.at(yy);
                    }
                    if(this->enableTwist)
                    {
                        t_modelstate.twist.linear.x = this->linearx_list.at(yy);
                        t_modelstate.twist.linear.y = this->lineary_list.at(yy);
                        t_modelstate.twist.linear.z = this->linearz_list.at(yy);
                        t_modelstate.twist.angular.x = this->angularx_list.at(yy);
                        t_modelstate.twist.angular.y = this->angulary_list.at(yy);
                        t_modelstate.twist.angular.z = this->angularz_list.at(yy);
                        
                    }
                    t_modelstate.reference_frame = "world";
                    modelStates.push_back(t_modelstate);
                }
            }

            //ROS_INFO_STREAM("Number of vehicles = "<<n_states);
            /*
            1. Pause Gazebo
            2. Set static variable to the current size of the queue
            3. When model state is received by publisher, decrement static variable
            4. If static variable is zero then unpause the Gazebo
            5. For receipt confirmation, change publisher to rosservice call back. Question: if Gazebo is paused, then will rosservice call send receipt.
            6. Metric Assessment
            */
            if(n_states == this->namespaces.size())
            {
                for(int yy  = 0; yy<this->namespaces.size(); ++yy)
                {
                    ROS_INFO_STREAM("Published................................................................................");
                    this->pub.publish(modelStates.at(yy));
                    // this->states.emplace(this->namespaces.at(yy) + "/"+ this->odomVelTopic, std::make_tuple(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 34) );
                }
            }
            
            else
            {
                ROS_INFO_STREAM("Not gonna publish");
            }

            //Reset


            if(n_states == this->namespaces.size())
            {
                for(int hh  = 0; hh<this->namespaces.size(); ++hh)
                {
                    this->v_isSubscribed.at(hh) = false;
                    this->posx_list.at(hh) = 0.0;
                    this->status_list.at(hh) = false;
                    this->posx_list.at(hh) = 0.0;
                    this->posy_list.at(hh) = 0.0;
                    this->posz_list.at(hh) = 0.0;
                    this->Qx_list.at(hh) = 0.0;
                    this->Qy_list.at(hh) = 0.0;
                    this->Qz_list.at(hh) = 0.0;
                    this->Qw_list.at(hh) = 0.0;
                    this->linearx_list.at(hh) = 0.0;
                    this->lineary_list.at(hh) = 0.0;
                    this->linearz_list.at(hh) = 0.0;
                    this->angularx_list.at(hh) = 0.0;
                    this->angulary_list.at(hh) = 0.0;
                    this->angularz_list.at(hh) = 0.0;
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
    
    double rate;
    multiModelUpdate->rosnode->param("rate", rate, 20.0);
    ROS_INFO_STREAM("Rate at which model will be updated is "<<rate);
    ros::Rate loop_rate(rate);
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
