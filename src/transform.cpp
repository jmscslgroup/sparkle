/*-------------------------------------------------------------------------


Author: Rahul Bhadani
Initial date: 1st Aug 2019

What: A node to broadcast odom tf of the model

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

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>

using namespace std;
string robotNamespace;

void publishTf(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
 //   ROS_INFO_STREAM("It is time to pPublish TF");
    ros::Time current_time = ros::Time::now();
    vector<string> modelNames = msg->name;
    int index = -1;

    //Find out which index has model state
    for (int i = 0; i<modelNames.size() && index <0; ++i)
    {
        if( robotNamespace == string(modelNames[i]))
        {
            index = i;
        }
    }

    if(index == -1)
    {
      // ROS_ERROR_STREAM(" Failed to find odometry for the model name " << robotNamespace << " = " <<    index);
    }
    else
    {
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
                    robotNamespace + "/odom",
                    robotNamespace + "/base_link"));
   //     ROS_INFO_STREAM("Published a transform");

    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sparkle_odom_tf");
    
    ros::NodeHandle nodeHandle("~");

    ROS_INFO_STREAM("Sparkle: Odom tf broadcaster started");
    
    robotNamespace = ros::this_node::getNamespace();

    robotNamespace = robotNamespace.substr(1, robotNamespace.size() - 1);
    
    ROS_INFO_STREAM("RobotNamespace retrieved is " <<robotNamespace);

    ros::Subscriber sub = nodeHandle.subscribe("/gazebo/model_states", 1, &publishTf);

    ros::spin();

    return 0;
}
