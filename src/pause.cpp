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
#include "gazebo_msgs/SetModelState.h"
#include <std_srvs/Empty.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "WorldPause"); 
    ros::NodeHandle nh("~"); //Create ROS Node Handle

    ros::ServiceClient pauseGazebo = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics"); 
    ros::ServiceClient unpauseGazebo = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    geometry_msgs::Point position; position.x = 1.0; position.y = 0.0; position.z = 0.0;
    geometry_msgs::Quaternion orientation; orientation.x = 0.0; orientation.y = 0.0; orientation.z = 0.0; orientation.w = 1.0;


    geometry_msgs::Pose start_pose;
    start_pose.position = position;
    start_pose.orientation = orientation;

    geometry_msgs::Twist start_twist; start_twist.linear.x = 0.0; start_twist.linear.y = 0.0; start_twist.linear.z = 0.0;
    start_twist.angular.x = 0.0; start_twist.angular.y = 0.0; start_twist.angular.z = 0.0;

    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = (std::string) "school";
    modelstate.pose = start_pose;
    modelstate.twist = start_twist;
    
    ros::ServiceClient stateclient = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState setmodelstate;
    setmodelstate.request.model_state = modelstate;


    std_srvs::Empty emptySrv;
    if(pauseGazebo.call(emptySrv))
    {
        ROS_INFO_STREAM("Paused");
    }
    else
    {
        ROS_INFO_STREAM("Failed to Pause");
    }
    
    start_pose.position.x += 10.0;
    modelstate.pose = start_pose;
    setmodelstate.request.model_state = modelstate;
    if(stateclient.call(setmodelstate))
    {
        ROS_INFO_STREAM("Successfully set the state of model"); 
    }
    else
    {
        ROS_INFO_STREAM("Setting the state of model stays unsuccessful"); 
    }
    sleep(10);
    if(unpauseGazebo.call(emptySrv))
    {
        ROS_INFO_STREAM("UnPaused");
    }
    else
    {
        ROS_INFO_STREAM("Failed to UnPause");
    }
    start_pose.position.x += 10.0;
    modelstate.pose = start_pose;
    setmodelstate.request.model_state = modelstate;
    if(stateclient.call(setmodelstate))
    {
        ROS_INFO_STREAM("Successfully set the state of model"); 
    }
    else
    {
        ROS_INFO_STREAM("Setting the state of model stays unsuccessful"); 
    }
    sleep(10);
    if(pauseGazebo.call(emptySrv))
    {
        ROS_INFO_STREAM("Paused");
    }
    else
    {
        ROS_INFO_STREAM("Failed to Pause");
    }
    start_pose.position.x += 10.0;
    modelstate.pose = start_pose;
    setmodelstate.request.model_state = modelstate;
    if(stateclient.call(setmodelstate))
    {
        ROS_INFO_STREAM("Successfully set the state of model"); 
    }
    else
    {
        ROS_INFO_STREAM("Setting the state of model stays unsuccessful"); 
    }
    sleep(10);
    if(unpauseGazebo.call(emptySrv))
    {
        ROS_INFO_STREAM("UnPaused");
    }
    else
    {
        ROS_INFO_STREAM("Failed to UnPause");
    }
    start_pose.position.x += 10.0;
    modelstate.pose = start_pose;
    setmodelstate.request.model_state = modelstate;
    if(stateclient.call(setmodelstate))
    {
        ROS_INFO_STREAM("Successfully set the state of model"); 
    }
    else
    {
        ROS_INFO_STREAM("Setting the state of model stays unsuccessful"); 
    }
    sleep(10);

    if(pauseGazebo.call(emptySrv))
    {
        ROS_INFO_STREAM("Paused");
    }
    else
    {
        ROS_INFO_STREAM("Failed to Pause");
    }
    start_pose.position.x += 10.0;
    modelstate.pose = start_pose;
    setmodelstate.request.model_state = modelstate;
    if(stateclient.call(setmodelstate))
    {
        ROS_INFO_STREAM("Successfully set the state of model"); 
    }
    else
    {
        ROS_INFO_STREAM("Setting the state of model stays unsuccessful"); 
    }
    sleep(10);
    if(unpauseGazebo.call(emptySrv))
    {
        ROS_INFO_STREAM("UnPaused");
    }
    else
    {
        ROS_INFO_STREAM("Failed to UnPause");
    }
    start_pose.position.x += 10.0;
    modelstate.pose = start_pose;
    setmodelstate.request.model_state = modelstate;
    if(stateclient.call(setmodelstate))
    {
        ROS_INFO_STREAM("Successfully set the state of model"); 
    }
    else
    {
        ROS_INFO_STREAM("Setting the state of model stays unsuccessful"); 
    }
    sleep(10);


    if(pauseGazebo.call(emptySrv))
    {
        ROS_INFO_STREAM("Paused");
    }
    else
    {
        ROS_INFO_STREAM("Failed to Pause");
    }
    start_pose.position.x += 10.0;
    modelstate.pose = start_pose;
    setmodelstate.request.model_state = modelstate;
    if(stateclient.call(setmodelstate))
    {
        ROS_INFO_STREAM("Successfully set the state of model"); 
    }
    else
    {
        ROS_INFO_STREAM("Setting the state of model stays unsuccessful"); 
    }
    sleep(10);
    if(unpauseGazebo.call(emptySrv))
    {
        ROS_INFO_STREAM("UnPaused");
    }
    else
    {
        ROS_INFO_STREAM("Failed to UnPause");
    }
    start_pose.position.x += 10.0;
    modelstate.pose = start_pose;
    setmodelstate.request.model_state = modelstate;
    if(stateclient.call(setmodelstate))
    {
        ROS_INFO_STREAM("Successfully set the state of model"); 
    }
    else
    {
        ROS_INFO_STREAM("Setting the state of model stays unsuccessful"); 
    }
    sleep(10);





    return 0;
}
