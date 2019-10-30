/* (C) Rahul Bhadani
 Initial Date: 28th October 2019
 Purpose: Set a timer companion to artifically change publishing rate of Simulink generated ROS Node.
 This ROS Node will enable and disable a ROS parameter based on given time interval/sampling time or
 desired frequency of publishing.

*/

#include <cstdio>
#include <iostream>
#include <ros/ros.h>
#include <ctime>
#include <cstdlib>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "timer_companion");
    
    ros::NodeHandle nodeHandle("~");

    ros::Rate loop_rate(1000);

    /* Desired Publish rate: This will be passed as ROS Param */
    double pub_rate;
    

    /*do_publish is the enable flag for ROS Publishers whose publish rate we want to manipulate artifically*/
    bool do_publish;

    /* Default is 10 Hz, i.e, we want to artificially manipulate another ROS parameter
    and sets its value to true every 0.1 second and then set false immediately */
    
    //nodeHandle.setParam("pub_rate", 10.0);
    
    nodeHandle.param("pub_rate", pub_rate, 10.0); 
    
    nodeHandle.getParam("pub_rate", pub_rate);

    nodeHandle.setParam("do_publish", false);
    
//    nodeHandle.param("do_publish", do_publish, false);

    ros::Time lastTime = ros::Time::now();

    while(ros::ok())
    {
        nodeHandle.setParam("do_publish",  false);
        ros::Time newTime = ros::Time::now();
        ros::Duration duration = newTime - lastTime;

        if (duration.toSec() >= 1.0/pub_rate)
        {
            lastTime = newTime;
            nodeHandle.setParam("do_publish",  true);
            ROS_INFO_STREAM("Published True");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO_STREAM("Shutting down...");

    return EXIT_SUCCESS;
}
