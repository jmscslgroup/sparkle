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
#include<rosgraph_msgs/Clock.h>
using namespace std;

int main(int argc, char **argv)
{
    // Before you do anything, do the ros init
    ros::init(argc, argv, "sclock");
    ros::NodeHandle n("~");
    double factor;
    double rate;
    n.param("factor", factor, 0.5);
    n.param("rate", rate, 100.0);
    
    ROS_INFO_STREAM("Factor = "<<factor);
    ROS_INFO_STREAM("Rate = "<<rate);

    ros::Publisher sclock_pub = n.advertise<rosgraph_msgs::Clock>("/clock", 1); // To publish scaled ROS Time
    ros::Publisher wclock_pub = n.advertise<rosgraph_msgs::Clock>("/wall_clock", 1); // To publish actual ROS time
    
    rosgraph_msgs::Clock sclock_time;
    ros::Time timestamp_ros;  

    rosgraph_msgs::Clock wallclock_time;
    ros::Time timestamp_wall;

    ros::WallRate loop_rate(rate*factor);
    
    ros::Time zero_time = ros::Time::now(); // Initial Time
    double zero_nsecs = zero_time.toNSec(); 
    
    ROS_INFO_STREAM("ZERO SECS: "<<zero_time.toSec());
    ROS_INFO_STREAM("ZERO NSECS: "<<zero_time.toNSec());

    double elapsed_nsec = 0.0;

    


    while( ros::ok() )
    {
        elapsed_nsec = elapsed_nsec +  ((1.0/rate)*1e9);
        

        //ROS_INFO_STREAM("Elapsed Time = "<<elapsed_nsec);     
        
        timestamp_ros.fromNSec( elapsed_nsec + zero_nsecs); // Add scaled delta T to zero time

        ros::WallTime new_wtime = ros::WallTime::now();
        double wnsecs = new_wtime.toNSec();
        timestamp_wall.fromNSec(wnsecs);

        sclock_time.clock = timestamp_ros;
        sclock_pub.publish(sclock_time);

        wallclock_time.clock = timestamp_wall;
        wclock_pub.publish(wallclock_time);
        
        ros::spinOnce( );
        
        loop_rate.sleep( );
    }
    return EXIT_SUCCESS;

}


