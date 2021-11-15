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
    ros::Publisher ros_clock_pub = n.advertise<rosgraph_msgs::Clock>("/ros_clock", 1); // To publish actual ROS time
    rosgraph_msgs::Clock sclock_time;
  //  rosgraph_msgs::Clock ros_clock_time;
    ros::Time timestamp_ros;
  //  ros::Time timestamp_ros_actual;
    //ros::Rate loop_rate(rate*factor);
    ros::WallRate loop_rate(rate*factor);
    
    ros::Time zero_time = ros::Time::now(); // Initial Time
    double zero_nsecs = zero_time.toNSec(); 
    
    ROS_INFO_STREAM("ZERO SECS: "<<zero_time.toSec());
    ROS_INFO_STREAM("ZERO NSECS: "<<zero_time.toNSec());

    double elapsed_nsec = 0.0;

    


    while( ros::ok() )
    {
        //ros::Time new_time = ros::Time::now();
        
        //ros::Duration diff = new_time - zero_time ; // Time duration passed

        // elapsed_nsec = diff.toSec();
        
        // toSec and toNSec returns same values, just in different units, former one in seconds, latter one in nanoseconds: No Need to Add toSec and toNSec
        // ROS_INFO("Seconds passed since last call: %10.10f ", elapsed_nsec);
        
        //elapsed_nsec = diff.toNSec();
        
        //ROS_INFO("NANOSeconds passed since last call: %10.10f ", nanoSecs);
        
        elapsed_nsec = elapsed_nsec +  ((1.0/rate)*1e9);
        ROS_INFO_STREAM("Elapsed Time = "<<elapsed_nsec);     
        //elapsed_nsec = elapsed_nsec/factor; // IF we want real time factor to be 2x faster, then we divide by factor if factor = 2
        
        timestamp_ros.fromNSec( elapsed_nsec + zero_nsecs); // Add scaled delta T to zero time

        sclock_time.clock = timestamp_ros;
        
 //       timestamp_ros_actual.fromNSec(new_time.toNSec());
        
    //    ros_clock_time.clock = timestamp_ros_actual;
        //ros_clock_time.clock = new_time;
        
        sclock_pub.publish(sclock_time);
        
  //      ros_clock_pub.publish(ros_clock_time);
        
        ros::spinOnce( );
        
        loop_rate.sleep( );
    }
    return EXIT_SUCCESS;

}


