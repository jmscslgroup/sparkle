#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<cstdio>
#include<cstdlib>
#include<ctime>
#include<iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SparkleSetVel");
    
    ros::NodeHandle nodeHandle("");

    double velx  = 0.0; //Linear X velocity
    double angularz = 0.0; //Angular Z steering Angle

    ros::Publisher vel_pub = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::Rate loop_rate(60);
    
    geometry_msgs::Twist velMsg;

    while(ros::ok())
    {
        nodeHandle.param("constVel", velx, 5.0);
        nodeHandle.param("strAngle", angularz,  0.0);
        velMsg.linear.x = velx;
        velMsg.angular.z = angularz;
        
        vel_pub.publish(velMsg);
        ros::spinOnce();
        loop_rate.sleep();
        
    }

    return EXIT_SUCCESS;
    
}
