#include<iostream>
#include<ros/ros.h>
#include<ctime>
#include<cstdio>
#include<cstdlib>
#include<stdlib.h>
#include<pwd.h>
#include<stdio.h>
#include<sys/types.h>
#include<string>
#include<vector>
#include<math.h>

#include<std_msgs/String.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Point.h>
#include<ignition/math/Vector3.hh>

using namespace std;

ros::Time lastUpdate;

bool newMessage = false;

string ns;

nav_msgs::Odometry newOdom;
geometry_msgs::Twist newVel;

double x_init, y_init, psi_init, wheelBase, str_init;

//Old vars
double x_old;
double y_old;
double yaw_old;
double str_old;

// This is for all director params
//std::map<std::string, bool> director_params;

// This is for this vehicle
//bool this_director;

double xdot;
double ydot;
double x, y, str, yaw;

double u2;

// This very simple callback looks through the data array, and then
// returns the value (not the index) of that distance
void velcallback( const geometry_msgs::Twist::ConstPtr& vel )
{
    double LinearX = vel->linear.x; 
    double AngularZ = vel->angular.z;

    ros::Time current_time = ros::Time::now();
    
    /* lastUpdate is initialized to zero because time is not initialized until first message has been received */
    if(lastUpdate.toSec() == 0)
    {
        lastUpdate = current_time;
        return;
    }
    ros::Duration duration = current_time - lastUpdate;

    lastUpdate = current_time;
    double dT = duration.toSec() + (duration.toNSec()*1e-9);
    
    //ROS_INFO_STREAM("ns: " << ns <<", dT: "<<dT<<endl);
    u2 = (AngularZ - str_old)/dT;

    //ROS_INFO_STREAM("NS: "<<ns<<", LinearX: "<<LinearX<<", AngularZ " << AngularZ<<endl);
    //ROS_INFO_STREAM("NS: "<<ns<<", STR OLD: "<<str_old<<", YAW OLD: "<<yaw_old<<endl);
    xdot = LinearX*cos(str_old)*cos(yaw_old);
    ydot = LinearX*cos(str_old)*sin(yaw_old);
    x = x_old + dT*xdot;
    y = y_old + dT*ydot;
    

    str = str_old + dT*u2;

    if(str> 0.5236)
    {
        str = 0.5236;
    }
    else if(str< -0.5236)
    {
        str = -0.5236;
    }

    yaw = yaw_old + dT*LinearX*(1.0/wheelBase)*sin(str_old);
    while(yaw < 0)
    {
        yaw = yaw + 2*M_PI;
    }
    while(yaw > 2*M_PI)
    {
        yaw = yaw - 2*M_PI;
    }

    //ROS_INFO_STREAM("NS: "<<ns<<", xdot: "<<xdot<<", ydot: " <<ydot <<", yaw: "<<yaw<<endl);

    newVel.linear.x = xdot;
    newVel.linear.y = ydot;
    newVel.angular.z = yaw;


    newOdom.header.stamp = current_time;
    newOdom.header.frame_id = ns + "/odom";
    
    // we calculate our index in the pose msg by
    newOdom.child_frame_id = ns + "/base_link";
    newOdom.pose.pose.position.x = x;
    newOdom.pose.pose.position.y = y;
    newOdom.pose.pose.position.z = 0.006517;

    newOdom.pose.pose.orientation.x = 0.0;
    newOdom.pose.pose.orientation.y = 0.0;
    newOdom.pose.pose.orientation.z = yaw;
    newOdom.pose.pose.orientation.w = 0.0;

    newOdom.twist.twist.linear.x = xdot;
    newOdom.twist.twist.linear.y = ydot;
    newOdom.twist.twist.linear.z = 0.0;

    
    newMessage = true;


    lastUpdate = current_time;
    x_old = x;
    y_old = y;
    yaw_old = yaw;
    str_old = str;

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "bicycle");
    ros::NodeHandle n("");
    lastUpdate = ros::Time();
    ROS_INFO_STREAM(" ######## Last Update Time is "<<lastUpdate.toSec());
    
    double updateRate;
    //map<string, bool>::iterator it;

    n.param("x_init", x_init, 0.0);
    n.param("y_init", y_init, 0.0);
    n.param("psi_init", psi_init, 0.0);
    n.param("str_init", str_init, 0.0);
    n.param("wheelBase", wheelBase, 2.70); //2.7m is wheelbase of toyota prius
    n.param("updateRate", updateRate, 2.0);

    x_old = x_init;
    y_old = y_init;
    yaw_old = psi_init;
    str_old = str_init;


    //n.getParam("/director", director_params);
    
    
    ns = ros::this_node::getNamespace();
    ROS_INFO_STREAM("Node namespace is " << ns );
    ROS_INFO_STREAM("Node name is " << ros::this_node::getName( ) );

    ros::Publisher setvel_pub = n.advertise<nav_msgs::Odometry>("setvel", 1);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("vel", 1);

    // we also want to subscribe to the signaller
    ros::Subscriber sub = n.subscribe("cmd_vel", 1, &velcallback);


    ros::Rate loop_rate(100);
    newMessage = false;
    
    ros::Time previousTime = ros::Time::now();
    ros::Time newTime = ros::Time::now();
    
    ros::Duration duration = newTime - previousTime;
    double dT;

    while(ros::ok())
    {
        if(newMessage)
        {
            newTime = ros::Time::now();
        
            duration = newTime - previousTime;
            dT = duration.toSec() + (duration.toNSec()*1e-9);
           
            //n.setParam("/director" + ns, true);            
            //n.getParam("/director", director_params);
            //int n_vehicles = director_params.size();
            //int sum = 0;
            //for ( it = director_params.begin(); it != director_params.end(); it++ )
            //{
            //    sum = sum + it->second;
            //}

        
            if(dT >= 1.0/updateRate)
            {
                //if (sum == n_vehicles)
                //{
                //n.setParam("/director" + ns, true);

                /*while(1)
                {
                    n.getParam("/director", director_params);
                    int n_vehicles = director_params.size();
                    int sum = 0;
                    for ( it = director_params.begin(); it != director_params.end(); it++ )
                    {
                        sum = sum + it->second;
                    }
                    if(sum == n_vehicles)
                        break;

                }*/
                //ROS_INFO_STREAM("Published state to "<<ns << " at time "<< ros::Time::now());
                setvel_pub.publish(newOdom);
                vel_pub.publish(newVel);
                previousTime = newTime;
                //}
            }
            //n.setParam("/director" + ns, false);
            newMessage = false;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return EXIT_SUCCESS;


}
