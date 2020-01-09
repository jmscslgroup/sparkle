/*-----------------------------------------------------------
Author: Rahul Bhadani
Initial Date: January 2020

This class implements a Gazebo sensor plugin that calculates
the minimum distance of all the obstacle among all the laser
points from front laser points 

-------------------------------------------------------------*/

#include<ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <tf/transform_broadcaster.h>

#include <ignition/math/Box.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo_plugins/gazebo_ros_laser.h>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo_plugins/PubQueue.h>
 
#include <iostream>
#include <cstdlib>
#include <string>
using namespace std;

namespace gazebo
{
    class distance: public SensorPlugin
    {
        private:

            physics::WorldPtr world; // Point to the World
            string distance_topic; // distance topic that will publish the minimum distance
            string angle_topic; // distance topic that will publish the angle of minimum distance
            
            string ns; // The Namespace

            ros::Publisher distance_publisher; // ROS publisher for minimum distance information
            ros::Publisher angle_publisher; // ROS publisher for angle of minimum distance information
            ros::NodeHandle* rosnode; // ROS Node
            sensor_msgs::LaserScan laser_data; // stores received laser scan data
            std_msgs::Float64 minimum_distance; // saves minimum distance found
            std_msgs::Float64 angle_of_min_distance; //saved the angle of minimum distance
            double angle_min;
            double angle_max;
            
            sensors::RaySensorPtr parent_sensor;
            event::ConnectionPtr new_laserscan_connection;
            
            gazebo::transport::NodePtr gazebo_node;
            gazebo::transport::SubscriberPtr laser_scan_sub;

        public:
        
            distance()
            {
                this->ns = "";
                this->distance_topic = this->ns + "/distance/dist";
                this->angle_topic = this->ns + "/distance/angle";
                this->angle_min = -M_PI;
                this->angle_max = M_PI;
            }

            void OnNewScan(ConstLaserScanStampedPtr &_msg)
            {
                vector<double> allranges;
                this->parent_sensor->Ranges(allranges);
                
                int raycount = this->parent_sensor->RayCount();
                
                // Get the minimum distance
                
                double min_dist = this->parent_sensor->RangeMax();
                double range_min = this->parent_sensor->RangeMin();
                
                double min_angle = this->parent_sensor->AngleMin()();
                
                double angle_tmp = min_angle;
                double angle_incr = this->parent_sensor->AngleResolution();
                
                for(vector<double>::const_iterator it = allranges.begin(); 
                    it != allranges.end(); it++, angle_tmp += angle_incr)
                {
                    if(min_dist > *it && *it > range_min 
                        && angle_tmp > this->angle_min 
                        && angle_tmp < this->angle_max
                        )
                        {
                            min_dist = *it;
                            min_angle = angle_tmp;
                        }
                }
                this->minimum_distance.data = min_dist;
                this->angle_of_min_distance.data = min_angle;
                
                this->distance_publisher.publish(this->minimum_distance);
                this->angle_publisher.publish(this->angle_of_min_distance);
                //this->distance_queue->push(this->minimum_distance, this->distance_publisher);
                //this->angle_queue->push(this->angle_of_min_distance, this->angle_publisher);
            }
            
            void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
            {
                

                ROS_INFO_STREAM("Minimum distance estimator SensorPlugin Loaded.");
                
                this->gazebo_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
                this->gazebo_node->Init("default");
                
                // Get the name of the parent sensor
                GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
                this->parent_sensor = dynamic_pointer_cast<sensors::RaySensor>(parent);
                
                if(!this->parent_sensor)
                {
                    gzthrow("[distance SensorPlugin] distance plugin requires a Ray Sensor as its parent");
                }
                    
                this->world = physics::get_world(this->parent_sensor->WorldName()); 
                string grandparent = this->parent_sensor->ParentName();
                size_t pos = grandparent.find("::");
                this->ns = "/" + grandparent.substr (0, pos); 
                ROS_INFO_STREAM("[distance SensorPlugin] Namespace retrieved is: "<<this->ns);
                this->distance_topic = this->ns + "/distance/dist";
                this->angle_topic = this->ns + "/distance/angle";
    
                
                if(sdf->HasElement("angleMin"))
                {
                    this->angle_min = sdf->GetElement("angleMin")->Get<double>();
                }
                else
                {
                    gzwarn << "[distance SensorPlugin] Using default minimum angle: " << this->angle_min << "\n";
                }
                if(sdf->HasElement("angleMax"))
                {
                    this->angle_max = sdf->GetElement("angleMax")->Get<double>();
                }
                else
                {
                    gzwarn << "[distance SensorPlugin] Using default maximum angle: " << this->angle_max << "\n";
                }
                              
                ROS_INFO_STREAM("[distance SensorPlugin] Plugin's parent name: "<<this->parent_sensor->Name());
                ROS_INFO_STREAM("[distance SensorPlugin] Plugin's granparent: "<<this->parent_sensor->ParentName() );
                ROS_INFO_STREAM("[distance SensorPlugin] Plugin's full scope name: "<<this->parent_sensor->ScopedName() );
                this->rosnode = new ros::NodeHandle(this->ns+"/distance");
                this->distance_publisher = this->rosnode->advertise<std_msgs::Float64>(this->distance_topic, 1);
                this->angle_publisher = this->rosnode->advertise<std_msgs::Float64>(this->angle_topic, 1);
                 
                //this->new_laserscan_connection = this->parent_sensor->LaserShape()->ConnectNewLaserScans(boost::bind(&distance::OnNewScan, this));
                
                // Subscribe Gazebo-topic and work on laser data and produce minimum of all distances.
                this->laser_scan_sub = this->gazebo_node->Subscribe(this->parent_sensor->Topic(), &distance::OnNewScan, this);

            }
        };
        GZ_REGISTER_SENSOR_PLUGIN(distance)
}

