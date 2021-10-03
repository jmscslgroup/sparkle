#include<ros/ros.h>
#include<math.h>



// Convert ROLL PITCH YAW to QUETERNION
geometry_msgs::Quaternion rpyToQuaternion(double roll, double pitch, double yaw)
{
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);


    double w = cy * cp * cr + sy * sp * sr;
    double x = cy * cp * sr - sy * sp * cr;
    double y = sy * cp * sr + cy * sp * cr;
    double z = sy * cp * cr - cy * sp * sr;

    geometry_msgs::Quaternion quat;
    quat.x = x;
    quat.y = y;
    quat.z = z;
    quat.w = w;

    return quat;
}

void printmap(std::map<std::string, std::tuple<bool, geometry_msgs::Point, geometry_msgs::Quaternion, geometry_msgs::Twist>> states)
{
    for(const auto& elem : states)
    {
        std::tuple<bool, geometry_msgs::Point, geometry_msgs::Quaternion, geometry_msgs::Twist> second;
        second = elem.second;
        ROS_INFO_STREAM("key = "<< elem.first<<"status = "<<std::get<0>(second) );
    }

}
