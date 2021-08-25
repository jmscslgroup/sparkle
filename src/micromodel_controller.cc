#include "sparkle/controller.hh"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "micromodel_controller");
	ros::NodeHandle nh;
	
	MicromodelController micromodel = MicromodelController(&nh);
	
	
	ros::Rate rate(20);
  	ros::Duration(1.0).sleep();
  	while (ros::ok())
	{
    		ros::spinOnce();
    		micromodel.publish();
    		rate.sleep();
  	}
	
  	return 0;

}	
