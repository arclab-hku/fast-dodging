#include "ros/ros.h"
#include "SortRos.h"



int main (int argc, char** argv) 
{

	ros::init (argc, argv, "sort_ros");
    ros::NodeHandle nh("~");

    SortRos sortros(nh);

	ros::spin();
    return 0;
    
}