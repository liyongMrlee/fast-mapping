#include "fast_mapping/fast_mapping.h"
#include <ros/ros.h>


int main(int argc,char** argv)
{
   ros::init(argc, argv, "jps_node");
   
   Fast_Mapping jps;

   ros::spin();

}