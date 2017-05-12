/**
 * \ref xtion_main.cpp
 *
 *  \date 24/giu/2015
 *  \author Alessio Levratti
 *  \version 1.0
 *  \bug
 *  \copyright GNU Public License.
 */

#include "skeleton_tracker/xtion_tracker.hpp"
#include <ros/ros.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "skeleton_tracker");
  ros::NodeHandle n;
  

  xtion_tracker* skeleton_tracker = new xtion_tracker();
  

  ros::ServiceServer srv = n.advertiseService("skeleton_service" , &xtion_tracker::servicecallback, skeleton_tracker);
  while (ros::ok())
  {
  	skeleton_tracker->spinner();
  	ros::spinOnce();
    
  }
  //ros::spin();

  delete skeleton_tracker;

  return 1;

}


