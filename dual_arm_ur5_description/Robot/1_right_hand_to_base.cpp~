#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_move_group");
  ros::NodeHandle node_handle;  
 
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  moveit::planning_interface::MoveGroup group("right_arm");
  //group.setPlannerId("RRTkConfigDefault");
  Eigen::Affine3d pose = Eigen::Translation3d(0.469, -0.451, 0.307)
                         * Eigen::Quaterniond(0.002, 0.704, 0.710, 0.020);
  group.setPoseTarget(pose);
  
   ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);
  group.move();
  sleep(2.0);
  

  

}

