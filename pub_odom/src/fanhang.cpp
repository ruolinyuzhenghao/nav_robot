#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h> //move_base的操作规范，这是一个将高级别接口暴露在导航堆栈中的 ROS 操作。
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <fstream>

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "fanhang");//初始化导航节点
  double dianya=0;
  
  MoveBaseClient ac("move_base", true); 
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  } 
  move_base_msgs::MoveBaseGoal goal;
  
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0;
  goal.target_pose.pose.position.y = 0;
  goal.target_pose.pose.orientation.w = 1.0;

  fstream myfile("/home/rover/dianya.txt");
  if (!myfile.is_open())
  {
	cout << "can not open this file" << endl;
	return 0;
  }
  while(!myfile.eof())
  {
	myfile>>dianya;
  }

  while(ros::ok())
  {
  if(dianya<22)
  {
  ROS_INFO("Sending goal");
  	ac.sendGoal(goal); 

  	ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    	ROS_INFO("Hooray, the base moved to the goal");
  else
    	ROS_INFO("The base failed to move to the goal");
  ros::spinOnce();//周期执行


  	 
  }
  
  return 0;
}


}
