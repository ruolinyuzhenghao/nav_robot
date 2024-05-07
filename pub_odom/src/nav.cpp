#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "actionlib_msgs/GoalID.h"
#include "move_base_msgs/MoveBaseActionGoal.h"


/************************************
              各项参数
*************************************/
float pose_x=0;//姿态
float pose_y=0;
float goal_x=0;//目标点
float goal_y=0;
/************************************
              订阅姿态
*************************************/
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr & msg )
{
    //ROS_INFO("pose->x=%f",msg->pose.position.x);
    //ROS_INFO("pose->y=%f",msg->pose.position.y);
    pose_x=msg->pose.position.x;
    pose_y=msg->pose.position.y;
}
/************************************
              订阅目标点
*************************************/
void goal_callback(const move_base_msgs::MoveBaseActionGoal::ConstPtr & msg )
{
    ROS_INFO("goal_pose->x=%f",msg->goal.target_pose.pose.position.x);
    ROS_INFO("goal_pose->y=%f",msg->goal.target_pose.pose.position.y);
    goal_x=msg->goal.target_pose.pose.position.x;
    goal_y=msg->goal.target_pose.pose.position.y;
}
using namespace std;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "nav_controller");//初始化导航节点
	ros::NodeHandle n;  //定义节点进程句柄
	
	ros::Subscriber pose_sub = n.subscribe("/tracked_pose", 10, pose_callback); //订阅/pose主题
	ros::Subscriber goal_sub = n.subscribe("/move_base/goal", 10, goal_callback); //订阅/pose主题
	
	
	int loop_times=100;
    	ros::Rate loop_rate(loop_times);//设置周期休眠时间
	while(ros::ok())
    	{
        	ros::spinOnce();//周期执行
		loop_rate.sleep();//周期休眠 
    	}
    
	


}
