/*******************************************************************/
#include "ros/ros.h" 
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <string>        
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include "serial/serial.h"
#include <math.h>
#include "std_msgs/Bool.h"
#include "actionlib_msgs/GoalID.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include <move_base_msgs/MoveBaseAction.h> //move_base的操作规范，这是一个将高级别接口暴露在导航堆栈中的 ROS 操作。
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>//amcl
/****************************************************************************/
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
#define Pi 3.1415926
/*****************************************************************************/
float ratio = 10000.0f ;   //转速转换比例，执行速度调整比例
float D = 0.2680859f ;    //两轮间距，单位是m
float d_wheel=0.125f ;
float linear_temp=0,angular_temp=0;//暂存的线速度和角速度
serial::Serial my_serial;
double last_a=0.0;
double last_v=0.0;
double max_a=-1000;
/****************************************************/
unsigned char speed_data[8]={
	0xff,0x01,0x04,
	0x00,0x00,
	0x00,0x00,
	0x00};  
unsigned char run_data[8]={
	0xff,0x01,0x01,
	0x00,
	0x00};
unsigned char count_data[8]={
	0xff,0x01,0x01,
	0x00,
	0x00};   
/************************************
              各项参数
*************************************/
float pose_x=0;//姿态
float pose_y=0;
float goal_x=0;//目标点
float goal_y=0;
string rec_buffer,rec_serial;  //串口数据接收变量
double eps=0.35;
double dis=0;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
float bili=1;
bool shache=false;
//发送给下位机的左右轮速度，里程计的坐标和方向
union floatData //union的作用为实现char数组和float之间的转换
{
    float d;
    unsigned char data[4];
}right_speed_data,left_speed_data,position_x,position_y,oriention,vel_linear,vel_angular;

/*******************************cmd_vel/发布速度*****************************/
void callback(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
    
    angular_temp = cmd_input.angular.z;//获取/cmd_vel的角速度,rad/s
    linear_temp = cmd_input.linear.x ;//获取/cmd_vel的线速度.m/s
    

    //ROS_INFO("bili=%f-->linear=%f",bili,linear_temp);
    //ROS_INFO("bili=%f-->angel=%f",bili,angular_temp);
    
    int16_t angular_tempp=int16_t(angular_temp*ratio);
    int16_t linear_tempp=int16_t(linear_temp*ratio);
    
    speed_data[3] = ( char)(linear_tempp>>8);
    speed_data[4] = ( char)(linear_tempp);
    
    speed_data[5] = ( char)(angular_tempp>>8);
    speed_data[6] = ( char)angular_tempp;
   // speed_data[3] = 0x00;
   // speed_data[4] = 0x00;
    
    //speed_data[5] = 0x00;
    //speed_data[6] = 0x00;
    
    for(int i=1;i<7;i++)
    {
    	speed_data[7]+=speed_data[i];
    }
    speed_data[7]=~speed_data[7];
    //写入数据到串口
    //ROS_INFO("last= v_v%f-->new=%f",last_v,linear_temp);
    //ROS_INFO("last v_a=%f-->new=%f",last_a,angular_temp);
    ros::Rate loopp_rate(10);//设置周期休眠时间
    if(last_a!=angular_temp||last_v!=linear_temp)
    {
    	if((last_a!=0&&angular_temp==0)||(linear_temp==0&&last_v!=0))
    	{
    		for(int i=0;i<10;i++)
    		{
    			if(i<9&&shache)
    			{    
    				angular_tempp=(3200-9/std::abs(last_a))*last_a/std::abs(last_a);
    				linear_tempp=(2200-9/std::abs(last_v))*last_v/std::abs(last_v);
    
    				speed_data[3] = ( char)(linear_tempp>>8);
    				speed_data[4] = ( char)(linear_tempp);
    
    				speed_data[5] = ( char)(angular_tempp>>8);
    				speed_data[6] = ( char)angular_tempp;
    				speed_data[7] = 0;
    				for(int i=1;i<7;i++)
    				{
    					speed_data[7]+=speed_data[i];
    				}
    				speed_data[7]=~speed_data[7];
    				if(i==8)
    				shache=false;
    				

    				
       			my_serial.write(speed_data,8);
       			ROS_INFO("last%f-->last_a=%d",last_a,angular_tempp);
    				ROS_INFO("last%f-->last_v=%d",last_v,linear_tempp);
    				loopp_rate.sleep();//周期休眠 
    					
    			}
   			else
    			{
    			
    				angular_tempp=0;
    				linear_tempp=0;
    
    				speed_data[3] = ( char)(linear_tempp>>8);
    				speed_data[4] = ( char)(linear_tempp);
    
    				speed_data[5] = ( char)(angular_tempp>>8);
    				speed_data[6] = ( char)angular_tempp;
    				speed_data[7] = 0;
    				for(int i=1;i<7;i++)
    				{
    					speed_data[7]+=speed_data[i];
    				}
    				speed_data[7]=~speed_data[7];
    			
    				my_serial.write(speed_data,8);
    				ROS_INFO("last%f-->mmmmmmangel=%d",last_a,angular_tempp);
    				ROS_INFO("last%f-->mmmmmmlinear=%d",last_v,linear_tempp);
    			}
    		}
    	}
    	else
    	{
    		my_serial.write(speed_data,8);
    		ROS_INFO("angel=%d",angular_tempp);
    		ROS_INFO("linear=%d",linear_tempp);
    	}     
    
    }
    //ROS_INFO("linear=%d",linear_tempp);
    //ROS_INFO("angel=%d",angular_tempp);
    
    speed_data[7] = 0;
    last_a=angular_temp;
    last_v=linear_temp;
}
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



int main(int argc, char **argv)
{
/*******************************ros初始化*****************************/
    ros::init(argc, argv, "base_controller");//初始化串口节点
    string port("/dev/ttyUSB0");//小车串口号
    ros::param::get("~serial_port", port);
    unsigned long baud = 115200;//+小车串口波特率
    ros::NodeHandle n;  //定义节点进程句柄
    
    try
    {
	 my_serial.setPort(port);
	 my_serial.setBaudrate(baud);
	 serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
         my_serial.setTimeout(to); 
         my_serial.open(); 
    }
    catch (serial::IOException& e)
    {
	ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }
    //检测串口是否已经打开，并给出提示信息 
    if(my_serial.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 

/*******************************ros订阅话题*****************************/
    ros::Subscriber sub = n.subscribe("cmd_vel", 20, callback); //订阅/cmd_vel主题
    ros::Subscriber goal_sub = n.subscribe("/move_base/goal", 10, goal_callback); //订阅/pose主题
    ros::Subscriber pose_sub = n.subscribe("/tracked_pose", 10, pose_callback); //订阅/pose主题
    ros::Publisher chatter_pub = n.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
    actionlib_msgs::GoalID empty_goal;
    MoveBaseClient mv_base_client("/move_base", true);
    int loop_times=10;
    ros::Rate loop_rate(loop_times);//设置周期休眠时间
     
    while(ros::ok())
    {
        
        dis = sqrt((pose_x-goal_x)*(pose_x-goal_x)+(pose_y-goal_y)*(pose_y-goal_y));
        
        if(0)
        {
	//OS_INFO("%f>>current goal could be reached.<<%f",dis,eps);
		chatter_pub.publish(empty_goal);
		shache=true;
        }  	
        ros::spinOnce();//周期执行	
        loop_rate.sleep();//周期休眠    
    }
    
  
    
    return 0;
}
