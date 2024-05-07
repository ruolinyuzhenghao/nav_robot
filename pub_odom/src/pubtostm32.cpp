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
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ros/ros.h>
#include <netinet/in.h>
#include <arpa/inet.h>
/****************************************************************************/
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
#define Pi 3.1415926
/*****************************************************************************/
float ratio = 100.0f ;   //转速转换比例，执行速度调整比例
float D = 0.2680859f ;    //两轮间距，单位是m
float d_wheel=0.125f ;
float linear_temp=0,angular_temp=0;//暂存的线速度和角速度
serial::Serial my_serial;
/****************************************************/
unsigned char speed_data[8]={
	0xff,0x01,0x04,
	0x00,0x00,
	0x00,0x00,
	0x00};  
unsigned char run_data[8]={
	0xff,0x01,0x01,0x03,
	0x00,
	0x00,0x00,
	0x00};  
/************************************
              各项参数
*************************************/
string rec_buffer,rec_serial;  //串口数据接收变量
static double rovpose_x=0,rovpose_y=0;
static vector<float>pathpoints;
double WARN_DIS=1.98;
double dis=0;
int pose_j=0;
int last_pose_j=0;
double min=1000;
uint16_t boat_conner_count=0;//boat拐弯次数
uint16_t rov_conner_count=0;//rov拐弯次数
int ret=0;
int ss = socket(AF_INET, SOCK_STREAM, 0);
//发送给下位机的左右轮速度，里程计的坐标和方向
union floatData //union的作用为实现char数组和float之间的转换
{
    float d;
    unsigned char data[4];
}right_speed_data,left_speed_data,position_x,position_y,oriention,vel_linear,vel_angular;


/************************************
           订阅速度信息发给下危机
*************************************/
void callback(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
    //ROS_WARN_THROTTLE(2, "left speed =  speed=");   
    angular_temp = cmd_input.angular.z/5;//获取/cmd_vel的角速度,rad/s

    linear_temp = cmd_input.linear.x/5 ;//获取/cmd_vel的线速度.m/s
    
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
    my_serial.write(speed_data,8);
    
    speed_data[7] = 0;
}
/************************************
        解析串口数据之Rov转弯次数与Rov坐标
*************************************/
int unpack_count(unsigned char ucData,int i)
{
    int Bag_rovstation_Len = 13;
    static int8_t Bag1Buffer[250];//存储数据
    static int8_t Bag1Cnt = 0;	//计数存储数据
    static int8_t Bag1checkbit=0X00;
    Bag1Buffer[Bag1Cnt]=ucData;	//将收到的数据存入缓冲区中
    if (Bag1Buffer[0]!=1||Bag1Buffer[1]!=1||Bag1Buffer[2]!=1)
    {
       if(Bag1Cnt==2)
       {
           Bag1Cnt=0;//重新计数
           Bag1checkbit=0X00;
           return i-2;
       }
    }
    if (Bag1Cnt<Bag_rovstation_Len-1)
    {
       Bag1checkbit+=Bag1Buffer[Bag1Cnt];
       Bag1Cnt++;
       return i;
    }
    else
    {
    
       if(~(Bag1checkbit-Bag1Buffer[0])==Bag1Buffer[Bag1Cnt])
       {
       	rov_conner_count=uint8_t(Bag1Buffer[4])*256+uint8_t(Bag1Buffer[5]);
       	rovpose_x=uint16_t(uint8_t(Bag1Buffer[6])*256+uint8_t(Bag1Buffer[7]))/10;
       	rovpose_y=uint16_t(uint8_t(Bag1Buffer[8])*256+uint8_t(Bag1Buffer[9]))/10;
		ROS_INFO("rov_couner_count==%d",rov_conner_count);
		ROS_INFO("rov_pose_x==%f",rovpose_x);
		ROS_INFO("rov_pose_y==%f",rovpose_y);
       }
       Bag1Cnt=0;//清空缓存区
       Bag1checkbit=0X00;
       return i;
    }
}
/************************************
       订阅船的坐标数据与串口数据
功能一：定位到全覆盖路径规划的路径节点
功能二：计算船的转弯次数
功能三：解析串口数据，计算距离，判断转弯次数
*************************************/
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg )
{
    min=1000;
    ROS_INFO("boat_pose->x=%f",msg->pose.pose.position.x);
    ROS_INFO("boat_pose->y=%f",msg->pose.pose.position.y);
    float boat_pose_x=msg->pose.pose.position.x;
    float boat_pose_y=msg->pose.pose.position.y;
    double path_dis=0;

    for(int i=0;i<pathpoints.size()-1;i=i+2)
    {
    	path_dis=std::sqrt((pathpoints[i]-boat_pose_x)*(pathpoints[i]-boat_pose_x)+(pathpoints[i+1]-boat_pose_y)*(pathpoints[i+1]-boat_pose_y));
    	if(path_dis<min)
    	{
    		min=path_dis;
    		pose_j=i;
    	}
    	//ROS_INFO("%lf---->%lf",min,path_dis);
    }
    //ROS_INFO("path_num--->%d",pose_j);
    //计算船的转弯次数
    pose_j=pose_j+2;
    if(pathpoints[pose_j+1]!=pathpoints[pose_j-1]&&pose_j>3&&pathpoints[pose_j+1]==pathpoints[pose_j+3]&&last_pose_j!=pose_j)
    {
    	last_pose_j=pose_j;
	boat_conner_count++;
	ROS_INFO("boat is zhuanwaning,%dcouner_count--->%d",pose_j,boat_conner_count);
    }
    pose_j=pose_j-2;
    // 接收rov的转弯次数
    char buf[128] = {'\0'};
    ret = recv(ss, buf, 128, MSG_DONTWAIT); // 接收数据到buf,并获得接收的长度ret.
    if(ret>0)
    {
    	cout << "recv_data: " <<buf << endl;//打印接受到的数据
    	for(int i=0;i<128;i++)
    	{
    		i = unpack_count(static_cast<unsigned char>(buf[i]),i);
    	}
    
    }  
    dis = sqrt((rovpose_x-boat_pose_x)*(rovpose_x-boat_pose_x)+(rovpose_y-boat_pose_y)*(rovpose_y-boat_pose_y));
    ROS_INFO("dis========%f",dis);
}

int main(int argc, char **argv)
{
/************************************
            Rov初始化
*************************************/
    ros::init(argc, argv, "base_controller");//初始化串口节点
    string port("/dev/ttyUSB0");//小车串口号
    ros::param::get("~serial_port", port);
    unsigned long baud = 115200;//+小车串口波特率
    ros::NodeHandle n;  //定义节点进程句柄

/************************************
              打开串口
*************************************/
    bool check=true;
    /*
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
    */
/************************************
              链接TCP
*************************************/
	// 2. 链接服务端
	sockaddr_in addr; // 不建议使用sockaddr,建议用sockaddr_in
	addr.sin_port = htons(8765); // 网络字节序
	addr.sin_addr.s_addr  = inet_addr("192.168.101.225"); // 网络字节序
	addr.sin_family = AF_INET; // 地址族
	int len = sizeof(sockaddr_in);
	//判断是否连接成功
	if(connect(ss, (sockaddr*)&addr, len) == -1)
	{
		ROS_INFO("TCP connect error");
		return -1;
	}
	ROS_INFO("TCP connect error");

/************************************
              订阅话题
*************************************/
    ros::Subscriber sub = n.subscribe("cmd_vel", 20, callback); //订阅/cmd_vel主题
    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 20, pose_callback); //订阅/pose主题
/************************************
              获取路径节点
*************************************/
    std::fstream myfile("/home/rover/coverage_path.txt");
    if (!myfile.is_open())
    {
	cout << "can not open this file" << endl;
	return 0;
    }
    while(!myfile.eof())
    {
	float_t i=0;
	myfile>>i;
	pathpoints.push_back(i);
    }
/************************************
              发布指令
功能一：判断二者拐弯次数，发布等待命令
功能二：判断是否超过线长
[1]船下一个目标点会缩短距离-->船继续rov停下
[2]船下一个目标点不会i缩短距离-->船停下rov继续
[3]船和rov均正常前进
*************************************/
    ros::Publisher stop_info_pub = n.advertise<std_msgs::Bool>("/btn_press", 10);
    ros::Publisher conncet_info_pub = n.advertise<std_msgs::Bool>("/btn_press_continue", 10);
    ros::Publisher chatter_pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 10);
      
    int loop_times=100;
    ros::Rate loop_rate(loop_times);//设置周期休眠时间
    while(ros::ok())
    {
    if(boat_conner_count>rov_conner_count)
    {
    	//船停下
    	ROS_INFO("waiting for rov");
    	std_msgs::Bool mmsg;
        mmsg.data=false;
    	//stop_info_pub.publish(mmsg);
    }
    else if(boat_conner_count<rov_conner_count)
    {
    	//rov停下
    	run_data[4]=0x01;
    	for(int i=1;i<7;i++)
    	{
    		run_data[7]+=run_data[i];
    	}
    	run_data[7]=~run_data[7];
    	//写入数据到串口
    	ROS_INFO("runmodel=%d",run_data[4]);
	ret = send(ss, run_data, 8, 0);//向服务器发送数据
    	run_data[7]=0x00;
    }
    else
    {   
        if(dis>WARN_DIS)
        {		
        	std_msgs::Bool mmsg;
		double warn_dis = std::sqrt((pathpoints[pose_j+1]-rovpose_x)*(pathpoints[pose_j+1]-rovpose_x)+(pathpoints[pose_j+2]-rovpose_y)*(pathpoints[pose_j+2]-rovpose_y));
		ROS_INFO("WARNING DIS WARNING");
		if(warn_dis<dis&&min!=1000)
		{
			//boat继续
			mmsg.data=true;
    			conncet_info_pub.publish(mmsg);
			ROS_INFO("continue<-->forward%lf---->%lf",dis,warn_dis);

			//rov停下
    			run_data[4]=0x01;
    			for(int i=1;i<7;i++)
    			{
    				run_data[7]+=run_data[i];
    			}
    			run_data[7]=~run_data[7];
    			//写入数据到串口
    			ROS_INFO("runmodel=%d",run_data[4]);
			ret = send(ss, run_data, 8, 0);//向服务器发送数据
    			run_data[7]=0x00;
		}
		else
		{
        		mmsg.data=false;
    			stop_info_pub.publish(mmsg);
		}
        }
        else
        {
        	std_msgs::Bool mmsg;
        	mmsg.data=true;
    		conncet_info_pub.publish(mmsg);
    		run_data[4]=0x01;
    		for(int i=1;i<7;i++)
    		{
    			run_data[7]+=run_data[i];
    		}
    		run_data[7]=~run_data[7];
    		//写入数据到串口
    		ROS_INFO("runmodel=%d",run_data[3]);
		ret = send(ss, run_data, 8, 0);//向服务器发送数据
    		run_data[7]=0x00;
        
        }
        ros::spinOnce();//周期执行
	loop_rate.sleep();//周期休眠
    		    
    }
    }
  
    
    return 0;
}
