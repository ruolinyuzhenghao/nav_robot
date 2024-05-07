#include <math.h>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "move_base_msgs/MoveBaseActionGoal.h"
#include <move_base_msgs/MoveBaseAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include "actionlib_msgs/GoalID.h"
#include "geometry_msgs/Twist.h"
#include <tf2/LinearMath/Quaternion.h>

static const char WINDOW[] = "Image window";
using namespace std;
using namespace cv;
using namespace cv::dnn;


/*****各项参数*********/
static double pose_x=0,pose_y=0;
bool pub_zero_vel=false;
/*
int flag=0;
bool flag0=true;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
class MoveControl{
private:
	MoveBaseClient ac;
public:
	MoveControl();
	void move();
	void SimpleDoneCallback(const actionlib::SimpleClientGoalState& state,
	const move_base_msgs::MoveBaseResultConstPtr& result);
	void SimpleFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
	void SimpleActiveCallBack();
};
MoveControl::MoveControl():ac("/move_base", true)
{
	ROS_INFO("Wating the Action server to start");
	ac.waitForServer();
	ROS_INFO("Done");
}
void MoveControl::move()
{
 	


	move_base_msgs::MoveBaseGoal goal;
	if(flag==0)
	{
	goal.target_pose.header.frame_id = "map"; 
	goal.target_pose.header.stamp = ros::Time::now(); 
	goal.target_pose.pose.position.x = 0; 
	goal.target_pose.pose.position.y = 0; 
	goal.target_pose.pose.position.z = 0; 
	goal.target_pose.pose.orientation.x = 0;
	goal.target_pose.pose.orientation.y = 0;
	goal.target_pose.pose.orientation.z = 0;
	goal.target_pose.pose.orientation.w = 1; 
	}  
	else if(flag==1)
	{ 
	goal.target_pose.header.frame_id = "map"; 
	goal.target_pose.header.stamp = ros::Time::now(); 
	goal.target_pose.pose.position.x = 1.67; 
	goal.target_pose.pose.position.y = 0; 
	goal.target_pose.pose.position.z = 0; 
	goal.target_pose.pose.orientation.x = 0;
	goal.target_pose.pose.orientation.y = 0;
	goal.target_pose.pose.orientation.z = 0;
	goal.target_pose.pose.orientation.w = 1; 
	}
	else if(flag==2)
	{ 
	goal.target_pose.header.frame_id = "map"; 
	goal.target_pose.header.stamp = ros::Time::now(); 
	goal.target_pose.pose.position.x = 3.70; 
	goal.target_pose.pose.position.y = 0.12; 
	goal.target_pose.pose.position.z = 0; 
	goal.target_pose.pose.orientation.x = 0;
	goal.target_pose.pose.orientation.y = 0;
	goal.target_pose.pose.orientation.z = 0;
	goal.target_pose.pose.orientation.w = 1; 
	}
	else if(flag==3)
	{ 
	goal.target_pose.header.frame_id = "map"; 
	goal.target_pose.header.stamp = ros::Time::now(); 
	goal.target_pose.pose.position.x = 3.90; 
	goal.target_pose.pose.position.y = 1.28; 
	goal.target_pose.pose.position.z = 0; 
	goal.target_pose.pose.orientation.x = 0;
	goal.target_pose.pose.orientation.y = 0;
	goal.target_pose.pose.orientation.z = 0;
	goal.target_pose.pose.orientation.w = 1; 
	}
	else if(flag==4)
	{ 
	goal.target_pose.header.frame_id = "map"; 
	goal.target_pose.header.stamp = ros::Time::now(); 
	goal.target_pose.pose.position.x = 2.19; 
	goal.target_pose.pose.position.y = 1.82; 
	goal.target_pose.pose.position.z = 0; 
	goal.target_pose.pose.orientation.x = 0;
	goal.target_pose.pose.orientation.y = 0;
	goal.target_pose.pose.orientation.z = 0;
	goal.target_pose.pose.orientation.w = 1; 
	}
	else
	{
	flag=0;
	goal.target_pose.header.frame_id = "map"; 
	goal.target_pose.header.stamp = ros::Time::now(); 
	goal.target_pose.pose.position.x = 0.20; 
	goal.target_pose.pose.position.y = 1.70; 
	goal.target_pose.pose.position.z = 0; 
	goal.target_pose.pose.orientation.x = 0;
	goal.target_pose.pose.orientation.y = 0;
	goal.target_pose.pose.orientation.z = 0;
	goal.target_pose.pose.orientation.w = 1; 

	}
	if(flag0)
	{
	ac.sendGoal(goal,
		boost::bind(&MoveControl::SimpleDoneCallback,this,_1,_2),
		boost::bind(&MoveControl::SimpleActiveCallBack,this),
		boost::bind(&MoveControl::SimpleFeedbackCallback,this,_1)
		);
	flag0=false;
	}
}
void MoveControl::SimpleDoneCallback(const actionlib::SimpleClientGoalState& state,
const move_base_msgs::MoveBaseResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s] ", state.toString().c_str());
	flag++;
	flag0=true;
}
void MoveControl::SimpleFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){
}

void MoveControl::SimpleActiveCallBack(){
}
*/
void pose_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ROS_INFO("boat_pose->x=%f",msg->pose.pose.position.x);
    ROS_INFO("boat_pose->y=%f",msg->pose.pose.position.y);
    pose_x=msg->pose.pose.position.x;
    pose_y=msg->pose.pose.position.y;

}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char *argv[])
{   
/*
    std::fstream myfile("/home/rover/xunhang.txt");
    if (!myfile.is_open())
    {
	cout << "can not open this file" << endl;
	return 0;
    }
    while(!myfile.eof())
    {
	std::string i;
	myfile>>i;
	std::cout<<i;
    }
*/

    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, 0, 0 );  // Create this quaternion from roll/pitch/yaw (in radians)
    ROS_INFO("%f  %f  %f  %f" ,myQuaternion.x(),myQuaternion.y(),myQuaternion.z(),myQuaternion.w());
    // ROS节点初始化
    ros::init(argc, argv, "xunhang_publisher");

    // 创建节点句柄
    ros::NodeHandle n;
    
    //MoveControl moveControl;

    // 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
    ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, pose_Callback);
    ros::Publisher chatter_pub = n.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
    geometry_msgs::Twist vel_msg;
    actionlib_msgs::GoalID empty_goal;
    /*
    MoveBaseClient ac("move_base", true);
    move_base_msgs::MoveBaseGoal goal;
    while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    } //等待操作服务器报告它已出现并准备开始处理目标
    */
    std_msgs::Bool msg;
    bool ifdetect=true;
    
    cv::VideoCapture cap(0);
    cap.set(CAP_PROP_FRAME_WIDTH,1280);
    cap.set(CAP_PROP_FRAME_HEIGHT,480);
    Mat input_image,frame,inputt_image ;
    //Reading an image from the file
    String yolo_cfg = "/home/rover/darknet-master/cfg/yolov4-custom.cfg";
    String yolo_widght = "/home/rover/darknet-master/backup/yolov4-custom_2000.weights";
    String yolo_class = "/home/rover/darknet-master/backup/classes.txt";
    const bool debug = true;

    //加载模型
    Net cnn_net = cv::dnn::readNetFromDarknet(yolo_cfg,yolo_widght);
    //使用GPU加速
    cnn_net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    cnn_net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    //获取识别类型
    vector<String> outNames = cnn_net.getUnconnectedOutLayersNames();
    if(debug)
        for (size_t i = 0; i < outNames.size(); i++)
        {
            cout<<"output layer name : "<<outNames[i].c_str()<<endl;
        }
	

    //read classes names
    vector<string> classNamesVec;
    ifstream classNamesFile(yolo_class);
    if (classNamesFile.is_open())
    {
        string className = "";
        while (getline(classNamesFile, className))
            classNamesVec.push_back(className);
    }


    //加载需要识别的图片
    while (ros::ok())
    {
        cap>>frame;
        //inputt_image=getimage(frame);
        inputt_image=frame(Range(0,480),Range(640,1280));
        flip(inputt_image,input_image,-1);//-1代表垂直和水平方向同时旋转，即沿X和Y轴翻转
        Mat inputBlob = blobFromImage(input_image, 1 / 255.F, Size(416,416), Scalar(), true, false);//转化图片形式，size为训练时输入图片大小，（cfg文件查看）
        cnn_net.setInput(inputBlob);//输入图片

        // 开始检测
        vector<Mat> outs;
        cnn_net.forward(outs, outNames); //向前推理，经过神经网络输出

        vector<double> layersTimings;
        vector<Rect> boxes;
        vector<int> classIds;
        vector<float> confidences;
        for (size_t i = 0; i<outs.size(); ++i)
        {
            float* data = (float*)outs[i].data;
            for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
            {
                Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                Point classIdPoint;
                double confidence;
                minMaxLoc(scores, 0, &confidence, 0, &classIdPoint); //找出置信度最大值的指针与最大值位置的指针
                if (confidence > 0.6)//0.2最低置信度，由于为训练设置0.2较低值
                {
                    int centerX = (int)(data[0] * input_image.cols);
                    int centerY = (int)(data[1] * input_image.rows);
                    int width = (int)(data[2] * input_image.cols);
                    int height = (int)(data[3] * input_image.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)confidence);
                    boxes.push_back(Rect(left, top, width, height)); //存储识别框参数【种类、置信度、位置】
                }
            }
        }

        vector<int> indices;

        NMSBoxes(boxes, confidences, 0.2, 0.2, indices);//进一步过滤，去掉重复框
	double min =1000;
	double dis =0;
	int xx=0,yy=0,x=0,y=0;
        for (size_t i = 0; i < indices.size(); ++i)     //画框，输出信息
        {
            
            int idx = indices[i];
            Rect box = boxes[idx];
            String className = classNamesVec[classIds[idx]];
            

            ifdetect=false;
            putText(input_image, className.c_str(), box.tl(), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0), 2, 8);
            rectangle(input_image, box, Scalar(0, 0, 255), 2, 8, 0);

            //获取框中心点坐标
            xx = box.x+0.5*box.width;
            yy = box.y+0.5*box.height;
            dis = sqrt((xx-320)*(xx-320)+(yy-240)*(yy-240));
            if(min>dis)
            {
            	min=dis;
            	x=xx;
            	y=yy;
            }
            
            
        }
	
	ros::spinOnce();
	cv::Point p1(0, 480), p2(200, 240); // (x, y) = (w, h)
	cv::Point p3(640, 480), p4(440, 240); // (x, y) = (w, h)
	cv::Point p5(300, 240), p6(340, 240); // (x, y) = (w, h)
	cv::Point p7(320, 260), p8(320, 220); // (x, y) = (w, h)
	cv::Scalar colorLine(0, 255, 0); // Green - (B, G, R)
	int thicknessLine = 1;

	cv::line(input_image, p1, p2, colorLine, 2);
	cv::line(input_image, p3, p4, colorLine, 2);
	cv::line(input_image, p5, p6, colorLine, thicknessLine);
	cv::line(input_image, p7, p8, colorLine, thicknessLine);

        // 发布消息

        if(!ifdetect)
        {	
        	chatter_pub.publish(empty_goal);
        	pub_zero_vel=true;
        	if(x<200)
        	{
    			ROS_INFO("current goal have been canceled" ); //打印消息
    			//diss=(200-x)/200;
    			//if((200-x)<100)
    			//	diss=0.5;
    			vel_msg.angular.z=0.24;
    			vel_msg.linear.x=0.0;
        		vel_pub.publish(vel_msg);
        	}
        	else if(x>440)
        	{        		
    			ROS_INFO("current goal have been canceled" ); //打印消息
    			//diss=(x-440)/200;
    			//if((x-440)<100)
    			//	diss=0.5;
    			vel_msg.angular.z=-0.24;
    			vel_msg.linear.x=0.0;
        		vel_pub.publish(vel_msg);
        	}
        	else
        	{
        		ROS_INFO("the boat is xunhanging ,trash detecting");
			vel_msg.angular.z=0;
			vel_msg.linear.x=0.1;
        		vel_pub.publish(vel_msg);
        	}    		
        }
        else
        {
        	//继续巡航
        	
        	if(pub_zero_vel)
        	{
        		vel_msg.angular.z=0.0;
			vel_msg.linear.x=0.0;
        		vel_pub.publish(vel_msg);
        		pub_zero_vel=false;
        		ROS_INFO("the boat is xunhanging ,no trash detected"); 
        	}
        	//moveControl.move();
        
        }
        

	ifdetect=true;
        //Show the image
        cv::namedWindow(WINDOW);
        cv::imshow(WINDOW,input_image);
        cv::waitKey(1);
    }
    
    return 0;
}
