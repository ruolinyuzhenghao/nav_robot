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
static const char WINDOW[] = "Image window";
using namespace std;
using namespace cv;
using namespace cv::dnn;

/*****各项参数*********/
static double pose_x=0,pose_y=0;
static vector<float>pathpoints;
double dis=0;
int pose_j=0;
int last_pose_j=0;
double min=1000;



void pose_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    double min=1000;
    ROS_INFO("boat_pose->x=%f",msg->pose.pose.position.x);
    ROS_INFO("boat_pose->y=%f",msg->pose.pose.position.y);
    pose_x=msg->pose.pose.position.x;
    pose_y=msg->pose.pose.position.y;
    
    for(int i=0;i<pathpoints.size()-1;i=i+2)
    {
    	double path_dis=std::sqrt((pathpoints[i]-pose_x)*(pathpoints[i]-pose_x)+(pathpoints[i+1]-pose_y)*(pathpoints[i+1]-pose_y));
    	if(path_dis<min)
    	{
    		min=path_dis;
    		pose_j=i;
    	}
    	//ROS_INFO("pose_j----->%d",pose_j);
    }

	
}


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char *argv[])
{
    //加载路径点
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
    
    // ROS节点初始化
    ros::init(argc, argv, "video_publisher");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
    ros::Publisher stop_info_pub = n.advertise<std_msgs::Bool>("/btn_press", 10);
    ros::Publisher connect_info_pub = n.advertise<std_msgs::Bool>("/btn_press_continue", 10);
    ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, pose_Callback);
    

    MoveBaseClient ac("move_base", true);
    move_base_msgs::MoveBaseGoal goal;
    while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    } //等待操作服务器报告它已出现并准备开始处理目标
    
    std_msgs::Bool msg;
    bool ifdetect=true;
    
    cv::VideoCapture cap(0);
    cap.set(CAP_PROP_FRAME_WIDTH,1280);
    cap.set(CAP_PROP_FRAME_HEIGHT,480);
    Mat input_image,frame;
    //Reading an image from the file
    String yolo_cfg = "/home/rover/下载/cfg/yolo.cfg";
    String yolo_widght = "/home/rover/下载/cfg/yolov4-tiny.weights";
    String yolo_class = "/home/rover/下载/cfg/classes.txt";
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
        input_image=frame(Range(0,480),Range(640,1280));
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
                if (confidence > 0.2)//0.2最低置信度，由于为训练设置0.2较低值
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
	int x=0,y=0;
        for (size_t i = 0; i < indices.size(); ++i)     //画框，输出信息
        {
            ifdetect=false;
            int idx = indices[i];
            Rect box = boxes[idx];
            String className = classNamesVec[classIds[idx]];
            putText(input_image, className.c_str(), box.tl(), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0), 2, 8);
            rectangle(input_image, box, Scalar(0, 0, 255), 2, 8, 0);

            //获取框中心点坐标
            int xx = box.x+0.5*box.width;
            int yy = box.y+0.5*box.height;

            

        }

	ros::spinOnce();
	
        // 发布消息
        if(!ifdetect)
        {
        	pose_j+=2;
    		if(pathpoints[pose_j+1]==pathpoints[pose_j-1]&&pose_j>3&&pathpoints[pose_j+1]!=pathpoints[pose_j+3]&&last_pose_j!=pose_j)
    		{
    			last_pose_j=pose_j;
    			msg.data=ifdetect;
			stop_info_pub.publish(msg);
			
			goal.target_pose.header.frame_id = "base_link";
  			goal.target_pose.header.stamp = ros::Time::now();
  			goal.target_pose.pose.position.x = 0.5;
  			goal.target_pose.pose.position.y = 0;
  			goal.target_pose.pose.orientation.w = 1.0;

  			ROS_INFO("Sending goal-->%d",pose_j);
  			ac.sendGoal(goal); 
  			ac.waitForResult();
	
  			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    			ROS_INFO("Hooray, the base moved to the goal");
  			else
    			ROS_INFO("The base failed to move to the goal"); 
    		}
		pose_j-=2;
        }
        

	ifdetect=true;
        //Show the image
        cv::namedWindow(WINDOW);
        cv::imshow(WINDOW,input_image);
        cv::waitKey(1);
    }
    
    return 0;
}
