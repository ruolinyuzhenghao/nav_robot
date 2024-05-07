#include<opencv2/opencv.hpp>
#include<iostream>
#include <fstream>
#include <vector>
std::ofstream output_stream;
std::string txt_path = "/home/rover/map_new.txt";

void on_mouse(int event, int x, int y, int flags, void* ustc);
void drawRectangle(cv::Mat src, cv::Point pt1, cv::Point pt2);
void drawCircle(cv::Mat src, cv::Point pt_center, int pt_radius);
void drawLine(cv::Mat src, cv::Point pt1, cv::Point pt2, cv::Scalar color);

cv::Point pre_pt= cv::Point(-1,-1), cur_pt= cv::Point(-1, -1);//鼠标点
std::vector<int>map_new;
int count=0;
#define WINDOW_NAME "image"

int main()
{
    cv::Mat srcImage;
    cv::Point center;

    cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);
    output_stream.open(txt_path);
    while (count<4)
    {
        srcImage=cv::imread("/home/rover/catkin_ws/src/ipa_coverage_planning-noetic_dev/ipa_room_exploration/map/map.pgm",CV_8U);
        if (srcImage.empty())
        {
            std::cout << "地图加载失败 ！" << std::endl;
                return -1;
        }
        double x=-6.98,y=-11.99;
        cv::circle(srcImage,//目标图形
			cv::Point((0-x)/0.05, (0-y)/0.05),//中心点坐标
			5,//半径
			CV_RGB(255, 0,0),//颜色（这里用黑色）
			5);//厚度
	cv::circle(srcImage,//目标图形
			cv::Point((5-x)/0.05, (0-y)/0.05),//中心点坐标
			5,//半径
			CV_RGB(255, 0,0),//颜色（这里用黑色）
			5);//厚度
	cv::circle(srcImage,//目标图形
			cv::Point((0-x)/0.05, (5-y)/0.05),//中心点坐标
			5,//半径
			CV_RGB(255, 0,0),//颜色（这里用黑色）
			5);//厚度
			
        cv::setMouseCallback(WINDOW_NAME, on_mouse, 0);
        if (cur_pt.x!=-1&& cur_pt.y != -1)
        {
            drawLine(srcImage, pre_pt, cur_pt, cv::Scalar(0,0,0));
            
        }


        imshow(WINDOW_NAME, srcImage);
        cv::waitKey(1);
    }
    	int max_x=0,max_y=0;
    	int min_x=1000,min_y=1000;
    	for(int i=0;i<map_new.size()-1;i=i+2)
    	{
    		min_x=std::min(min_x,map_new[i]);
    		max_x=std::max(max_x,map_new[i]);
    		min_y=std::min(min_y,map_new[i+1]);
    		max_y=std::max(max_y,map_new[i+1]);	
    	}
    	 std::cout <<srcImage.cols<< std::endl;
    	for(int i=0;i<srcImage.cols;i++)
    	{
    		for(int j=0;j<srcImage.rows;j++)
    		{
    			if(i>min_x&&i<max_x&&j>min_y&&j<max_y)
    			{
    				srcImage.at<uchar>(j,i)=255;
    			}
    			else
    			{
    				srcImage.at<uchar>(j,i)=0;
    			}
    		}
    	}
    	srcImage.convertTo(srcImage, CV_8U);

    	cv::imwrite("/home/rover/catkin_ws/src/ipa_coverage_planning-noetic_dev/ipa_room_exploration/map/map.pgm",srcImage);
    //按ESC退出时销毁所有窗口
    cv::destroyAllWindows();
    return 0;
}

//画线
void drawLine(cv::Mat src, cv::Point pt1, cv::Point pt2, cv::Scalar color)
{
    cv::line(src, pt1, pt2, color, 1);
}

//鼠标操作
void on_mouse(int event, int x, int y, int flags, void* ustc)
{
    //左键按下
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        cv::Point pt= cv::Point(x, y);
        pre_pt = pt;

    }
    //左键按下并且鼠标移动
    else if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON))
    {
        cv::Point pt = cv::Point(x, y);
        cur_pt = pt;
    }
    //左键弹起
    else if (event == cv::EVENT_LBUTTONUP)
    {
        cv::Point pt = cv::Point(x, y);
        cur_pt = pt;
        //保存路径点到path.txt文件
        map_new.push_back(cur_pt.x);
        map_new.push_back(cur_pt.y);
        count++;
        
    }
}

