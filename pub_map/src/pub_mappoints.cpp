/*
    需求: 循环发布地图点

*/

#include "ros/ros.h"
#include "pub_map/Map.h"
#include <fstream>
#include <vector>
using namespace std;
int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"talker_map");

    //2.创建 ROS 句柄
    ros::NodeHandle nh;

    //3.创建发布者对象
    ros::Publisher pub = nh.advertise<pub_map::Map>("chatter_map",1000);

    //4.组织被发布的消息，编写发布逻辑并发布消息
    pub_map::Map p;
    fstream myfile("/home/rover/map_point.txt");
    if (!myfile.is_open())
    {
	cout << "can not open this file" << endl;
	return 0;
    }

    vector<float>mappoints;
    while(!myfile.eof())
    {
	float_t i=0;
	myfile>>i;
	mappoints.push_back(i);
    }
    ros::Rate r(1);
    int count=0;
    p.ip="map_mappoints";
    while (ros::ok())
    {
        p.x1=mappoints[count];
        count++;
        p.y1=mappoints[count];
        count++;
        p.x2=mappoints[count];
        count++;
        p.y2=mappoints[count];
        count++;

        pub.publish(p);
        if(count>=mappoints.size())
        {
        	count=0;
        	continue;
        }
        ROS_INFO("第%d条边界端点坐标是[%f,%f],[%f,%f]",count/4,p.x1,p.y1,p.x2,p.y2);

        r.sleep();
        ros::spinOnce();
    }



    return 0;
}

