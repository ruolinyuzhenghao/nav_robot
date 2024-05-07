/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2017 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: autopnp
 * \note
 * ROS package name: ipa_room_exploration
 *
 * \author
 * Author: Richard Bormann
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 02.2017
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/


#include <ipa_room_exploration/room_rotator.h>
#include <fstream>

#include <vector>
#define WINDOW_NAME "image"
using namespace cv;
//画线
int count=0;
cv::Point pre_pt= cv::Point(-1,-1), cur_pt= cv::Point(-1, -1);//鼠标点
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
        count++;
    }
}

void RoomRotator::rotateRoom(const cv::Mat& room_map, cv::Mat& rotated_room_map, const cv::Mat& R, const cv::Rect& bounding_rect)
{
	// rotate the image
	cv::warpAffine(room_map, rotated_room_map, R, bounding_rect.size(), cv::INTER_AREA);

	// apply a binary filter to create a binary image, also use a closing operator to smooth the output (the rotation might produce
	// black pixels reaching into the white area that were not there before, causing new, wrong cells to open)
#if CV_MAJOR_VERSION<=3
	cv::threshold(rotated_room_map, rotated_room_map, 127, 255, CV_THRESH_BINARY);
#else
	cv::threshold(rotated_room_map, rotated_room_map, 127, 255, cv::THRESH_BINARY);
	/***********裁剪地图v1***********/
	/*
	cv::Mat edge_map;
	cv::Canny(rotated_room_map, edge_map, 50, 150, 3);
	std::vector<cv::Vec4i> lines;
	double min_line_length = 1.0;	// in [m]
	for (; min_line_length > 0.1; min_line_length -= 0.2)
	{
	cv::HoughLinesP(edge_map, lines, 1, CV_PI/180, min_line_length*20, min_line_length*20, 1.5*min_line_length*20);
	cv::Mat room_hough = edge_map.clone();
	for (size_t i=0; i<lines.size(); ++i)
	{
	cv::Point p1(lines[i][0], lines[i][1]), p2(lines[i][2], lines[i][3]);
	cv::line(room_hough, p1, p2, cv::Scalar(128), 2);
	}
	}
	//找出四个点
	int x = 10000;
	int xx=0;
	for(int i=0;i<lines.size();++i)
	{
		x=std::min(x,lines[i][1]);
		x=std::min(x,lines[i][3]);
		xx=std::max(xx,lines[i][1]);
		xx=std::max(xx,lines[i][3]);
	}
	for(int i=x;i<x+10;i++)
	{
		for(int j=0;j<rotated_room_map.cols;j++)
		{
			rotated_room_map.at<uchar>(i,j)=0;
		}
	}
	for(int i=xx;i>xx-10;i--)
	{
		for(int j=0;j<rotated_room_map.rows;j++)
		{
			rotated_room_map.at<uchar>(i,j)=0;
		}
	}
	*/
	/***********裁剪地图v2***********/
	/*
	std::fstream myfile("/home/rover/map_new.txt");
    	if (!myfile.is_open())
    	{
		std::cout << "can not open this file" << std::endl;
    	}

   	std::vector<int>map_new;
    	while(!myfile.eof())
    	{
		int i=0;
		myfile>>i;
		map_new.push_back(i);
    	}
    	//cv::circle(rotated_room_map, cv::Point(map_new[0], map_new[1]), 2, cv::Scalar(128), -1);
    	//cv::line(rotated_room_map, cv::Point(map_new[0], map_new[1]),cv::Point(map_new[2], map_new[3]), cv::Scalar(0,255,0), 1);
    	//cv::line(rotated_room_map, cv::Point(map_new[2], map_new[3]),cv::Point(map_new[4], map_new[5]), cv::Scalar(0,255,0), 1);
    	//cv::line(rotated_room_map, cv::Point(map_new[4], map_new[5]),cv::Point(map_new[6], map_new[7]), cv::Scalar(0,255,0), 1);
    	//cv::line(rotated_room_map, cv::Point(map_new[6], map_new[7]),cv::Point(map_new[0], map_new[1]), cv::Scalar(0,255,0), 1);
    	//for(int i=0;i<map_new.size()-3;i=i+4)
    	//{
    	//	cv::line(rotated_room_map, cv::Point(map_new[i], map_new[i+1]),cv::Point(map_new[i+2], map_new[i+3]), cv::Scalar(0,255,0), 1);
    	//}
    	int max_x=0,max_y=0;
    	int min_x=1000,min_y=1000;
    	for(int i=0;i<map_new.size()-1;i=i+2)
    	{
    		min_x=std::min(min_x,map_new[i]);
    		max_x=std::max(max_x,map_new[i]);
    		min_y=std::min(min_y,map_new[i+1]);
    		max_y=std::max(max_y,map_new[i+1]);	
    	}
    	
    	for(int i=0;i<rotated_room_map.cols;i++)
    	{
    		for(int j=0;j<rotated_room_map.rows;j++)
    		{
    			if(i>min_x&&i<max_x&&j>min_y&&j<max_y)
    			{
    				rotated_room_map.at<uchar>(j,i)=255;
    			}
    			else
    			{
    				rotated_room_map.at<uchar>(j,i)=0;
    			}
    		}
    	}
    	*/

cv::imwrite("/home/rover/rotated_room_map.jpg",rotated_room_map);

	
#endif
// this should not be used because it removes smaller obstacles like thin walls from the room and hence lets a planner generate paths through walls
//	cv::dilate(rotated_room_map, rotated_room_map, cv::Mat(), cv::Point(-1,-1), 1);
//	cv::erode(rotated_room_map, rotated_room_map, cv::Mat(), cv::Point(-1,-1), 1);
}

// compute the affine rotation matrix for rotating a room into parallel alignment with x-axis (longer side of the room is aligned with x-axis)
// R is the transform
// bounding_rect is the ROI of the warped image
double RoomRotator::computeRoomRotationMatrix(const cv::Mat& room_map, cv::Mat& R, cv::Rect& bounding_rect,
		const double map_resolution, const cv::Point* center, const double rotation_offset)
{
	// rotation angle of the map s.t. the most occurring gradient is in 90 degree to the x-axis
	double rotation_angle = computeRoomMainDirection(room_map, map_resolution) + rotation_offset;
	std::cout << "RoomRotator::computeRoomRotationMatrix: main rotation angle: " << rotation_angle << std::endl;

	// get rotation matrix R for rotating the image around the center of the room contour
	//	Remark: rotation angle in degrees for opencv
	cv::Point center_of_rotation;
	if (center == 0)
	{
		cv::Point min_room, max_room;
		getMinMaxCoordinates(room_map, min_room, max_room);
		center_of_rotation.x = 0.5*(min_room.x+max_room.x);
		center_of_rotation.y = 0.5*(min_room.y+max_room.y);
	}
	else
		center_of_rotation = *center;

	// compute rotation
	R = cv::getRotationMatrix2D(center_of_rotation, (rotation_angle*180)/CV_PI, 1.0);

	// determine bounding rectangle to find the size of the new image
	bounding_rect = cv::RotatedRect(center_of_rotation, room_map.size(), (rotation_angle*180)/CV_PI).boundingRect();
	// adjust transformation matrix
	R.at<double>(0,2) += 0.5*bounding_rect.width - center_of_rotation.x;
	R.at<double>(1,2) += 0.5*bounding_rect.height - center_of_rotation.y;

	return rotation_angle;
}

// computes the major direction of the walls from a map (preferably one room)
// the map (room_map, CV_8UC1) is black (0) at impassable areas and white (255) on drivable areas
double RoomRotator::computeRoomMainDirection(const cv::Mat& room_map, const double map_resolution)
{
	unsigned long txtCount = 0; 
	std::ofstream output_stream;
	std::string txt_path = "/home/rover/map_point.txt"; 
	output_stream.open(txt_path);
	
	const double map_resolution_inverse = 1./map_resolution;

	// compute Hough transform on edge image of the map
	cv::Mat edge_map;
	cv::Canny(room_map, edge_map, 50, 150, 3);
	std::vector<cv::Vec4i> lines;
	double min_line_length = 1.0;	// in [m]
	for (; min_line_length > 0.1; min_line_length -= 0.2)
	{
		cv::HoughLinesP(edge_map, lines, 1, CV_PI/180, min_line_length*map_resolution_inverse, min_line_length*map_resolution_inverse, 1.5*min_line_length*map_resolution_inverse);
		cv::Mat room_hough = edge_map.clone();
		for (size_t i=0; i<lines.size(); ++i)
		{
			cv::Point p1(lines[i][0], lines[i][1]), p2(lines[i][2], lines[i][3]);
			cv::line(room_hough, p1, p2, cv::Scalar(128), 2);
			cv::circle(room_hough, p1, 4, cv::Scalar(100), cv::FILLED);
			cv::circle(room_hough, p2, 4, cv::Scalar(100), cv::FILLED);
			output_stream<< p1.x*0.05-1.2;
			output_stream<<"  ";
			output_stream<<p1.y*0.05-1.15;
			output_stream<<"  ";
			output_stream<< p2.x*0.05-1.2;
			output_stream<<"  ";
			output_stream<< p2.y*0.05-1.15;
			output_stream<<std::endl;
			
		}
		cv::imwrite("/home/rover/room_hough.jpg", room_hough);
		
		if (lines.size() >= 4)
			break;
	}
	// setup a histogram on the line directions weighted by their length to determine the major direction
	Histogram<double> direction_histogram(0, CV_PI, 36);
	std::cout<<"linesize:"<<lines.size()<<std::endl;
	for (size_t i=0; i<lines.size(); ++i)
	{
		double dx = lines[i][2] - lines[i][0];
		double dy = lines[i][3] - lines[i][1];
		if(dy*dy+dx*dx > 0.0)
		{
			double current_direction = std::atan2(dy, dx);
			while (current_direction < 0.)
				current_direction += CV_PI;
			while (current_direction > CV_PI)
				current_direction -= CV_PI;
			direction_histogram.addData(current_direction, sqrt(dy*dy+dx*dx));
			std::cout << " dx=" << dx << "   dy=" << dy << "   dir=" << current_direction << "   len=" << sqrt(dy*dy+dx*dx)*0.05 << std::endl;
		}
	}
}

void RoomRotator::transformPathBackToOriginalRotation(const std::vector<cv::Point2f>& fov_middlepoint_path, std::vector<geometry_msgs::Pose2D>& path_fov_poses, const cv::Mat& R)
{
	path_fov_poses.clear();

	// transform the calculated path back to the originally rotated map
	cv::Mat R_inv;
	cv::invertAffineTransform(R, R_inv);
	std::vector<cv::Point2f> fov_middlepoint_path_transformed;
	cv::transform(fov_middlepoint_path, fov_middlepoint_path_transformed, R_inv);

	// create poses with an angle
	transformPointPathToPosePath(fov_middlepoint_path_transformed, path_fov_poses);
}

void RoomRotator::transformPointPathToPosePath(const std::vector<cv::Point2f>& point_path, std::vector<geometry_msgs::Pose2D>& pose_path)
{
	// create poses with an angle
	for(size_t point_index = 0; point_index < point_path.size(); ++point_index)
	{
		// get the vector from the previous to the current point
		const cv::Point2f& current_point = point_path[point_index];

		// add the next navigation goal to the path
		geometry_msgs::Pose2D current_pose;
		current_pose.x = current_point.x;
		current_pose.y = current_point.y;
		current_pose.theta = 0.;
		cv::Point2f vector(0,0);
		if (point_index > 0)
		{
			// compute the direction as the line from the previous point to the current point
			vector = current_point - point_path[point_index-1];
		}
		else if (point_path.size() >= 2)
		{
			// for the first point take the direction between first and second point
			vector = point_path[point_index+1] - current_point;
		}
		// only sample different points
		if (vector.x!=0 || vector.y!=0)
		{
			current_pose.theta = std::atan2(vector.y, vector.x);
			pose_path.push_back(current_pose);
		}
	}
}

void RoomRotator::transformPointPathToPosePath(std::vector<geometry_msgs::Pose2D>& pose_path)
{
	// create point vector
	std::vector<cv::Point2f> point_path;
	for (size_t i=0; i<pose_path.size(); ++i)
		point_path.push_back(cv::Point2f(pose_path[i].x, pose_path[i].y));

	// create poses with an angle
	pose_path.clear();
	transformPointPathToPosePath(point_path, pose_path);
}

void RoomRotator::getMinMaxCoordinates(const cv::Mat& map, cv::Point& min_room, cv::Point& max_room)
{
	min_room.x = std::numeric_limits<int>::max();
	min_room.y = std::numeric_limits<int>::max();
	max_room.x = 0;
	max_room.y = 0;
	for (int v=0; v<map.rows; ++v)
	{
		for (int u=0; u<map.cols; ++u)
		{
			if (map.at<uchar>(v,u)==255)
			{
				min_room.x = std::min(min_room.x, u);
				min_room.y = std::min(min_room.y, v);
				max_room.x = std::max(max_room.x, u);
				max_room.y = std::max(max_room.y, v);
			}
		}
	}
}
