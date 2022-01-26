#pragma once
#ifndef POINT_CLOUD_RECOGNISE_FUNCTION_H

#include"opencv2/opencv.hpp"
#include<stdlib.h>
#include<algorithm>
#include<time.h>
#include<vector>
#include<string>
#include<string.h>
#include<iostream>
#include<thread>
#include<list>

#endif // !POINT_CLOUD_RECOGNISE_FUNCTION_H
using namespace cv;
using namespace std;

struct flawimformation
{

	/// <summary>
	/// contour 缺陷的连通域信息
	/// center 缺陷的中心坐标 x，y
	/// area 缺陷的面积
	/// length 缺陷的长度
	/// position 缺陷的位置
	/// type 缺陷类型
	/// depth 缺陷深度
	/// </summary>
	vector<Point>contour;

	Point2f center;

	double area;
	double length;
	double depth;
	int position;
	int type;


	flawimformation(vector<Point>contour, Point2f center, double area,
		double length, int position, int type, double depth) :contour(contour), center(center), area(area), length(length), position(position), type(type), depth(depth) {}
};


struct recogn_OUT_for_point_cloud
{

	vector<int> max_compare;

	vector<Mat> image;

	vector<flawimformation>flaw_total;

	recogn_OUT_for_point_cloud(vector<int> max_compare, vector<Mat> image, vector<flawimformation>flaw_total) :max_compare(max_compare), image(image), flaw_total(flaw_total) {}
};

void hstack(Mat image_src1, Mat image_src2, Mat& image);

void image_auto_cut(Mat image, vector<int>& ROI, int border, int flat);

void show_image(Mat image, int gan);
