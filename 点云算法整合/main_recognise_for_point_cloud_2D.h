#pragma once
#include"point_cloud_recognise_function.h"

// 点云2D的面积阈值
#define AREA_point_cloud_for_2D 50
// 点云2D的对比度阈值
#define COMPARE_point_cloud_for_2D 20

void show_image(Mat image, int gan);

void statistics(Mat& image, vector<int>& px, int& num);

void show_compare_px(Mat& image_src, Mat& mask, Mat& img, int& max_compare);

int median(Mat& image_src, Mat& draw_image_mask);

void remove_small_objects(Mat& image, Mat& mask, int area_max);

void remove_px_double(Mat& image_src, Mat& image, int min_precent, int max_precent);

bool main_recognise_for_point_cloud_2D(Mat& image, recogn_OUT_for_point_cloud& OUT, int flat);

void image_auto_cut(Mat image, vector<int>& ROI, int border, int flat);

void mask_to_image_result(Mat& image_src, Mat mask, int roi_min);

void mask_to_image(Mat& mask, Mat& image_src, Mat& image);

void argsort(vector<int>px, vector<int>& px_sort);

void adapt_remove_px(Mat& image, Mat& image_res, int threshold);

void recognise_flaw_for_point_cloud_2D(Mat& image, Mat& image_res, vector<flawimformation>& flaw_total, int roi_min, int roi_max, int min_threshold, int max_threshold, int position, int& max_compare);

