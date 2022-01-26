#pragma once
#include"point_cloud_recognise_function.h"

// 候选深度阈值
#define candidate_depth_threshold 10

// 盲区2D无效值阈值
#define middle_Invalid_px_threshold 35

// ransac 分割次数
#define ransac_step 250

// ransac 循环次数
#define ransac_times 20

// ransac 模式阈值
#define ransac_threshold 5

// 点云3D面积阈值(最大阈值)
#define point_cloud_area_max 5000

// 点云3D面积阈值（最小阈值）
#define point_cloud_area_min 50

// 缺陷长度阈值
#define flaw_length_threshold 200

// 缺陷宽度阈值
#define flaw_weight_threshold 5

#define invalid_zvalue -2500

#define IMPOSSIBLE_ZVALUE_THRESHOLD 50

void compensate(Mat img_3D, vector<float>& list_px, int roi_min, int roi_max);

void ransac_recognise(Mat& image, Mat& image_mask, int ROI_min, int ROI_max);

void show_image(Mat image, int size);

void ransac_partiton(Mat img_3D, Mat& mask, Mat& image_mask, vector<int>& px_partition, double threshold_hight);

void remove_mask_px(Mat& mask, Mat& img_2D, int roi_min, int roi_max, int threshold_px);

void cut_long_flaw(Mat& mask, int px_length, int threshold_length);

void cut_long(Mat& mask, int roi_min, int roi_max, int flaw_length_min);

void get_deep_total(Mat& image_src, Mat& draw_image_mask, vector<float>& px);

void get_depth(Mat& image_mask, Mat& img_2D_show, recogn_OUT_for_point_cloud& out, vector<int>& px_partition, Mat& img_3D, Mat& img_2D);

void remove_mask_px(Mat& mask, Mat& img_2D, int roi_min, int roi_max, int threshold_px);

void main_recognise_for_point_cloud_3D(Mat& img_3D, Mat& img_2D, recogn_OUT_for_point_cloud& out, int flat);