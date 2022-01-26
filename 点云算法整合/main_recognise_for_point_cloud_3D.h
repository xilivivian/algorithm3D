#pragma once
#include"point_cloud_recognise_function.h"

// ��ѡ�����ֵ
#define candidate_depth_threshold 10

// ä��2D��Чֵ��ֵ
#define middle_Invalid_px_threshold 35

// ransac �ָ����
#define ransac_step 250

// ransac ѭ������
#define ransac_times 20

// ransac ģʽ��ֵ
#define ransac_threshold 5

// ����3D�����ֵ(�����ֵ)
#define point_cloud_area_max 5000

// ����3D�����ֵ����С��ֵ��
#define point_cloud_area_min 50

// ȱ�ݳ�����ֵ
#define flaw_length_threshold 200

// ȱ�ݿ����ֵ
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