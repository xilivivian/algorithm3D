#include"point_cloud_recognise_function.h"

// 2D主区域分割（手动版）
void image_auto_cut(Mat image, vector<int>& ROI, int border, int flat)
{

	switch (flat)
	{

	case 0: {

		int image_row = image.rows;
		int image_col = image.cols;

		int Q4 = 898;
		int Q1 = Q4 - 507;
		int Q2 = Q4 - 390;
		int Q3 = Q4 - 336;
		int Q5 = Q4 + 505;
		int Q6 = Q4 + 680;
		int Q7 = Q4 + 872;

		ROI.push_back(0 - border);
		ROI.push_back(Q1);
		ROI.push_back(Q1);
		ROI.push_back(Q2);
		ROI.push_back(Q3);
		ROI.push_back(Q4);
		ROI.push_back(Q4);
		ROI.push_back(Q5);
		ROI.push_back(Q5);
		ROI.push_back(Q6);
		ROI.push_back(Q6);
		ROI.push_back(Q7);
		ROI.push_back(Q7);
		ROI.push_back(image_col + border);


	}break;
	case 1: {

		int image_row = image.rows;
		int image_col = image.cols;

		int Q4 = 898;
		int Q1 = Q4 - 507;
		int Q2 = Q4 - 390;
		int Q3 = Q4 - 336;
		int Q5 = Q4 + 505;
		int Q6 = Q4 + 680;
		int Q7 = Q4 + 872;

		ROI.push_back(0 - border);
		ROI.push_back(Q1);
		ROI.push_back(Q1);
		ROI.push_back(Q2);
		ROI.push_back(Q3);
		ROI.push_back(Q4);
		ROI.push_back(Q4);
		ROI.push_back(Q5);
		ROI.push_back(Q5);
		ROI.push_back(Q6);
		ROI.push_back(Q6);
		ROI.push_back(Q7);
		ROI.push_back(Q7);
		ROI.push_back(image_col + border);

	}break;
	default:
		break;
	}
}

// 图片拼接函数，调算法输出看
void hstack(Mat image_src1, Mat image_src2, Mat& image)
{

	int totalCols = image_src1.cols + image_src2.cols;
	image = Mat::zeros(image_src1.rows, totalCols, image_src1.type());
	Mat submat = image.colRange(0, image_src1.cols);
	image_src1.copyTo(submat);
	submat = image.colRange(image_src1.cols, totalCols);
	image_src2.copyTo(submat);
}

// 图片显示显示函数，调算法看
void show_image(Mat image, int gan)
{

	resize(image, image, Size(image.cols / gan, image.rows / gan), 0, 0);

	imshow("1", image);
	waitKey(0);
}

