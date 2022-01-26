#include<iostream>
#include <fstream>
#include <sstream>
#include"main_recognise_for_point_cloud_3D.h"
#include"main_recognise_for_point_cloud_2D.h"
#include<direct.h>
#include<io.h>

using namespace std;

/// <summary>
/// Macro to check whether the point is measured or not
/// </summary>
#define NOT_MEASURED_POINT(x) ((x)>9999998)

void read_csv(string filename, Mat& img_2D, Mat& img_3D, int x_rate, int y_rate);

bool SaveBatchPointCloud(Mat& img_2D, Mat& img_3D, const float* zValues, const float* intensityValues, int image_width, int image_height);

/// <summary>
/// 读取csv文件，构2D、3D图
/// </summary>
/// <param name="filename"></param>
/// <param name="img_2D"></param>
/// <param name="img_3D"></param>
/// <param name="x_rate"></param>
/// <param name="y_rate"></param>
//void read_csv(string filename, Mat& img_2D, Mat& img_3D, int x_rate, int y_rate)
//{
//	ifstream inFile(filename, ios::in);
//	string lineStr;
//	vector<vector<float>> strArray;
//
//	float max_x = -10000000.0, max_y = -10000000.0;
//
//	float min_x = 10000000.0, min_y = 10000000.0;
//
//	while (getline(inFile, lineStr))
//	{
//		// 打印整行字符串
//		//cout << lineStr << endl;
//		// 存成二维表结构
//		stringstream ss(lineStr);
//		string str;
//		vector<float> lineArray;
//		// 按照' '分隔
//
//		int i = 0;
//		while (getline(ss, str, ' '))
//		{
//			float data = stof(str);
//			if (i == 0)
//			{
//				data *= 1000000.0;
//				max_x = max(max_x, data);
//				min_x = min(min_x, data);
//			}
//			else if (i == 1)
//			{
//				data *= 1000000.0;
//				max_y = max(max_y, data);
//				min_y = min(min_y, data);
//			}
//			lineArray.push_back(data);
//			i += 1;
//		}
//		strArray.push_back(lineArray);
//	}
//
//	int strArray_length = strArray.size();
//
//	int image_col = int((max_y - min_y) / y_rate) + 1;
//	int image_row = int((max_x - min_x) / x_rate) + 1;
//
//	img_3D = Mat::ones(Size(image_row, image_col), CV_32FC1);
//
//	img_3D *= -2;
//
//	img_2D = Mat::zeros(Size(image_row, image_col), CV_8UC1);
//
//	for (int i = 0; i < strArray_length; i++)
//	{
//
//		int x = (strArray[i][0] - min_x) / x_rate;
//		int y = (strArray[i][1] - min_y) / y_rate;
//
//		img_3D.at<float>(y, x) = strArray[i][2];
//
//		img_2D.at<uchar>(y, x) = int(strArray[i][3]);
//	}
//}


/// <summary>
/// 读取csv文件，构2D、3D图
/// </summary>
/// <param name="filename"></param>
/// <param name="img_2D"></param>
/// <param name="img_3D"></param>
/// <param name="x_rate"></param>
/// <param name="y_rate"></param>
//void read_csv(string filename, Mat& img_2D, Mat& img_3D, int x_rate, int y_rate)
//{
//	ifstream inFile(filename, ios::in);
//	string lineStr;
//	vector<vector<float>> strArray;
//
//	float max_x = -10000000.0, max_y = -10000000.0;
//
//	float min_x = 10000000.0, min_y = 10000000.0;
//
//	clock_t start = clock();
//	//printf("creat image using time is %d s\n", start);
//	//clock_t end;
//
//	//clock_t time_total;
//
//	while (getline(inFile, lineStr))
//	{
//
//		// 打印整行字符串
//		//cout << lineStr << endl;
//		// 存成二维表结构
//		vector<float> lineArray = vector<float>(4);
//
//		// 按照' '分隔
//
//		string str;
//		int size;
//		int index;
//		//string str;
//		for (int j = 0; j < 4; j++)
//		{
//			size = lineStr.length();
//			index = lineStr.find_first_of(' ');
//			str = lineStr.substr(0, index);
//
//			float data = atof(str.data());
//
//			if (j == 0)
//			{
//				data *= 1000000.0;
//				max_x = max(max_x, data);
//				min_x = min(min_x, data);
//			}
//			else if (j == 1)
//			{
//				data *= 1000000.0;
//				max_y = max(max_y, data);
//				min_y = min(min_y, data);
//			}
//
//			lineArray[j] = data;
//			lineStr = lineStr.substr(index + 1, size - index);
//
//		}
//		strArray.push_back(lineArray);
//	}
//
//	int strArray_length = strArray.size();
//
//	int image_col = int((max_y - min_y) / y_rate) + 1;
//	int image_row = int((max_x - min_x) / x_rate) + 1;
//
//	img_3D = Mat::ones(Size(image_row, image_col), CV_32FC1);
//
//	img_3D *= -2;
//
//	img_2D = Mat::zeros(Size(image_row, image_col), CV_8UC1);
//
//	for (int i = 0; i < strArray_length; i++)
//	{
//
//		int x = (strArray[i][0] - min_x) / x_rate;
//		int y = (strArray[i][1] - min_y) / y_rate;
//
//		img_3D.at<float>(y, x) = strArray[i][2];
//
//		img_2D.at<uchar>(y, x) = int(strArray[i][3]);
//	}
//}


void read_csv_2D(string filename, Mat& img_2D)
{

	ifstream inFile(filename, ios::in);
	string lineStr;

	img_2D = Mat::zeros(Size(2048, 1000), CV_8UC1);

	int i = 0;

	for (int i = 0; i < 1000 * 2048; i++)
	{
		if (!getline(inFile, lineStr))
		{
			break;
		}

		lineStr = lineStr.substr(0, lineStr.length() - 1);

		if (lineStr == "999.00")
		{
			continue;
		}

		int data = stoi(lineStr);

		//cout << data << endl;

		img_2D.at<uchar>(i / 2048, i % 2048) = data;
	}
}


void read_csv_3D(string filename, Mat& img_3D)
{

	ifstream inFile(filename, ios::in);
	string lineStr;

	img_3D = Mat::ones(Size(2048, 1000), CV_32FC1);

	img_3D *= invalid_zvalue;

	int i = 0;

	for (int i = 0; i < 1000 * 2048; i++)
	{
		if (!getline(inFile, lineStr))
		{
			break;
		}

		lineStr = lineStr.substr(0, lineStr.length() - 1);

		if (lineStr == "999.00")
		{
			continue;
		}

		float data = stoi(lineStr);

		//cout << data << endl;

		img_3D.at<float>(i / 2048, i % 2048) = data;
	}
}


/// <summary>
/// 识别点云主函数
/// flat 
/// 输入 0  3D上半区
/// 输入 1  3D下半区
/// </summary>
/// <param name="img_3D"></param>
/// <param name="img_2D"></param>
/// <param name="flat"></param>
void main_recognise_for_point_cloud(Mat& img_3D, Mat& img_2D, recogn_OUT_for_point_cloud& out, int flat)
{

	main_recognise_for_point_cloud_3D(img_3D, img_2D, out, flat);
	//main_recognise_for_point_cloud_2D(img_2D, out, flat);

	//// 3D算法调用
	//thread thread_fo_3D = thread(main_recognise_for_point_cloud_3D, ref(img_3D), ref(img_2D), ref(out), flat);
	////thread_fo_3D.join();
	//// 2D算法调用
	//thread thread_fo_2D = thread(main_recognise_for_point_cloud_2D, ref(img_2D), ref(out), flat);

	//thread_fo_3D.join();
	//thread_fo_2D.join();
}


//bool SaveBatchPointCloud(Mat& img_2D, Mat& img_3D, const float* zValues, const float* intensityValues, int image_width, int image_height)
//{
//	static auto batchNumber = 0;
//	const int max_column_pos = 50;
//
//	const std::string fullPath = "batch_" + std::to_string(batchNumber++) + "_layer#" + std::to_string(0) + ".csv";
//
//	img_3D = Mat::ones(Size(image_height, image_width), CV_32FC1);
//	img_3D *= -2;
//	img_2D = Mat::zeros(Size(image_height, image_width), CV_8UC1);
//
//	for (auto j = 0; j < image_height; j++)
//	{
//		float* data_3D = img_3D.ptr<float>(j);
//		uchar* data_2D = img_2D.ptr<uchar>(j);
//
//		for (auto i = 0; i < image_width; i++)
//		{
//			const auto intensity = intensityValues[j * image_width + i];
//			const auto zValue = zValues[j * image_width + i];
//
//			if (NOT_MEASURED_POINT(intensity) || intensity < 0.1)
//				continue;
//			if (NOT_MEASURED_POINT(fabs(zValue)))
//				continue;
//
//			data_3D[i] = static_cast<float>(zValue);
//			data_2D[i] = int(intensity);
//		}
//	}
//
//	delete zValues;
//	delete intensityValues;
//
//	return true;
//}


/// <summary>
/// 原始数据转2D、3D图
/// </summary>
/// <param name="img_2D"></param>
/// <param name="img_3D"></param>
/// <param name="zValues"></param>
/// <param name="intensityValues"></param>
/// <param name="image_width"></param>
/// <param name="image_height"></param>
/// <returns></returns>
bool SaveBatchPointCloud(Mat& img_2D, Mat& img_3D, const float* zValues, const float* intensityValues, int image_width, int image_height)
{
	static auto batchNumber = 0;
	const int max_column_pos = 50;

	const std::string fullPath = "batch_" + std::to_string(batchNumber++) + "_layer#" + std::to_string(0) + ".csv";

	img_3D = Mat::ones(Size(image_width, image_height), CV_32FC1);
	img_3D *= invalid_zvalue;
	img_2D = Mat::zeros(Size(image_width, image_height), CV_8UC1);

	for (auto j = 0; j < image_height; j++)
	{
		float* data_3D = img_3D.ptr<float>(j);
		uchar* data_2D = img_2D.ptr<uchar>(j);

		for (auto i = 0; i < image_width; i++)
		{
			const auto intensity = intensityValues[j * image_width + i];
			const auto zValue = zValues[j * image_width + i];

			if (NOT_MEASURED_POINT(intensity) || intensity < 0.1)
				continue;
			if (NOT_MEASURED_POINT(fabs(zValue)))
				continue;

			data_3D[i] = static_cast<float>(zValue);
			data_2D[i] = int(intensity);
		}
	}

	delete zValues;
	delete intensityValues;

	return true;
}


/// <summary>
/// 点云识别函数
/// </summary>
/// <param name="zValues"></param>
/// <param name="intensityValues"></param>
/// <param name="image_width"></param>
/// <param name="image_height"></param>
/// <param name="xStep"></param>
/// <param name="yStep"></param>
/// <param name="out"></param>
/// <param name="flat"></param>
//void main_recognise_for_point_cloud(const float* zValues, const float* intensityValues, int image_width, int image_height, double xStep, double yStep, recogn_OUT_for_point_cloud& out, int flat)
//{
//
//	Mat img_2D;
//	Mat img_3D;
//	SaveBatchPointCloud(img_2D, img_3D, zValues, intensityValues, image_width, image_height);
//
//	thread thread_fo_3D = thread(main_recognise_for_point_cloud_3D, ref(img_3D), ref(img_2D), ref(out), flat);
//	//thread thread_fo_2D = thread(main_recognise_for_point_cloud_2D, ref(img_2D), ref(out), flat);
//
//	thread_fo_3D.join();
//	//thread_fo_2D.join();
//}


int main()
{

	// 新建保存文件
	string savepath = "./result";

	if (_access(savepath.c_str(), 0) == -1)
	{
		_mkdir(savepath.c_str());
	}

	for (int i = 0; i < 25; i++)
	{
		string filename_i = "./dataset_0125_0/" + to_string(i) + "_i" + ".csv";
		string filename_z = "./dataset_0125_0/" + to_string(i) + "_z" + ".csv";

		Mat img_2D, img_3D;
		int x_rate = 5593;
		int y_rate = 33333;

		// 构图时间
		clock_t start = clock();

		read_csv_2D(filename_i, img_2D);
		read_csv_3D(filename_z, img_3D);

		clock_t end = clock();
		printf("creat image total using time is %d s\n", (end - start) / CLOCKS_PER_SEC);

		//show_image(img_2D, 3);

		clock_t start1 = clock();
		for (int j = 0; j < 1; j++)
		{
			vector<int> max_compare;
			vector<Mat> img_res;
			vector<flawimformation>flaw_total;

			// 声明输出结果
			recogn_OUT_for_point_cloud out(max_compare, img_res, flaw_total);

			// 调用识别函数
			main_recognise_for_point_cloud(img_3D, img_2D, out, 0);

			/*for (int l = 0; l < out.flaw_total.size() && l < 3; l++)
			{
				cout << "this flaw depth is " << out.flaw_total[l].depth << endl;
			}*/

			// 保存识别图片
			imwrite(savepath + '/' + (to_string(i) + "_" + "3D.bmp"), out.image[0]);
			//imwrite(savepath + '/' + (to_string(i) + "_" + "2D.bmp"), out.image[1]);
		}

		// 算法识别用时
		clock_t end1 = clock();
		printf("%d algorithm using time is %d s\n", i, (end1 - start1) / CLOCKS_PER_SEC);
	}

	return 0;
}
