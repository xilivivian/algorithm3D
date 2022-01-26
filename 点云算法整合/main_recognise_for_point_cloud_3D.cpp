#include"main_recognise_for_point_cloud_3D.h"

/// <summary>
/// 3D点云拉平函数
/// </summary>
/// <param name="img_3D"></param>
/// <param name="list_px"></param>
/// <param name="roi_min"></param>
/// <param name="roi_max"></param>
void compensate(Mat img_3D, vector<float>& list_px, int roi_min, int roi_max)
{

	int img_3D_row = img_3D.rows;

	Rect rect(roi_min, 0, roi_max - roi_min, img_3D_row);

	Mat img = Mat(img_3D, rect);

	int img_row = img.rows;
	int img_col = img.cols;

	Mat sort_mask = Mat::zeros(img.size(), CV_16FC1);

	img.copyTo(sort_mask);

	for (int i = 0; i < img_row; i++)
	{

		float* data = sort_mask.ptr<float>(i);

		sort(data, data + img_col);

		list_px.push_back(data[img_col / 2]);

	}

	sort(list_px.begin(), list_px.end());

	float line_px_median = list_px[list_px.size() / 2];
}


/// <summary>
/// RANSAC算法识别（分区域识别区域）
/// </summary>
/// <param name="image"></param>
/// <param name="image_mask"></param>
/// <param name="ROI_min"></param>
/// <param name="ROI_max"></param>
void ransac_recognise(Mat& image, Mat& image_mask, int ROI_min, int ROI_max)
{

	int image_row = image.rows;
	int image_col = image.cols;

	if (image_col < ROI_min || image_col < ROI_max)
	{
		return;
	}

	int step = ransac_step;

	int step_size = image_row / step;

	srand((unsigned)time(NULL));

	//show_image(image, 3);

	int times = ransac_times;

	int number = 3;

	float threshold_value = ransac_threshold;

	float good_A, good_B, good_C, good_D;

	int max_num, sum_num;

	int ROI_row, ROI_col;

	Mat ROI;

	float x0, x1, x2;

	float y0, y1, y2;

	float z0, z1, z2;

	float A, B, C, D;

	float pre, loss;

	int start, end;

	for (int i = 0; i < step; i++)
	{
		start = i * step_size;
		if (i != (step - 1))
		{
			end = (i + 1) * step_size;
		}
		else
		{
			end = image_row;
		}

		ROI_row = end - start;
		ROI_col = ROI_max - ROI_min;

		max_num = 0;

		// 0.004s
		for (int j = 0; j < times;)
		{

			x0 = float(rand() % ROI_row);
			x1 = float(rand() % ROI_row);
			x2 = float(rand() % ROI_row);

			y0 = float(rand() % ROI_col);
			y1 = float(rand() % ROI_col);
			y2 = float(rand() % ROI_col);

			z0 = image.at<float>(int(x0) + start, int(y0) + ROI_min);
			z1 = image.at<float>(int(x1) + start, int(y1) + ROI_min);
			z2 = image.at<float>(int(x2) + start, int(y2) + ROI_min);

			if ((y2 - y1) / (x2 - x1) != (y1 - y0) / (x1 - x0))
			{

				j++;

				A = (y0 - y1) * (z0 - z2) - (z0 - z1) * (y0 - y2);
				B = (z0 - z1) * (x0 - x2) - (x0 - x1) * (z0 - z2);
				C = (x0 - x1) * (y0 - y2) - (y0 - y1) * (x0 - x2);

				D = A * x0 + B * y0 + C * z0;

				sum_num = 0;
				for (int x = 0; x < ROI_row; x++)
				{
					float* data = image.ptr<float>(x + start);
					for (int y = 0; y < ROI_col; y++)
					{
						pre = (D - A * x - B * y) / C;

						if (abs(pre - data[y + ROI_min]) < threshold_value)
						{
							sum_num += 1;
						}
					}
				}

				if (sum_num > max_num)
				{
					max_num = sum_num;
					good_A = A;
					good_B = B;
					good_C = C;
					good_D = D;
				}
			}
		}

		//cout << "point in area max num is " << max_num << endl;

		// 0.00009s
		for (int x = 0; x < ROI_row; x++)
		{
			float* data = image_mask.ptr<float>(x + start);
			for (int y = 0; y < ROI_col; y++)
			{
				pre = (good_D - good_A * x - good_B * y) / good_C;

				loss = pre - image.at<float>(x + start, y + ROI_min);

				if (!data[y + ROI_min] || data[y + ROI_min] > abs(loss))
				{
					data[y + ROI_min] = loss;
				}
			}
		}
	}
}


/// <summary>
/// RANSAC总函数
/// </summary>
/// <param name="img_3D"></param>
/// <param name="mask"></param>
/// <param name="image_mask"></param>
/// <param name="px_partition"></param>
/// <param name="threshold_hight"></param>
void ransac_partiton(Mat img_3D, Mat& mask, Mat& image_mask, vector<int>& px_partition, double threshold_hight)
{

	int image_row = img_3D.rows;
	int image_col = img_3D.cols;

	image_mask = Mat::zeros(img_3D.size(), CV_32FC1);

	int px_partition_length = px_partition.size();

	thread* threads = new thread[px_partition_length / 2];

	for (int i = 0; i < px_partition_length; i += 4)
	{
		//ransac_recognise(img_3D, image_mask, px_partition[i], px_partition[i + 1]);
		threads[i / 2] = thread(ransac_recognise, ref(img_3D), ref(image_mask), px_partition[i], px_partition[i + 1]);
	}

	for (int i = 0; i < px_partition_length; i += 4)
	{
		threads[i / 2].join();
	}


	for (int i = 2; i < px_partition_length; i += 4)
	{
		//ransac_recognise(img_3D, image_mask, px_partition[i], px_partition[i + 1]);
		threads[i / 2] = thread(ransac_recognise, ref(img_3D), ref(image_mask), px_partition[i], px_partition[i + 1]);
	}

	for (int i = 2; i < px_partition_length; i += 4)
	{
		threads[i / 2].join();
	}

	// 0.014ms
	mask = Mat::zeros(img_3D.size(), CV_8UC1);
	for (int i = 5; i < image_row - 5; i++)
	{
		float* data_image_mask = image_mask.ptr<float>(i);
		for (int j = 300; j < image_col; j++)
		{
			if (j >= 800 && j < 840)
			{
				continue;
			}

			float px = img_3D.at<float>(i, j);
			if (abs(data_image_mask[j]) > threshold_hight && px != invalid_zvalue && abs(data_image_mask[j]) < IMPOSSIBLE_ZVALUE_THRESHOLD)
			{
				mask.at<uchar>(i, j) = 255;
			}
		}
	}
}


/// <summary>
/// 获取缺陷所有的深度
/// </summary>
/// <param name="image_src"></param>
/// <param name="draw_image_mask"></param>
/// <param name="px"></param>
void get_deep_total(Mat& image_src, Mat& draw_image_mask, vector<float>& px)
{

	int image_row = image_src.rows;
	int image_col = image_src.cols;

	for (int i = 0; i < image_row; i++)
	{
		uchar* data = draw_image_mask.ptr<uchar>(i);
		for (int j = 0; j < image_col; j++)
		{
			if (data[j])
			{
				float number = image_src.at<float>(i, j);
				px.push_back(number);
			}
		}
	}
}


/// <summary>
///  获取所有缺陷的深度，并输出结果
/// </summary>
/// <param name="image_mask"></param>
/// <param name="img_2D_show"></param>
/// <param name="out"></param>
/// <param name="px_partition"></param>
/// <param name="img_3D"></param>
/// <param name="img_2D"></param>
void get_depth(Mat& image_mask, Mat& img_2D_show, recogn_OUT_for_point_cloud& out, vector<int>& px_partition, Mat& img_3D, Mat& img_2D)
{

	int image_row = image_mask.rows;
	int image_col = image_mask.cols;

	vector<vector<Point>>contours;
	vector<Vec4i>hierarchy;

	findContours(image_mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	int contours_length = contours.size();

	cvtColor(img_2D, img_2D_show, COLOR_GRAY2BGR);

	int max_size = point_cloud_area_max;
	int min_size = point_cloud_area_min;
	int depth = 0;
	int type = 0;

	int px_partition_size = px_partition.size();

	for (int i = 0; i < contours_length; i++)
	{

		int point_size = contourArea(contours[i]);

		if (point_size<min_size || point_size>max_size)
		{
			continue;
		}

		Mat mask = Mat::zeros(image_mask.size(), CV_8UC1);

		drawContours(mask, contours, i, 1, -1);

		vector<float>deep_total;

		get_deep_total(img_3D, mask, deep_total);

		sort(deep_total.begin(), deep_total.end());

		double num_max = deep_total[deep_total.size() - 1];
		double num_min = deep_total[0];

		double depth = 0.0;

		if (abs(num_max) > abs(num_min))
		{
			depth = num_max;
		}
		else
		{
			depth = num_min;
		}

		/*if (abs(depth) < 0.02)
		{
			continue;
		}*/

		Point2f center;
		float radius;

		minEnclosingCircle(contours[i], center, radius);

		drawContours(img_2D_show, contours, i, Scalar(0, 255, 0), 1);

		double area = contourArea(contours[i]);

		if (depth > 0)
		{
			type = 2;
		}
		else
		{
			type = 1;
		}

		int position = int(center.x) % 7;

		for (int j = 0; j < px_partition_size; j += 2)
		{
			if (px_partition[j] > center.x && px_partition[j + 1] > center.x)
			{
				position = (j + 1) / 2;
			}
		}

		out.flaw_total.push_back(flawimformation{ contours[i],center,area,(2 * radius),position,type,(depth / 1000.0) });

		putText(img_2D_show, to_string(depth / 1000.0), Point(int(center.x), int(center.y)),
			FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0, 0, 255), 1);
	}
	//out.image.push_back(img_2D_show);
}


/// <summary>
/// 移除盲区的无效数据
/// </summary>
/// <param name="mask"></param>
/// <param name="img_2D"></param>
/// <param name="roi_min"></param>
/// <param name="roi_max"></param>
/// <param name="threshold_px"></param>
void remove_mask_px(Mat& mask, Mat& img_2D, int roi_min, int roi_max, int threshold_px)
{

	int image_row = mask.rows;
	int image_col = mask.cols;

	int num = 5;

	for (int i = 0; i < image_row; i++)
	{

		vector<int>left_stack;
		vector<int>right_stack;

		int left = roi_min;
		int right = roi_max;

		uchar* data = img_2D.ptr<uchar>(i);

		while (left_stack.size() < num && left < roi_max)
		{

			if (left_stack.size() && data[left] > threshold_px)
			{
				left_stack.clear();
			}
			else
			{
				left_stack.push_back(left);
			}
			left += 1;
		}

		if (left_stack.size())
		{
			left = left_stack[0];
		}

		while (right_stack.size() < num && right < roi_max)
		{

			if (right_stack.size() && data[right] > threshold_px)
			{
				right_stack.clear();
			}
			else
			{
				right_stack.push_back(right);
			}
			right -= 1;
		}

		if (right_stack.size())
		{
			right = right_stack[0];
		}

		//printf("middle px is %d %d\n", left, right);

		for (int j = left; j < right; j++)
		{
			mask.at<uchar>(i, j) = 0;
		}

	}
}

//
//void cut_long(Mat& mask, int flaw_length)
//{
//
//	int image_row = mask.rows;
//	int image_col = mask.cols;
//
//	vector<vector<Point>>contours;
//	vector<Vec4i>hierarchy;
//
//	findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
//
//	int contours_length = contours.size();
//
//	for (int i = 0; i < contours_length; i++)
//	{
//		int length = arcLength(contours[i], true);
//
//
//
//	}
//
//}


/// <summary>
/// 裁切长度较长的候选缺陷
/// </summary>
/// <param name="mask"></param>
/// <param name="px_length"></param>
/// <param name="threshold_length"></param>
//void cut_long_flaw(Mat& mask, int flaw_length, int flaw_weight)
//{
//
//	int image_row = mask.rows;
//	int image_col = mask.cols;
//
//	vector<vector<Point>>contours;
//	vector<Vec4i>hierarchy;
//
//	findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
//
//	int contours_length = contours.size();
//
//	for (int i = 0; i < contours_length; i++)
//	{
//
//		RotatedRect retval = minAreaRect(contours[i]);
//
//		float tank = abs(retval.angle);
//
//		int length = arcLength(contours[i], true);
//
//		if (tank > 10.0 && tank < 80.0)
//		{
//			continue;
//		}
//
//		if (length > 200)
//		{
//
//			Point2f vertices[4];
//
//			retval.points(vertices);
//
//			float start_x = vertices[0].x, start_y = vertices[0].y, end_x = vertices[0].x, end_y = vertices[0].y;
//
//			for (int l = 0; l < 4; l++)
//			{
//				start_x = min(start_x, vertices[l].x);
//				start_y = min(start_y, vertices[l].y);
//				end_x = max(end_x, vertices[l].x);
//				end_y = max(end_y, vertices[l].y);
//			}
//
//			start_x = max(start_x, 0.0f);
//			start_y = max(start_y, 0.0f);
//			end_x = min(end_x, float(image_col));
//			end_y = min(end_y, float(image_row));
//
//			for (int p = start_y; p < end_y; p++)
//			{
//
//				int num_start = start_x;
//				bool flat = true;
//
//				uchar* data = mask.ptr<uchar>(p);
//
//				for (int j = start_x; j < end_x; j++)
//				{
//					data[j] = 0;
//
//				}
//			}
//		}
//	}
//}


void cut_long(Mat& mask, int roi_min, int roi_max, int flaw_length_min)
{

	int image_row = mask.rows;
	int image_col = mask.cols;

	for (int i = 0; i < image_row; i++)
	{

		bool flat = true;
		int start = roi_min;
		uchar* data = mask.ptr<uchar>(i);

		for (int j = roi_min; j < roi_max; j++)
		{

			if (data[j])
			{
				if (flat)
				{
					start = j;
					flat = false;
				}
			}
			else
			{
				if (!flat)
				{
					if ((j - start) < flaw_length_min)
					{
						for (int l = start; l < j; l++)
						{
							data[l] = 0;
						}
					}
					flat = true;
				}
			}
		}

		if (!flat)
		{
			if ((roi_max - start) < flaw_length_min)
			{
				for (int l = start; l < roi_max; l++)
				{
					data[l] = 0;
				}
			}
		}
	}
}


///// <summary>
///// 裁切长度较长的候选缺陷
///// </summary>
///// <param name="mask"></param>
///// <param name="px_length"></param>
///// <param name="threshold_length"></param>
//void cut_long_flaw(Mat& mask, int flaw_length, int flaw_weight)
//{
//
//	int image_row = mask.rows;
//	int image_col = mask.cols;
//
//	vector<vector<Point>>contours;
//	vector<Vec4i>hierarchy;
//
//	findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
//
//	int contours_length = contours.size();
//
//	for (int i = 0; i < contours_length; i++)
//	{
//
//		RotatedRect retval = minAreaRect(contours[i]);
//
//		float tank = abs(retval.angle);
//
//		int length = arcLength(contours[i], true);
//
//		if (tank > 20.0 && tank < 70.0)
//		{
//			continue;
//		}
//
//		if (length > flaw_length)
//		{
//
//			Point2f vertices[4];
//
//			retval.points(vertices);
//
//			float start_x = vertices[0].x, start_y = vertices[0].y, end_x = vertices[0].x, end_y = vertices[0].y;
//
//			for (int l = 0; l < 4; l++)
//			{
//				start_x = min(start_x, vertices[l].x);
//				start_y = min(start_y, vertices[l].y);
//				end_x = max(end_x, vertices[l].x);
//				end_y = max(end_y, vertices[l].y);
//			}
//
//			start_x = max(start_x, 0.0f);
//			start_y = max(start_y, 0.0f);
//			end_x = min(end_x, float(image_col));
//			end_y = min(end_y, float(image_row));
//
//			for (int p = start_y; p < end_y; p++)
//			{
//
//				int num_start = start_x;
//				bool flat = true;
//
//				uchar* data = mask.ptr<uchar>(p);
//
//				for (int j = start_x; j < end_x; j++)
//				{
//
//					if (data[j])
//					{
//						if (flat)
//						{
//							num_start = j;
//							flat = false;
//						}
//					}
//					else
//					{
//						if (!flat)
//						{
//							if ((j - num_start) < flaw_weight)
//							{
//								for (int l = num_start; l < j; l++)
//								{
//									data[l] = 0;
//								}
//							}
//							flat = true;
//						}
//					}
//				}
//
//				if (!flat && (end_x - num_start) < flaw_weight)
//				{
//					for (int l = num_start; l < end_x; l++)
//					{
//						data[l] = 0;
//					}
//				}
//			}
//		}
//	}
//}


void cut_long_flaw(Mat& mask, int threshold_weigth, int threshold_length)
{

	int image_row = mask.rows;
	int image_col = mask.cols;

	vector<vector<Point>>contours;
	vector<Vec4i>hierarchy;

	findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	int contours_length = contours.size();

	for (int i = 0; i < contours_length; i++)
	{

		RotatedRect retval = minAreaRect(contours[i]);

		float tank = abs(retval.angle);

		Rect rect = retval.boundingRect();

		int width = rect.width;
		int height = rect.height;

		int length = arcLength(contours[i], true);

		if (width < threshold_weigth && (length > threshold_length || (tank < 5.0 || tank>85.0)))
		{
			drawContours(mask, contours, i, Scalar(0), -1);
		}

	}


}


/// <summary>
/// 3D识别主程序
/// </summary>
/// <param name="img_3D"></param>
/// <param name="img_2D"></param>
/// <param name="out"></param>
/// <param name="flat"></param>
void main_recognise_for_point_cloud_3D(Mat& img_3D, Mat& img_2D, recogn_OUT_for_point_cloud& out, int flat)
{

	int image_row = img_3D.rows;
	int image_col = img_3D.cols;

	vector<int>px_partition;

	switch (flat)
	{
	case 0: {
		/*px_partition = { 0, 450, 350, 550, 500, 950,
				800, 1500, 1350, 1650, 1550, 1850, 1670, image_col };
		  px_partition = { 0, 270, 230, 400, 330, 810,
				730, 1270, 1200, 1450, 1370, 1630, 1570, image_col };
		px_partition = { 0, 370, 320, 480, 480, 840,
					800, 1330, 1280, 1550, 1450, 1720, 1670, image_col }; */
		px_partition = { 50, 350, 275, 460, 500, 900,
					800, 1350, 1250, 1550, 1450, 1720, 1620, image_col };

	}break;
	case 1: {
		px_partition = { 0, 450, 350, 550, 500, 950,
				800, 1500, 1350, 1650, 1550, 1850, 1670, image_col };

	}break;
	default:
		break;
	}

	Mat mask, image_mask;

	double threshold_hight = candidate_depth_threshold;

	ransac_partiton(img_3D, mask, image_mask, px_partition, threshold_hight);


	//show_image(mask, 3);

	cut_long(mask, 410, 460, 10);

	cut_long(mask, 1650, 1700, 10);

	//remove_mask_px(mask, img_2D, 1640, 1700, middle_Invalid_px_threshold);

	cut_long_flaw(mask, flaw_weight_threshold, flaw_length_threshold);

	Mat mask_rgb;
	cvtColor(mask, mask_rgb, COLOR_GRAY2RGB);

	//show_image(mask, 3);

	Mat img2D_show;

	get_depth(mask, img2D_show, out, px_partition, image_mask, img_2D);

	Mat image_out;
	hstack(mask_rgb, img2D_show, image_out);

	out.image.push_back(image_out);

}

