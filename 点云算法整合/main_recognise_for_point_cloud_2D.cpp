#include"main_recognise_for_point_cloud_2D.h"

/// <summary>
/// 筛选2D图像的候选区域（函数一）
/// </summary>
/// <param name="image_src"></param>
/// <param name="image"></param>
/// <param name="min_precent"></param>
/// <param name="max_precent"></param>
void remove_px_double(Mat& image_src, Mat& image, int min_precent, int max_precent)
{

	vector<int>px_total;

	int px_num = 0;

	statistics(image_src, px_total, px_num);

	int min_num = px_num * min_precent / 100;
	int max_num = px_num * max_precent / 100;

	int min_sum = 0;
	int max_sum = 0;

	int min_px = 1;
	int max_px = 255;

	while (min_sum < min_num)
	{
		min_sum += px_total[min_px];
		min_px += 1;
	}

	while (max_sum < max_num)
	{
		max_sum += px_total[max_px];
		max_px -= 1;
	}

	image = Mat::zeros(image_src.size(), CV_8UC1);

	int image_row = image_src.rows;
	int image_col = image_src.cols;

	for (int i = 0; i < image_row; i++)
	{
		uchar* data = image_src.ptr<uchar>(i);
		for (int j = 0; j < image_col; j++)
		{
			if (data[j] && (data[j]<min_px || data[j]>max_px))
			{
				image.at<uchar>(i, j) = 1;
			}
		}
	}
}

/// <summary>
/// 对图片像素进行排序
/// </summary>
/// <param name="px"></param>
/// <param name="px_sort"></param>
void argsort(vector<int>px, vector<int>& px_sort)
{

	px_sort = vector<int>(px.size());

	int px_num = px.size();

	int old_j = 0;

	for (int l = 0; l < px_num; l++)
	{
		int j = old_j;
		for (int i = 0; i < px_num; i++)
		{
			if (px[l] > px[i])
			{
				j += 1;
			}
		}
		if (px_sort[j] != 0)
		{
			while (px_sort[j] != 0)
			{
				j += 1;
			}
		}
		px_sort[j] = l;

	}
}

/// <summary>
///  筛选2D图像的候选区域（函数二）
/// </summary>
/// <param name="image"></param>
/// <param name="image_res"></param>
/// <param name="threshold"></param>
void adapt_remove_px(Mat& image, Mat& image_res, int threshold)
{

	int image_row = image.rows;
	int image_col = image.cols;

	int box_weight = 100;
	int box_hight = box_weight;

	int step = box_weight / 5;

	int image_row_ = image_row - box_weight;
	int image_col_ = image_col - box_hight;

	image_res = Mat::zeros(image.size(), CV_8UC1);

	for (int i = 0; i < image_row_; i += step)
	{
		for (int j = 0; j < image_col_; j += step)
		{

			Rect rect(j, i, box_weight, box_hight);

			Mat ROI = Mat(image, rect);

			vector<int>px_total, px_sort;

			int px_num = 0;

			statistics(ROI, px_total, px_num);

			argsort(px_total, px_sort);

			int px_max = px_num * threshold / 100;

			int px_i = 255;

			int px_sum = 0;

			while (px_sum < px_max)
			{
				px_sum += px_total[px_sort[px_i]];
				px_i -= 1;
			}

			vector<int>px_list = vector<int>(256);

			for (int i = 0; i < px_i; i++)
			{
				px_list[px_sort[i]] = 1;
			}

			for (int x = 0; x < box_weight; x++)
			{
				uchar* data = ROI.ptr<uchar>(x);
				for (int y = 0; y < box_hight; y++)
				{
					if (data[y] && px_list[data[y]])
					{
						image_res.at<uchar>(x + i, y + j) = 1;
					}
				}
			}

		}
	}

}

/// <summary>
/// 把掩膜上的标记显示到图片上
/// </summary>
/// <param name="mask"></param>
/// <param name="image_src"></param>
/// <param name="image"></param>
void mask_to_image(Mat& mask, Mat& image_src, Mat& image)
{

	cvtColor(image_src, image, COLOR_GRAY2BGR);

	int image_row = image.rows;
	int image_col = image.cols;

	for (int i = 0; i < image_row; i++)
	{
		uchar* data = mask.ptr<uchar>(i);
		for (int j = 0; j < image_col; j++)
		{
			if (data[j])
			{
				image.at<Vec3b>(i, j)[0] = 0;
				image.at<Vec3b>(i, j)[1] = 255;
				image.at<Vec3b>(i, j)[2] = 0;
			}
		}
	}
}

/// <summary>
/// 根据对比度筛选缺陷
/// </summary>
/// <param name="image_src"></param>
/// <param name="image_mask"></param>
/// <param name="mask"></param>
/// <param name="flaw_total"></param>
/// <param name="gan"></param>
/// <param name="px_threshold"></param>
/// <param name="postion"></param>
void deep_recognise(Mat& image_src, Mat& image_mask, Mat& mask, vector<flawimformation>& flaw_total, int gan, int px_threshold, int postion)
{

	int image_row = image_src.rows;
	int image_col = image_src.cols;

	vector<vector<Point>>contours;

	vector<Vec4i>hierarchy;

	findContours(image_mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

	int contour_length = contours.size();

	Mat draw_image_edge = Mat::zeros(image_mask.size(), CV_8UC1);

	Mat draw_image_out = Mat::zeros(image_mask.size(), CV_8UC1);

	mask = Mat::zeros(image_mask.size(), CV_8UC1);

	int px_flaw = 0, px_out = 0;

	int type;

	for (int i = 0; i < contour_length; i++)
	{

		drawContours(draw_image_edge, contours, i, 1, -1);
		drawContours(draw_image_out, contours, i, 1, gan);

		bitwise_or(draw_image_edge, draw_image_out, draw_image_out);

		px_flaw = median(image_src, draw_image_edge);

		bitwise_xor(draw_image_edge, draw_image_out, draw_image_out);

		px_out = median(image_src, draw_image_out);

		if (abs(px_flaw - px_out) > px_threshold)
		{
			drawContours(mask, contours, i, 1, -1);

			Point2f center;
			float radius;
			double area = contourArea(contours[i]);
			minEnclosingCircle(contours[i], center, radius);

			double area_real = area * 0.004717;

			double length = radius * 2 * 0.01325;

			if (px_out > 128)// 大白为缺肉
			{
				type = 1;
			}
			else// 大黑为清洁度
			{
				type = 3;
			}

			flaw_total.push_back(flawimformation{ contours[i], center,area_real,length ,postion,type ,0 });
		}
	}
}


int median(Mat& image_src, Mat& draw_image_mask)
{

	int image_row = image_src.rows;
	int image_col = image_src.cols;

	int num = 0;

	vector<int>px;

	for (int i = 0; i < image_row; i++)
	{
		uchar* data = draw_image_mask.ptr<uchar>(i);
		for (int j = 0; j < image_col; j++)
		{
			if (data[j])
			{
				px.push_back(image_src.at<uchar>(i, j));
			}
		}
	}
	sort(px.begin(), px.end());

	if (px.size() % 2)
	{
		num = px[px.size() / 2];
	}
	else
	{
		num = (px[px.size() / 2] + px[px.size() / 2 + 1]) / 2;
	}

	return num;
}

/// <summary>
/// 统计像素个数
/// </summary>
/// <param name="image"></param>
/// <param name="px"></param>
/// <param name="num"></param>
void statistics(Mat& image, vector<int>& px, int& num)
{

	px = vector<int>(256, 0);

	int image_row = image.rows;
	int image_col = image.cols;

	for (int i = 0; i < image_row; i++)
	{
		uchar* data = image.ptr<uchar>(i);
		for (int j = 0; j < image_col; j++)
		{
			if (data[j])
			{
				px[data[j]] += 1;
				num += 1;
			}
		}
	}
}

/// <summary>
///  去除连通域较小的缺陷
/// </summary>
/// <param name="image"></param>
/// <param name="mask"></param>
/// <param name="area_max"></param>
void remove_small_objects(Mat& image, Mat& mask, int area_max)
{

	vector<vector<Point>>contours;
	vector<Vec4i>hierarchy;

	findContours(image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	int contours_length = contours.size();

	mask = Mat::zeros(image.size(), CV_8UC1);

	for (int i = 0; i < contours_length; i++)
	{
		int area = contourArea(contours[i]);

		if (area > area_max)
		{
			drawContours(mask, contours, i, 1, -1);
		}
	}
}


void show_compare_px(Mat& image_src, Mat& mask, Mat& img, int& max_compare)
{

	cvtColor(image_src, img, COLOR_GRAY2BGR);

	int image_row = image_src.rows;
	int image_col = image_src.cols;

	vector<vector<Point>>contours;

	vector<Vec4i>hierarchy;

	findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

	int contour_length = contours.size();

	Mat draw_image_edge = Mat::zeros(image_src.size(), CV_8UC1);

	Mat draw_image_out = Mat::zeros(image_src.size(), CV_8UC1);

	int px_flaw = 0, px_out = 0, value;

	Point2f center;
	float radius;

	int x = 0, y = 0;

	for (int i = 0; i < contour_length; i++)
	{

		drawContours(draw_image_edge, contours, i, 1, 1);
		drawContours(draw_image_out, contours, i, 1, 2);

		px_flaw = median(image_src, draw_image_edge);

		bitwise_xor(draw_image_edge, draw_image_out, draw_image_out);

		px_out = median(image_src, draw_image_out);

		value = abs(px_flaw - px_out);

		if (value > max_compare)
		{
			max_compare = value;
		}

		minEnclosingCircle(contours[i], center, radius);

		x = center.x;
		y = center.y;

		putText(img, to_string(px_flaw), Point(x, y - 5),
			FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 0, 0), 1);

		putText(img, to_string(px_flaw), Point(x, y + 5),
			FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255), 1);

		drawContours(img, contours, i, Scalar(0, 255, 0), 1);
	}
}

/// <summary>
///  将2D上的缺陷输出
/// </summary>
/// <param name="image"></param>
/// <param name="image_res"></param>
/// <param name="flaw_total"></param>
/// <param name="roi_min"></param>
/// <param name="roi_max"></param>
/// <param name="min_threshold"></param>
/// <param name="max_threshold"></param>
/// <param name="position"></param>
/// <param name="max_compare"></param>
void recognise_flaw_for_point_cloud_2D(Mat& image, Mat& image_res, vector<flawimformation>& flaw_total, int roi_min, int roi_max, int min_threshold, int max_threshold, int position, int& max_compare)
{

	int image_row = image.rows;
	int image_col = image.cols;

	if (image_col < roi_min || image_col < roi_max)
	{
		return;
	}

	int min_size = AREA_point_cloud_for_2D;

	int px_threshold = COMPARE_point_cloud_for_2D;

	Rect rect(roi_min, 0, roi_max - roi_min, image_row);

	Mat ROI = Mat(image, rect);
	//cvtColor(ROI, ROI, COLOR_BGR2GRAY);

	Mat image_remove_px;
	//clock_t start1 = clock();
	//for (int p = 0; p < 10; p++)
	adapt_remove_px(ROI, image_remove_px, 98);
	//clock_t end1 = clock();
	//printf("part 1 using %d s\n", (end1 - start1) / CLOCKS_PER_SEC);

	Mat image_remove_small;
	//clock_t start2 = clock();
	//for (int p = 0; p < 10; p++)
	remove_small_objects(image_remove_px, image_remove_small, min_size);
	//clock_t end2 = clock();
	//printf("part 2 using %d s\n", (end2 - start2) / CLOCKS_PER_SEC);

	Mat image_1;
	mask_to_image(image_remove_small, ROI, image_1);

	int gan = 10;

	Mat image_deep_recognise;
	//clock_t start3 = clock();
	//for (int p = 0; p < 10; p++)
	deep_recognise(ROI, image_remove_small, image_deep_recognise, flaw_total, gan, px_threshold, position);
	//clock_t end3 = clock();
	//printf("part 3 using  %d s\n", (end3 - start3) / CLOCKS_PER_SEC);

	Mat image_2;
	//int max_compare = 0;
	show_compare_px(ROI, image_deep_recognise, image_2, max_compare);

	mask_to_image_result(image_res, image_deep_recognise, roi_min);

}

/// <summary>
/// 显示缺陷位置
/// </summary>
/// <param name="image_src"></param>
/// <param name="mask"></param>
/// <param name="roi_min"></param>
void mask_to_image_result(Mat& image_src, Mat mask, int roi_min)
{

	vector<vector<Point>>contours;
	vector<Vec4i>hierarchy;

	findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

	int contours_size = contours.size();

	for (int i = 0; i < contours_size; i++)
	{
		int contour_size = contours[i].size();
		for (int j = 0; j < contour_size; j++)
		{
			int x = contours[i][j].x + roi_min;
			int y = contours[i][j].y;

			image_src.at<Vec3b>(y, x)[0] = 0;
			image_src.at<Vec3b>(y, x)[1] = 255;
			image_src.at<Vec3b>(y, x)[2] = 0;

		}
	}
}

/// <summary>
/// 2D识别缺陷主函数
/// </summary>
/// <param name="image"></param>
/// <param name="OUT"></param>
/// <param name="flat"></param>
/// <returns></returns>
bool main_recognise_for_point_cloud_2D(Mat& image, recogn_OUT_for_point_cloud& OUT, int flat)
{

	vector<int>ROI;

	int border = 10;

	int min_threshold = 2;
	int max_threshold = 2;

	image_auto_cut(image, ROI, border, flat);

	int ROI_size = ROI.size();

	Mat image_res;
	cvtColor(image, image_res, COLOR_GRAY2BGR);

	int max_compare = 0;

	thread* threads = new thread[ROI_size];

	for (int i = 0; i < ROI_size; i += 2)
	{
		//recognise_flaw_for_point_cloud_2D(image, image_res, OUT.flaw_total
			//, ROI[i] + border, ROI[i + 1] - border, min_threshold, max_threshold, i / 2, max_compare);
		threads[i / 2] = thread(recognise_flaw_for_point_cloud_2D, ref(image), ref(image_res), ref(OUT.flaw_total)
			, ROI[i] + border, ROI[i + 1] - border, min_threshold, max_threshold, i / 2, ref(max_compare));
	}

	for (int i = 0; i < ROI_size; i += 2)
	{
		threads[i / 2].join();
	}

	OUT.image.push_back(image_res);
	OUT.max_compare.push_back(max_compare);

	//show_image(OUT.image, 3);

	return true;
}