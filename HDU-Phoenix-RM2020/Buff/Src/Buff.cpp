#include "Buff.h"
/// \brief 构造函数
Detect::Detect() {
	lenet = cv::dnn::readNetFromCaffe(lenet_txt_file, lenet_model_file);
	//yolo_lite = cv::dnn::readNetFromDarknet(yolo_txt_file, yolo_model_file);
}

/// \brief 清空数据
void Detect::clear() {
	mode = 0;
	dirFlag = false;
	frame_cnt = 0;
	memset(&lastData, 0, sizeof(armorData));
	memset(&lostData, 0, sizeof(armorData));
}

/// \brief 保证用 rect 截图安全
/// \param rect 图中的roi范围
/// \param size 图的大小
bool Detect::makeRectSafe(const cv::Rect rect, const cv::Size size) {
	if (rect.x < 0)
		return false;
	if (rect.x + rect.width > size.width)
		return false;
	if (rect.y < 0)
		return false;
	if (rect.y + rect.height > size.height)
		return false;
	if (rect.width <= 0 || rect.height <= 0)
		return false;
	return true;
}

/// \brief 根据点集使用最小二乘法拟合圆
/// \param points 点集
/// \param R_center 圆心
bool Detect::circleLeastFit(const std::vector<cv::Point2f> &points, cv::Point2f &R_center) {
	float center_x = 0.0f;
	float center_y = 0.0f;
	float radius = 0.0f;
	if (points.size() < 3) {
		return false;
	}

	double sum_x = 0.0f, sum_y = 0.0f;
	double sum_x2 = 0.0f, sum_y2 = 0.0f;
	double sum_x3 = 0.0f, sum_y3 = 0.0f;
	double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

	int N = points.size();
	for (int i = 0; i < N; i++) {
		double x = points[i].x;
		double y = points[i].y;
		double x2 = x * x;
		double y2 = y * y;
		sum_x += x;
		sum_y += y;
		sum_x2 += x2;
		sum_y2 += y2;
		sum_x3 += x2 * x;
		sum_y3 += y2 * y;
		sum_xy += x * y;
		sum_x1y2 += x * y2;
		sum_x2y1 += x2 * y;
	}

	double C, D, E, G, H;
	double a, b, c;

	C = N * sum_x2 - sum_x * sum_x;
	D = N * sum_xy - sum_x * sum_y;
	E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
	G = N * sum_y2 - sum_y * sum_y;
	H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
	a = (H * D - E * G) / (C * G - D * D);
	b = (H * C - E * D) / (D * D - G * C);
	c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

	center_x = a / (-2);
	center_y = b / (-2);
	radius = sqrt(a * a + b * b - 4 * c) / 2;
	R_center = cv::Point2f(center_x, center_y);
	return true;
}

/// \brief 根据象限把旋转矩形的角度转为360
/// \param quadrant 象限
/// \param angle 原始角度
/// \param tran_angle 转化的角度
bool Detect::change_angle(const int quadrant, const float angle, float &tran_angle) {
	if (quadrant == 1) {
		tran_angle = angle;
	}
	else if (quadrant == 2) {
		tran_angle = 90 + 90 - angle;
	}
	else if (quadrant == 3) {
		tran_angle = 180 + angle;
	}
	else if (quadrant == 4) {
		tran_angle = 270 + 90 - angle;
	}
	else {
		std::cout << "象限为0" << std::endl;
		return false;
	}
	return true;
}


/// \brief 设置roi
/// \param src 原图
/// \param dect_src roi 区域
/// \param offset roi 偏移
bool Detect::setImage(const cv::Mat src, cv::Mat &dect_src, cv::Point2f &offset) {
	if (lastData.isFind == false) {
		dect_src = src;
	}
	else {
		float scale = 1.5;
		float lu_x = lastData.R_center.x - param.radius * scale;//转变为左上角的点
		float lu_y = lastData.R_center.y - param.radius * scale;
		cv::Rect2f rect(lu_x, lu_y, param.radius * 2 * scale, param.radius * 2 * scale);
		if (makeRectSafe(rect, src.size()) == false) {
			dect_src = src;
			offset = cv::Point2f(0, 0);
		}
		else {
			dect_src = src(rect);
			offset = rect.tl();//返回左上角的点
		}
	}
	return true;
}

/// \brief 二值化图像
/// \param src 原图
/// \param binary 得到的二值图
/// \param bMode 二值方法
bool Detect::setBinary(const cv::Mat src, cv::Mat &binary, int bMode) {
	if (src.empty() || src.channels() != 3) return false;
	cv::Mat gray, gray_binary, tempBinary;

	if (bMode == BGR) {
		// 灰度阈值二值
		cvtColor(src, gray, cv::COLOR_BGR2GRAY);
		threshold(gray, gray_binary, 50, 255, cv::THRESH_BINARY);
		imshow("grayBinary", gray_binary);

		// 红蓝通道相减
		std::vector<cv::Mat> splited;
		split(src, splited);
		if (mode == RED_CLOCK || mode == RED_ANCLOCK || mode == RED_STATIC) {
			subtract(splited[2], splited[0], tempBinary);//红-蓝
			threshold(tempBinary, tempBinary, 135, 255, THRESH_BINARY);
		}
		else if (mode == BLUE_CLOCK || mode == BLUE_ANCLOCK || mode == BLUE_STATIC) {
			subtract(splited[0], splited[2], tempBinary);//蓝-红
			threshold(tempBinary, tempBinary, 60, 255, cv::THRESH_BINARY);
		}
		else {
			return false;
		}
		dilate(tempBinary, tempBinary, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
		imshow("tempBinary", tempBinary);
		// mask 操作
		binary = tempBinary & gray_binary;
	}
	else if (bMode == HSV) {// 如果明的话是v通道，暗的话可以直接用灰度图

	   // 亮度图
		cvtColor(src, gray, cv::COLOR_BGR2GRAY);
		threshold(gray, gray_binary, 80, 255, cv::THRESH_BINARY);

		// 颜色阈值分割
		cv::Mat imgHSV;
		cvtColor(src, imgHSV, cv::COLOR_BGR2HSV);
		if (mode == RED_ANCLOCK || mode == RED_CLOCK || mode == RED_STATIC) {
			cv::Mat temp;
			inRange(imgHSV, Scalar(0, 60, 80), Scalar(25, 255, 255), temp);
			inRange(imgHSV, Scalar(156, 60, 80), Scalar(181, 255, 255), tempBinary);
			tempBinary = temp | tempBinary;
		}
		else if (mode == BLUE_ANCLOCK || mode == BLUE_CLOCK || mode == BLUE_STATIC) {
			inRange(imgHSV, cv::Scalar(35, 46, 80), cv::Scalar(99, 255, 255), tempBinary);
		}
		else {
			return false;
		}
		imshow("tempBinary", tempBinary);
		dilate(tempBinary, tempBinary, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
		// mask 操作
		binary = tempBinary & gray_binary;
	}
	else if (bMode == BGR_useG) {
		cvtColor(src, gray, cv::COLOR_BGR2GRAY);
		threshold(gray, gray_binary, 80, 255, THRESH_BINARY);
		imshow("gray_binary", gray_binary);

		// 与绿通道相减
		std::vector<cv::Mat> splited;
		split(src, splited);
		if (mode == RED_CLOCK || mode == RED_ANCLOCK || mode == RED_STATIC) {
			subtract(splited[2], splited[1], tempBinary);
			threshold(tempBinary, tempBinary, 40, 255, THRESH_BINARY);
		}
		else if (mode == BLUE_CLOCK || mode == BLUE_ANCLOCK || mode == BLUE_STATIC) {
			subtract(splited[0], splited[1], tempBinary);
			threshold(tempBinary, tempBinary, 20, 255, THRESH_BINARY);
		}
		else {
			return false;
		}

		dilate(tempBinary, tempBinary, getStructuringElement(MORPH_RECT, Size(3, 3)));
		imshow("tempBinary", tempBinary);
		// mask 操作
		binary = tempBinary & gray_binary;
	}
	else if (bMode == OTSU) {
		// 大津算法
		cvtColor(src, gray, COLOR_BGR2GRAY);
		double test = threshold(gray, tempBinary, 0, 255, THRESH_OTSU);// 可以得出一个阈值
		cout << "test:" << test << endl;
		binary = tempBinary;
	}
	else if (bMode == GRAY) {
		// 灰度阈值
		cvtColor(src, gray, COLOR_BGR2GRAY);
		threshold(gray, gray_binary, 40, 255, THRESH_BINARY);
		binary = gray_binary;
	}
	else if (bMode == YCrCb) {

		Mat Ycrcb;
		cvtColor(src, Ycrcb, COLOR_BGR2YCrCb);
		vector<Mat> splited;
		split(Ycrcb, splited);

		// 亮度图
		threshold(splited[0], gray_binary, 60, 255, THRESH_BINARY);

		// cr和cb通道
		if (mode == RED_CLOCK || mode == RED_ANCLOCK || mode == RED_STATIC) {
			subtract(splited[1], splited[2], tempBinary);
			threshold(tempBinary, tempBinary, 20, 255, THRESH_BINARY);
		}
		else if (mode == BLUE_CLOCK || mode == BLUE_ANCLOCK || mode == BLUE_STATIC) {
			subtract(splited[2], splited[1], tempBinary);
			threshold(tempBinary, tempBinary, 40, 255, THRESH_BINARY);
		}
		else {
			return false;
		}

		dilate(tempBinary, tempBinary, getStructuringElement(MORPH_RECT, Size(3, 3)));
		// mask 操作
		binary = tempBinary & gray_binary;
	}
	else if (bMode == LUV) {
		Mat luv;
		cvtColor(src, luv, COLOR_BGR2Luv);
		vector<Mat> splited;
		split(luv, splited);

		// 亮度图
		threshold(splited[0], gray_binary, 60, 255, THRESH_BINARY);

		// 颜色阈值
		if (mode == RED_ANCLOCK || mode == RED_CLOCK || mode == RED_STATIC) {
			threshold(splited[2], tempBinary, 160, 255, THRESH_BINARY);
		}
		else if (mode == BLUE_ANCLOCK || mode == BLUE_CLOCK || mode == BLUE_STATIC) {
			threshold(splited[1], tempBinary, 70, 255, THRESH_BINARY_INV);

		}
		else {
			return false;
		}
		imshow("tempBinary", tempBinary);
		dilate(tempBinary, tempBinary, getStructuringElement(MORPH_RECT, Size(3, 3)));

		// mask操作
		binary = gray_binary & tempBinary;
	}
	else {
		return false;
	}

	return true;
}

/// \brief 检测装甲板
/// \param src 原图
/// \param bMode 二值方法
/// \param data 装甲板信息-
/// \param offset 偏移
bool Detect::getArmorCenter(const Mat src, const int bMode, armorData &data, Point2f offset) {

	/********************************* 二值化 ************************************/
	Mat binary;
	if (setBinary(src, binary, bMode) == false) return false;
	dilate(binary, binary, param.element);// 膨胀程度
	if (sParam.debug) imshow("binary", binary);

	/******************************* 检测扇叶 *************************************/
	vector<vector<Point> > armorContours;
	vector<Vec4i> armorHierarchy;
	findContours(binary, armorContours, armorHierarchy, RETR_TREE, CHAIN_APPROX_NONE);

	// 轮廓的数目
	size_t armorContours_size = armorContours.size();

	if (armorContours_size == 0) {
		cout << "装甲板检测: 二值图没有轮廓" << endl;
		return false;
	}

	// 统计每个轮廓的子轮廓的个数(作用存疑)
	int *findCount = new int[armorContours_size];
	//int findCount[ armorContours_size ];// 计数数组
	memset(findCount, 0, sizeof(findCount));
	for (size_t i = 0; i < armorContours_size; ++i) {
		// 选择有父轮廓的
		if (armorHierarchy[i][3] != -1) {// 可以尝试加入0个的
			// 去掉噪点
			if (contourArea(armorContours[i]) > param.noise_point_area) {
				findCount[armorHierarchy[i][3]]++;
			}
		}
	}

	// 选择只有一个子轮廓的箭头,可能有多个
	vector<int> conIndexs;// 得到候选轮廓在binary检测出的contours的序号
	for (size_t i = 0; i < armorContours_size; ++i) {
		if (findCount[i] == 1) {
			RotatedRect rrect = minAreaRect(armorContours[i]);

			// condition1: 长宽比
			float width = MAX(rrect.size.width, rrect.size.height);
			float height = MIN(rrect.size.width, rrect.size.height);
			float whrio = width / height;
			bool condition1 = whrio < param.flabellum_whrio_max && whrio > param.flabellum_whrio_min;

			// condition2: 面积筛选
			float area = contourArea(armorContours[i]);
			bool condition2 = area > param.flabellum_area_min;
			if (condition1 && condition2) {
				conIndexs.push_back(i);
			}
		}
	}


	if (conIndexs.size() == 0) {
		cout << "装甲板检测: 没有箭头" << endl;
		return false;
	}

	// 第二次筛选,得到最后的箭头轮廓
	int index = NO_TARGET;// 最后的轮廓编号
	float min_score = MAX_NUM;
	for (size_t i = 0; i < conIndexs.size(); ++i) {
#ifdef GET_ROI
		// 扇叶截图
		Rect rect = boundingRect(armorContours[conIndexs[i]]);
		if (!makeRectSafe(rect, src.size())) continue;
		Mat roi = src(rect);

		// 扇叶的最小外接矩形
		RotatedRect rrect = minAreaRect(armorContours[conIndexs[i]]);
		float rot_angle = 0;
		if (rrect.size.width > rrect.size.height) {
			rot_angle = 90 + rrect.angle;
			swap(rrect.size.width, rrect.size.height);
		}
		else {
			rot_angle = rrect.angle;
		}
		Point2f roi_center = Point2f(roi.cols / 2, roi.rows / 2);

		// 旋转图形，使图片信息不丢失
		Mat rot = getRotationMatrix2D(roi_center, rot_angle, 1);
		Rect2f bbox = RotatedRect(roi_center, roi.size(), rot_angle).boundingRect2f();
		rot.at<double>(0, 2) += bbox.width / 2.0 - roi.cols / 2.0;
		rot.at<double>(1, 2) += bbox.height / 2.0 - roi.rows / 2.0;
		Mat rot_roi;
		warpAffine(roi, rot_roi, rot, bbox.size());

		// 扇叶中心旋转后的点
		Mat rrect_center_mat = (Mat_<double>(3, 1) << rrect.center.x - rect.tl().x, rrect.center.y - rect.tl().y, 1);
		Mat rot_mat = rot * rrect_center_mat;
		Point2f rot_center = Point2f(rot_mat.at<double>(0, 0), rot_mat.at<double>(1, 0));

		// 截取矫正的图形
		Mat dst;
		getRectSubPix(rot_roi, rrect.size, rot_center, dst);

		Mat gray;
		cvtColor(dst, gray, COLOR_BGR2GRAY);
		static int number = 0;
		if (number < 2000) {
			char filename[100];
			sprintf(filename, "/home/chan/My_Git/lenet_image/positive_more/%d.jpg", number);
			imwrite(filename, gray);
			number++;
		}
#endif
		// 分类
		if (sParam.use_lenet) {
			// 扇叶截图
			Rect rect = boundingRect(armorContours[conIndexs[i]]);
			if (!makeRectSafe(rect, src.size())) continue;
			Mat roi = src(rect);

			// 扇叶的最小外接矩形
			RotatedRect rrect = minAreaRect(armorContours[conIndexs[i]]);
			float rot_angle = 0;
			if (rrect.size.width > rrect.size.height) {
				rot_angle = 90 + rrect.angle;
				swap(rrect.size.width, rrect.size.height);
			}
			else {
				rot_angle = rrect.angle;
			}
			Point2f roi_center = Point2f(roi.cols / 2, roi.rows / 2);

			// 旋转图形，使图片信息不丢失
			Mat rot = getRotationMatrix2D(roi_center, rot_angle, 1);
			Rect2f bbox = RotatedRect(roi_center, roi.size(), rot_angle).boundingRect2f();
			rot.at<double>(0, 2) += bbox.width / 2.0 - roi.cols / 2.0;
			rot.at<double>(1, 2) += bbox.height / 2.0 - roi.rows / 2.0;
			Mat rot_roi;
			warpAffine(roi, rot_roi, rot, bbox.size());

			// 扇叶中心旋转后的点
			Mat rrect_center_mat = (Mat_<double>(3, 1) << rrect.center.x - rect.tl().x,
				rrect.center.y - rect.tl().y,
				1);
			Mat rot_mat = rot * rrect_center_mat;
			Point2f rot_center = Point2f(rot_mat.at<double>(0, 0), rot_mat.at<double>(1, 0));

			// 截取矫正的图形
			Mat dst;
			getRectSubPix(rot_roi, rrect.size, rot_center, dst);

			// lenet forward
			Mat gray_input;
			cvtColor(dst, gray_input, COLOR_BGR2GRAY);
			Mat inputBlob = dnn::blobFromImage(gray_input, 0.00390625f, Size(28, 28), Scalar(), false);
			lenet.setInput(inputBlob, "data");
			Mat prob = lenet.forward("prob");
			Mat probMat = prob.reshape(1, 1);
			int classId;
			double classProb;
			Point classNumber;
			minMaxLoc(probMat, nullptr, &classProb, nullptr, &classNumber);
			classId = classNumber.x;    // 类别：0是noise，1是true
			if (classId == 0) {
				cout << "noise" << endl;
				if (sParam.debug) imshow("noise", gray_input);
				continue;
			}
			else {
				if (sParam.debug) imshow("true", gray_input);
			}
		}

		// 根据面积和长度计算分数
		float final_length = arcLength(armorContours[conIndexs[i]], true);
		float fianl_area = contourArea(armorContours[conIndexs[i]]);
		float score = fianl_area + final_length * 10;

		if (score < min_score) {
			min_score = score;
			index = conIndexs[i];
		}
	}

	if (index == NO_TARGET) {
		cout << "装甲板检测失败: 找不到箭头轮廓 " << endl;
		return false;
	}

	/************************** 在最后箭头轮廓中找装甲板 ****************************/
	bool findFlag = false;
	Rect final_rect = boundingRect(armorContours[index]);
	if (!makeRectSafe(final_rect, src.size())) return false;
	Mat final_ROI = binary(final_rect);// 从二值图中截取

	// 选择面积最大,得到rrect
	RotatedRect final_squa;
	float max_area = 0;

	vector<vector<Point> > final_contours;
	vector<Vec4i> final_hierarchy;
	findContours(final_ROI, final_contours, final_hierarchy, RETR_TREE, CHAIN_APPROX_NONE, final_rect.tl());
	for (size_t i = 0; i < final_contours.size(); ++i) {
		if (final_hierarchy[i][3] != -1) {// 有父轮廓
			RotatedRect squa = minAreaRect(final_contours[i]);

			// condition1: 长宽比
			float width = MAX(squa.size.width, squa.size.height);
			float height = MIN(squa.size.width, squa.size.height);
			float whrio = width / height;
			bool condition1 = whrio < param.armor_whrio_max && whrio > param.armor_whrio_min;

			// condition2: 矩匹配:用外接矩形与轮廓匹配
			Point2f P[4];
			squa.points(P);
			vector<Point2f> P1;
			for (int i = 0; i < 4; i++) {
				P1.push_back(P[i]);
			}
			double rev = matchShapes(final_contours[i], P1, CONTOURS_MATCH_I1, 0.0);
			bool condition2 = rev < param.armor_rev_thres;

			// condition3: 面积:最小面积
			float area = contourArea(final_contours[i]);
			bool condition3 = area > param.armor_area_min;

			if (condition1 && condition2 && condition3) {
				if (sParam.debug) {
					for (int j = 0; j < 4; j++) {
						cv::line(debug_src, P[j] + Point2f(offset), P[(j + 1) % 4] + Point2f(offset), cv::Scalar(0, 255, 0), 3);
					}
				}
				if (area > max_area) {
					max_area = area;
					final_squa = squa;
					findFlag = true;
				}
			}
		}
	}
	if (findFlag == false) {
		cout << "装甲板检测失败: 旋转矩形检测失败" << endl;
		return false;
	}

	/*********************************数据填充 ************************************/

	// 装甲板中心
	data.armorCenter = final_squa.center + offset;

	// 箭头中心
	RotatedRect final_rrect = minAreaRect(armorContours[index]);
	Point2f arrowCenter = final_rrect.center + offset;

	float min = MIN(final_squa.size.height, final_squa.size.width);
	if (distance(arrowCenter, data.armorCenter) < min * 0.8) {
		data.isFind = false;
	}
	else {
		float tran_angle = 0.0;
		if (final_squa.size.width > final_squa.size.height) {
			tran_angle = 90 - fabs(final_squa.angle);
		}
		else {
			tran_angle = fabs(final_squa.angle);
		}
		// 角度填充
		data.angle = tran_angle;

		// 象限填充
		if (tran_angle < 20) {
			if (final_squa.size.width < final_squa.size.height
				&& arrowCenter.x < data.armorCenter.x) {
				data.quadrant = 1;
			}
			else if (final_squa.size.width > final_squa.size.height
				&& arrowCenter.x > data.armorCenter.x) {
				data.quadrant = 2;
			}
			else if (final_squa.size.width < final_squa.size.height
				&& arrowCenter.x > data.armorCenter.x) {
				data.quadrant = 3;
			}
			else if (final_squa.size.width > final_squa.size.height
				&& arrowCenter.x < data.armorCenter.x) {
				data.quadrant = 4;
			}
		}
		else if (tran_angle > 70) {
			if (final_squa.size.width < final_squa.size.height
				&& arrowCenter.y > data.armorCenter.y) {
				data.quadrant = 1;
			}
			else if (final_squa.size.width > final_squa.size.height
				&& arrowCenter.y > data.armorCenter.y) {
				data.quadrant = 2;
			}
			else if (final_squa.size.width < final_squa.size.height
				&& arrowCenter.y < data.armorCenter.y) {
				data.quadrant = 3;
			}
			else if (final_squa.size.width > final_squa.size.height
				&& arrowCenter.y < data.armorCenter.y) {
				data.quadrant = 4;
			}
		}
		else {
			if (arrowCenter.x < data.armorCenter.x && arrowCenter.y >= data.armorCenter.y
				&& final_squa.size.width < final_squa.size.height) {
				data.quadrant = 1;
			}
			else if (arrowCenter.x >= data.armorCenter.x && arrowCenter.y > data.armorCenter.y
				&& final_squa.size.width >= final_squa.size.height) {
				data.quadrant = 2;
			}
			else if (arrowCenter.x > data.armorCenter.x && arrowCenter.y <= data.armorCenter.y
				&& final_squa.size.width < final_squa.size.height) {
				data.quadrant = 3;
			}
			else if (arrowCenter.x <= data.armorCenter.x && arrowCenter.y < data.armorCenter.y
				&& final_squa.size.width >= final_squa.size.height) {
				data.quadrant = 4;
			}
		}

		// 圆心填充
		if (data.quadrant == 1) {
			data.R_center.x = data.armorCenter.x - param.radius * cos(data.angle * CV_PI / 180);
			data.R_center.y = data.armorCenter.y + param.radius * sin(data.angle * CV_PI / 180);
		}
		else if (data.quadrant == 2) {
			data.R_center.x = data.armorCenter.x + param.radius * cos(data.angle * CV_PI / 180);
			data.R_center.y = data.armorCenter.y + param.radius * sin(data.angle * CV_PI / 180);
		}
		else if (data.quadrant == 3) {
			data.R_center.x = data.armorCenter.x + param.radius * cos(data.angle * CV_PI / 180);
			data.R_center.y = data.armorCenter.y - param.radius * sin(data.angle * CV_PI / 180);
		}
		else if (data.quadrant == 4) {
			data.R_center.x = data.armorCenter.x - param.radius * cos(data.angle * CV_PI / 180);
			data.R_center.y = data.armorCenter.y - param.radius * sin(data.angle * CV_PI / 180);
		}
		data.isFind = true;
	}


	if (sParam.debug) {
		circle(debug_src, data.armorCenter, 5, Scalar(255, 255, 255), 2);
		circle(debug_src, arrowCenter, 5, Scalar(120, 120, 125), 2);
		circle(debug_src, data.R_center, 5, Scalar(255, 255, 255), 2);
		circle(debug_src, data.R_center, distance(data.armorCenter, data.R_center), Scalar(0, 255, 0), 2);
	}
	return true;
}

/// \brief 判断顺逆时针旋转
bool Detect::getDirection() {
	int frame_nums = 20;
	static int times = 0;
	static vector<armorData> datas;
	if (times < frame_nums) {
		datas.push_back(lastData);
		times++;
		return false;
	}
	else {
		if (int(datas.size()) != frame_nums) {
			times = 0;
			datas.clear();
			return false;
		}

		// 记录角度和象限
		float *angles = new float[frame_nums];
		//float angles[frame_nums];
		memset(angles, 0, sizeof(angles));
		for (int i = 0; i < frame_nums; ++i) {
			change_angle(datas[i].quadrant, datas[i].angle, angles[i]);
		}

		int positive = 0;
		int negetive = 0;
		for (int j = 1; j < 3; ++j) {
			for (int i = 0; i < frame_nums - j; ++i) {
				if ((angles[i] - angles[i + j]) > 0 || (angles[i] - angles[i + j]) < -300) {
					positive++;
				}
				else if ((angles[i] - angles[i + j]) < 0 || (angles[i] - angles[i + j]) > 300) {
					negetive++;
				}
			}
		}

		if (positive > negetive) {
			dirFlag = true;
			cout << "顺时针:" << positive << endl;
		}
		else if (positive < negetive) {
			dirFlag = false;
			cout << "逆时针:" << negetive << endl;
		}
		times = 0;
		datas.clear();
		return true;
	}
}



/// \brief 判断是否切换
/// \param new_data 新的数据
/// \param status 当前的状态
void Detect::isCut(const armorData new_data, int &status) {

	// 连续两帧都识别到
	if (new_data.isFind == true && lastData.isFind == true) {

		// 新数据角度
		float new_tran_angle = 0.0;
		change_angle(new_data.quadrant, new_data.angle, new_tran_angle);

		// 旧数据角度
		float last_tran_angle = 0.0;
		change_angle(lastData.quadrant, lastData.angle, last_tran_angle);

		// 1和4象限边界
		if (new_data.quadrant == 4 && lastData.quadrant == 1) {
			last_tran_angle += 360;
		}
		else if (new_data.quadrant == 1 && lastData.quadrant == 4) {
			new_tran_angle += 360;
		}

		float dis = fabs(new_tran_angle - last_tran_angle);
		if (dis < 40) {
			status = 1;
		}
		else {
			status = 2;
			// 对切换指令作限制
			if (frame_cnt < param.cutLimitedTime) {// 400ms
				status = 1;
			}
			else {
				frame_cnt = 0;
			}
		}
	}

	// 掉帧后开始识别到
	else if (new_data.isFind == true
		&& lastData.isFind == false) {

		if (lostData.isFind == true) {
			// 新数据角度
			float new_tran_angle = 0.0;
			change_angle(new_data.quadrant, new_data.angle, new_tran_angle);

			// 最后一次丢失的角度
			float lost_tran_angle = 0.0;
			change_angle(lostData.quadrant, lostData.angle, lost_tran_angle);

			// 1和4象限边界
			if (new_data.quadrant == 4 && lostData.quadrant == 1) {
				lost_tran_angle += 360;
			}
			else if (new_data.quadrant == 1 && lostData.quadrant == 4) {
				new_tran_angle += 360;
			}

			float dis = fabs(new_tran_angle - lost_tran_angle);
			if (dis < 50) {
				status = 1;
			}
			else {
				status = 2;
				// 对切换指令作限制
				if (frame_cnt < param.cutLimitedTime) {// 400ms
					status = 1;
				}
				else {
					frame_cnt = 0;
				}
			}
		}
		else if (lostData.isFind == false) {
			status = 2; // 第一帧开始识别,直接给出切换
		}
	}
	// 第一帧开始掉，记录最后一次的数据
	else if (new_data.isFind == false && lastData.isFind == true) {
		lostData = lastData;
		status = 0;

	}
	// 一直识别不到
	else if (new_data.isFind == false && lastData.isFind == false) {
		status = 0;
	}
	frame_cnt++;
}

/// \brief 检测函数
/// \param frame 图
/// \param mode 接受到的指令
/// \param status 返回当前的状态
void Detect::detect(const Mat frame, int Mode, Point2f &pt, int &status) {
	// init
	if (sParam.debug) debug_src = frame.clone();
	mode = Mode;
	Mat src = frame;

	// setImage
	Point2f offset = Point2f(0, 0);

	setImage(frame, src, offset);// roi 操作


	// detect the armor
	if (mode == 7 || mode == 8) {// 小符
		armorData armordata;
		if (getArmorCenter(src, param.bMode, armordata, offset) == false) {
			pt = Point2f(0, 0);
		}
		else {
			pt = armordata.armorCenter;
		}

		isCut(armordata, status);// 判断是否切换
		if (status == 0 && pt != Point2f(0, 0)) {
			status = 1;
		}
		lastData = armordata;
	}
	else if (mode == 3 || mode == 4 || mode == 5 || mode == 6) {// 大符
		armorData armordata;
		if (getArmorCenter(src, param.bMode, armordata, offset) == false) {
			pt = Point2f(0, 0);
		}
		else {
			Point2f preCenter;
			//if (predict(armordata, preCenter, param.pMode) == false) {
			//	pt = Point2f(0, 0);
			//}
			//else {
			pt = preCenter;
			//}
		}

		isCut(armordata, status);// 判断是否切换
		lastData = armordata;
	}
	else {
		return;
	}

	if (sParam.debug) {
		imshow("debug", debug_src);
	}
}