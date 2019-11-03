#include "Buff.h"
/// \brief ���캯��
Detect::Detect() {
	lenet = cv::dnn::readNetFromCaffe(lenet_txt_file, lenet_model_file);
	//yolo_lite = cv::dnn::readNetFromDarknet(yolo_txt_file, yolo_model_file);
}

/// \brief �������
void Detect::clear() {
	mode = 0;
	dirFlag = false;
	frame_cnt = 0;
	memset(&lastData, 0, sizeof(armorData));
	memset(&lostData, 0, sizeof(armorData));
}

/// \brief ��֤�� rect ��ͼ��ȫ
/// \param rect ͼ�е�roi��Χ
/// \param size ͼ�Ĵ�С
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

/// \brief ���ݵ㼯ʹ����С���˷����Բ
/// \param points �㼯
/// \param R_center Բ��
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

/// \brief �������ް���ת���εĽǶ�תΪ360
/// \param quadrant ����
/// \param angle ԭʼ�Ƕ�
/// \param tran_angle ת���ĽǶ�
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
		std::cout << "����Ϊ0" << std::endl;
		return false;
	}
	return true;
}


/// \brief ����roi
/// \param src ԭͼ
/// \param dect_src roi ����
/// \param offset roi ƫ��
bool Detect::setImage(const cv::Mat src, cv::Mat &dect_src, cv::Point2f &offset) {
	if (lastData.isFind == false) {
		dect_src = src;
	}
	else {
		float scale = 1.5;
		float lu_x = lastData.R_center.x - param.radius * scale;//ת��Ϊ���Ͻǵĵ�
		float lu_y = lastData.R_center.y - param.radius * scale;
		cv::Rect2f rect(lu_x, lu_y, param.radius * 2 * scale, param.radius * 2 * scale);
		if (makeRectSafe(rect, src.size()) == false) {
			dect_src = src;
			offset = cv::Point2f(0, 0);
		}
		else {
			dect_src = src(rect);
			offset = rect.tl();//�������Ͻǵĵ�
		}
	}
	return true;
}

/// \brief ��ֵ��ͼ��
/// \param src ԭͼ
/// \param binary �õ��Ķ�ֵͼ
/// \param bMode ��ֵ����
bool Detect::setBinary(const cv::Mat src, cv::Mat &binary, int bMode) {
	if (src.empty() || src.channels() != 3) return false;
	cv::Mat gray, gray_binary, tempBinary;

	if (bMode == BGR) {
		// �Ҷ���ֵ��ֵ
		cvtColor(src, gray, cv::COLOR_BGR2GRAY);
		threshold(gray, gray_binary, 50, 255, cv::THRESH_BINARY);
		imshow("grayBinary", gray_binary);

		// ����ͨ�����
		std::vector<cv::Mat> splited;
		split(src, splited);
		if (mode == RED_CLOCK || mode == RED_ANCLOCK || mode == RED_STATIC) {
			subtract(splited[2], splited[0], tempBinary);//��-��
			threshold(tempBinary, tempBinary, 135, 255, THRESH_BINARY);
		}
		else if (mode == BLUE_CLOCK || mode == BLUE_ANCLOCK || mode == BLUE_STATIC) {
			subtract(splited[0], splited[2], tempBinary);//��-��
			threshold(tempBinary, tempBinary, 60, 255, cv::THRESH_BINARY);
		}
		else {
			return false;
		}
		dilate(tempBinary, tempBinary, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
		imshow("tempBinary", tempBinary);
		// mask ����
		binary = tempBinary & gray_binary;
	}
	else if (bMode == HSV) {// ������Ļ���vͨ�������Ļ�����ֱ���ûҶ�ͼ

	   // ����ͼ
		cvtColor(src, gray, cv::COLOR_BGR2GRAY);
		threshold(gray, gray_binary, 80, 255, cv::THRESH_BINARY);

		// ��ɫ��ֵ�ָ�
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
		// mask ����
		binary = tempBinary & gray_binary;
	}
	else if (bMode == BGR_useG) {
		cvtColor(src, gray, cv::COLOR_BGR2GRAY);
		threshold(gray, gray_binary, 80, 255, THRESH_BINARY);
		imshow("gray_binary", gray_binary);

		// ����ͨ�����
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
		// mask ����
		binary = tempBinary & gray_binary;
	}
	else if (bMode == OTSU) {
		// ����㷨
		cvtColor(src, gray, COLOR_BGR2GRAY);
		double test = threshold(gray, tempBinary, 0, 255, THRESH_OTSU);// ���Եó�һ����ֵ
		cout << "test:" << test << endl;
		binary = tempBinary;
	}
	else if (bMode == GRAY) {
		// �Ҷ���ֵ
		cvtColor(src, gray, COLOR_BGR2GRAY);
		threshold(gray, gray_binary, 40, 255, THRESH_BINARY);
		binary = gray_binary;
	}
	else if (bMode == YCrCb) {

		Mat Ycrcb;
		cvtColor(src, Ycrcb, COLOR_BGR2YCrCb);
		vector<Mat> splited;
		split(Ycrcb, splited);

		// ����ͼ
		threshold(splited[0], gray_binary, 60, 255, THRESH_BINARY);

		// cr��cbͨ��
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
		// mask ����
		binary = tempBinary & gray_binary;
	}
	else if (bMode == LUV) {
		Mat luv;
		cvtColor(src, luv, COLOR_BGR2Luv);
		vector<Mat> splited;
		split(luv, splited);

		// ����ͼ
		threshold(splited[0], gray_binary, 60, 255, THRESH_BINARY);

		// ��ɫ��ֵ
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

		// mask����
		binary = gray_binary & tempBinary;
	}
	else {
		return false;
	}

	return true;
}

/// \brief ���װ�װ�
/// \param src ԭͼ
/// \param bMode ��ֵ����
/// \param data װ�װ���Ϣ-
/// \param offset ƫ��
bool Detect::getArmorCenter(const Mat src, const int bMode, armorData &data, Point2f offset) {

	/********************************* ��ֵ�� ************************************/
	Mat binary;
	if (setBinary(src, binary, bMode) == false) return false;
	dilate(binary, binary, param.element);// ���ͳ̶�
	if (sParam.debug) imshow("binary", binary);

	/******************************* �����Ҷ *************************************/
	vector<vector<Point> > armorContours;
	vector<Vec4i> armorHierarchy;
	findContours(binary, armorContours, armorHierarchy, RETR_TREE, CHAIN_APPROX_NONE);

	// ��������Ŀ
	size_t armorContours_size = armorContours.size();

	if (armorContours_size == 0) {
		cout << "װ�װ���: ��ֵͼû������" << endl;
		return false;
	}

	// ͳ��ÿ���������������ĸ���(���ô���)
	int *findCount = new int[armorContours_size];
	//int findCount[ armorContours_size ];// ��������
	memset(findCount, 0, sizeof(findCount));
	for (size_t i = 0; i < armorContours_size; ++i) {
		// ѡ���и�������
		if (armorHierarchy[i][3] != -1) {// ���Գ��Լ���0����
			// ȥ�����
			if (contourArea(armorContours[i]) > param.noise_point_area) {
				findCount[armorHierarchy[i][3]]++;
			}
		}
	}

	// ѡ��ֻ��һ���������ļ�ͷ,�����ж��
	vector<int> conIndexs;// �õ���ѡ������binary������contours�����
	for (size_t i = 0; i < armorContours_size; ++i) {
		if (findCount[i] == 1) {
			RotatedRect rrect = minAreaRect(armorContours[i]);

			// condition1: �����
			float width = MAX(rrect.size.width, rrect.size.height);
			float height = MIN(rrect.size.width, rrect.size.height);
			float whrio = width / height;
			bool condition1 = whrio < param.flabellum_whrio_max && whrio > param.flabellum_whrio_min;

			// condition2: ���ɸѡ
			float area = contourArea(armorContours[i]);
			bool condition2 = area > param.flabellum_area_min;
			if (condition1 && condition2) {
				conIndexs.push_back(i);
			}
		}
	}


	if (conIndexs.size() == 0) {
		cout << "װ�װ���: û�м�ͷ" << endl;
		return false;
	}

	// �ڶ���ɸѡ,�õ����ļ�ͷ����
	int index = NO_TARGET;// �����������
	float min_score = MAX_NUM;
	for (size_t i = 0; i < conIndexs.size(); ++i) {
#ifdef GET_ROI
		// ��Ҷ��ͼ
		Rect rect = boundingRect(armorContours[conIndexs[i]]);
		if (!makeRectSafe(rect, src.size())) continue;
		Mat roi = src(rect);

		// ��Ҷ����С��Ӿ���
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

		// ��תͼ�Σ�ʹͼƬ��Ϣ����ʧ
		Mat rot = getRotationMatrix2D(roi_center, rot_angle, 1);
		Rect2f bbox = RotatedRect(roi_center, roi.size(), rot_angle).boundingRect2f();
		rot.at<double>(0, 2) += bbox.width / 2.0 - roi.cols / 2.0;
		rot.at<double>(1, 2) += bbox.height / 2.0 - roi.rows / 2.0;
		Mat rot_roi;
		warpAffine(roi, rot_roi, rot, bbox.size());

		// ��Ҷ������ת��ĵ�
		Mat rrect_center_mat = (Mat_<double>(3, 1) << rrect.center.x - rect.tl().x, rrect.center.y - rect.tl().y, 1);
		Mat rot_mat = rot * rrect_center_mat;
		Point2f rot_center = Point2f(rot_mat.at<double>(0, 0), rot_mat.at<double>(1, 0));

		// ��ȡ������ͼ��
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
		// ����
		if (sParam.use_lenet) {
			// ��Ҷ��ͼ
			Rect rect = boundingRect(armorContours[conIndexs[i]]);
			if (!makeRectSafe(rect, src.size())) continue;
			Mat roi = src(rect);

			// ��Ҷ����С��Ӿ���
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

			// ��תͼ�Σ�ʹͼƬ��Ϣ����ʧ
			Mat rot = getRotationMatrix2D(roi_center, rot_angle, 1);
			Rect2f bbox = RotatedRect(roi_center, roi.size(), rot_angle).boundingRect2f();
			rot.at<double>(0, 2) += bbox.width / 2.0 - roi.cols / 2.0;
			rot.at<double>(1, 2) += bbox.height / 2.0 - roi.rows / 2.0;
			Mat rot_roi;
			warpAffine(roi, rot_roi, rot, bbox.size());

			// ��Ҷ������ת��ĵ�
			Mat rrect_center_mat = (Mat_<double>(3, 1) << rrect.center.x - rect.tl().x,
				rrect.center.y - rect.tl().y,
				1);
			Mat rot_mat = rot * rrect_center_mat;
			Point2f rot_center = Point2f(rot_mat.at<double>(0, 0), rot_mat.at<double>(1, 0));

			// ��ȡ������ͼ��
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
			classId = classNumber.x;    // ���0��noise��1��true
			if (classId == 0) {
				cout << "noise" << endl;
				if (sParam.debug) imshow("noise", gray_input);
				continue;
			}
			else {
				if (sParam.debug) imshow("true", gray_input);
			}
		}

		// ��������ͳ��ȼ������
		float final_length = arcLength(armorContours[conIndexs[i]], true);
		float fianl_area = contourArea(armorContours[conIndexs[i]]);
		float score = fianl_area + final_length * 10;

		if (score < min_score) {
			min_score = score;
			index = conIndexs[i];
		}
	}

	if (index == NO_TARGET) {
		cout << "װ�װ���ʧ��: �Ҳ�����ͷ���� " << endl;
		return false;
	}

	/************************** ������ͷ��������װ�װ� ****************************/
	bool findFlag = false;
	Rect final_rect = boundingRect(armorContours[index]);
	if (!makeRectSafe(final_rect, src.size())) return false;
	Mat final_ROI = binary(final_rect);// �Ӷ�ֵͼ�н�ȡ

	// ѡ��������,�õ�rrect
	RotatedRect final_squa;
	float max_area = 0;

	vector<vector<Point> > final_contours;
	vector<Vec4i> final_hierarchy;
	findContours(final_ROI, final_contours, final_hierarchy, RETR_TREE, CHAIN_APPROX_NONE, final_rect.tl());
	for (size_t i = 0; i < final_contours.size(); ++i) {
		if (final_hierarchy[i][3] != -1) {// �и�����
			RotatedRect squa = minAreaRect(final_contours[i]);

			// condition1: �����
			float width = MAX(squa.size.width, squa.size.height);
			float height = MIN(squa.size.width, squa.size.height);
			float whrio = width / height;
			bool condition1 = whrio < param.armor_whrio_max && whrio > param.armor_whrio_min;

			// condition2: ��ƥ��:����Ӿ���������ƥ��
			Point2f P[4];
			squa.points(P);
			vector<Point2f> P1;
			for (int i = 0; i < 4; i++) {
				P1.push_back(P[i]);
			}
			double rev = matchShapes(final_contours[i], P1, CONTOURS_MATCH_I1, 0.0);
			bool condition2 = rev < param.armor_rev_thres;

			// condition3: ���:��С���
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
		cout << "װ�װ���ʧ��: ��ת���μ��ʧ��" << endl;
		return false;
	}

	/*********************************������� ************************************/

	// װ�װ�����
	data.armorCenter = final_squa.center + offset;

	// ��ͷ����
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
		// �Ƕ����
		data.angle = tran_angle;

		// �������
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

		// Բ�����
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

/// \brief �ж�˳��ʱ����ת
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

		// ��¼�ǶȺ�����
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
			cout << "˳ʱ��:" << positive << endl;
		}
		else if (positive < negetive) {
			dirFlag = false;
			cout << "��ʱ��:" << negetive << endl;
		}
		times = 0;
		datas.clear();
		return true;
	}
}



/// \brief �ж��Ƿ��л�
/// \param new_data �µ�����
/// \param status ��ǰ��״̬
void Detect::isCut(const armorData new_data, int &status) {

	// ������֡��ʶ��
	if (new_data.isFind == true && lastData.isFind == true) {

		// �����ݽǶ�
		float new_tran_angle = 0.0;
		change_angle(new_data.quadrant, new_data.angle, new_tran_angle);

		// �����ݽǶ�
		float last_tran_angle = 0.0;
		change_angle(lastData.quadrant, lastData.angle, last_tran_angle);

		// 1��4���ޱ߽�
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
			// ���л�ָ��������
			if (frame_cnt < param.cutLimitedTime) {// 400ms
				status = 1;
			}
			else {
				frame_cnt = 0;
			}
		}
	}

	// ��֡��ʼʶ��
	else if (new_data.isFind == true
		&& lastData.isFind == false) {

		if (lostData.isFind == true) {
			// �����ݽǶ�
			float new_tran_angle = 0.0;
			change_angle(new_data.quadrant, new_data.angle, new_tran_angle);

			// ���һ�ζ�ʧ�ĽǶ�
			float lost_tran_angle = 0.0;
			change_angle(lostData.quadrant, lostData.angle, lost_tran_angle);

			// 1��4���ޱ߽�
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
				// ���л�ָ��������
				if (frame_cnt < param.cutLimitedTime) {// 400ms
					status = 1;
				}
				else {
					frame_cnt = 0;
				}
			}
		}
		else if (lostData.isFind == false) {
			status = 2; // ��һ֡��ʼʶ��,ֱ�Ӹ����л�
		}
	}
	// ��һ֡��ʼ������¼���һ�ε�����
	else if (new_data.isFind == false && lastData.isFind == true) {
		lostData = lastData;
		status = 0;

	}
	// һֱʶ�𲻵�
	else if (new_data.isFind == false && lastData.isFind == false) {
		status = 0;
	}
	frame_cnt++;
}

/// \brief ��⺯��
/// \param frame ͼ
/// \param mode ���ܵ���ָ��
/// \param status ���ص�ǰ��״̬
void Detect::detect(const Mat frame, int Mode, Point2f &pt, int &status) {
	// init
	if (sParam.debug) debug_src = frame.clone();
	mode = Mode;
	Mat src = frame;

	// setImage
	Point2f offset = Point2f(0, 0);

	setImage(frame, src, offset);// roi ����


	// detect the armor
	if (mode == 7 || mode == 8) {// С��
		armorData armordata;
		if (getArmorCenter(src, param.bMode, armordata, offset) == false) {
			pt = Point2f(0, 0);
		}
		else {
			pt = armordata.armorCenter;
		}

		isCut(armordata, status);// �ж��Ƿ��л�
		if (status == 0 && pt != Point2f(0, 0)) {
			status = 1;
		}
		lastData = armordata;
	}
	else if (mode == 3 || mode == 4 || mode == 5 || mode == 6) {// ���
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

		isCut(armordata, status);// �ж��Ƿ��л�
		lastData = armordata;
	}
	else {
		return;
	}

	if (sParam.debug) {
		imshow("debug", debug_src);
	}
}