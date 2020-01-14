#include <opencv2/opencv.hpp>
#include "opencv_extend.h"
#include <array>
#include <list>

using namespace cv;
namespace cvex
{

	using cv::Mat;
	using cv::MatND;
	using cv::Rect;
	using cv::RotatedRect;
	using cv::Vec4i;
	using cv::Scalar;
	using cv::Point;

	using cv::imshow;
	using cv::waitKey;

	using std::vector;

	void showHist(const Mat img)
	{
		Mat hist;
		int histSize = 256;
		cv::calcHist(&img, 1, 0, Mat(), hist, 1, &histSize, 0);

		Mat histImage = Mat::ones(200, 320, CV_8U) * 255;

		normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, CV_32F);

		histImage = Scalar::all(255);
		int binW = cvRound((double)histImage.cols / histSize);

		for (int i = 0; i < histSize; i++)
			rectangle(histImage, Point(i*binW, histImage.rows),
				Point((i + 1)*binW, histImage.rows - cvRound(hist.at<float>(i))),
				Scalar::all(0), -1, 8, 0);
		imshow("histogram", histImage);
		waitKey();
	}

	void rotatedRectangle(Mat& img, const RotatedRect& rec, const Scalar & color) //��ʶ��ת����
	{
		if (&rec == nullptr) return;
		Point2f pt[4];
		int i;
		for (i = 0; i < 4; i++)
		{
			pt[i].x = 0;
			pt[i].y = 0;
		}
		rec.points(pt); //�����ά���Ӷ��� 
		line(img, pt[0], pt[1], BLUE, 2, 8, 0);
		line(img, pt[1], pt[2], BLUE, 2, 8, 0);
		line(img, pt[2], pt[3], BLUE, 2, 8, 0);
		line(img, pt[3], pt[0], BLUE, 2, 8, 0);
		//imshow("aa", img);
	}

}