#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include <typeinfo>
#include "thread"

#include <opencv4/opencv2/opencv.hpp>
// #include "opencv4/opencv2/opencv_modules.hpp"
#include <opencv4/opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "ArmorDector.h"
#include "general.h"
#include "ArmorDector.h"
#include "opencv_extend.h"
#include "serial.h"
#include "log.h"
#include "ImgProdCons.h"
#include "img_buffer.h"

using namespace std;
using namespace cv;

ImgProdCons::ImgProdCons()
{
}

void ImgProdCons::Init()
{

    //Initialize camera
    // while(!camera_wrapper_.Init())
    // {

    // }
    //camera_wrapper_.Init();

    //Initilize serial

    //Initialize angle solver
    // AngleSolverParam angleParam;
    // angleParam.readFile(CAMERA_NUMBER);//choose camera
    // _solverPtr->init(angleParam);
    // _solverPtr->setResolution(_videoCapturePtr->getResolution());

    //Initialize armor detector
    // ArmorParam armorParam;
    // _armorDetectorPtr->init(armorParam);
    // _armorDetectorPtr->setEnemyColor(self_color == rm::BLUE ? rm::RED : rm::BLUE);

    //Initialize rune detector
    // _runeDetectorPtr->init();
}

void ImgProdCons::Produce()
{
    cv::Mat src;
    int temp_index = 0;
	int endmain_flag = 0;
	//打开
	while (!mycamera.open())
		;
	//设置相机参数
	while (!mycamera.setVideoparam())
		;
	//不断读取图片
	while (!mycamera.startStream());
	while (1)
	{
		DLOG_INFO << "Video Run";
		if (!mycamera.getVideoimage())
		{
			continue;
		}
		if (!mycamera.rgbtocv())
		{
			continue;
		}
        src = mycamera.getiamge();
        if(src.empty())
        {
            LOG_ERROR << "src empty";
            continue;
        }
        else
        {
            buffer_.ImgEnterBuffer(src);
        }
	}
	while (!mycamera.closeStream());
	endmain_flag = 1;
}

        // Mat dst;
        // buffer_.GetImage(dst);
        // DLOG_ERROR << dst.size();
        // cv::imshow("111", dst);
        // cv::waitKey(10);

        // Arm.loadImg(src);
        // Arm.setEnemyColor(BLUE);
        // int find_flag = Arm.detect();

        // if (find_flag != 0)
        // {
        //     std::vector<cv::Point2f> Points = Arm.getArmorVertex();
        //     cv::Point aimPoint;
        //     aimPoint.x = aimPoint.y = 0;

        //     for (const auto &point : Points)
        //     {
        //         aimPoint.x += point.x;
        //         aimPoint.y += point.y;
        //     }
        //     aimPoint.x = aimPoint.x / 4;
        //     aimPoint.y = aimPoint.y / 4;

        //     // sendBoxPosition(aimPoint);
        // }
        // else
        // {
        //     DLOG_INFO << "can't find enemy";
        // }

        // if (cv::waitKey(10) >= 0)
        // {
        //     break;
        // }
        // if (mycamera.endmain_flag == 1)
        // {
        //     break;
        // }

void ImgProdCons::Sense()
{
}

void ImgProdCons::Consume()
{
    Mat src;
    int buffindex;
    while (1)
    {
        int num = buffer_.GetImage(src);
        if (src.size().width != 640 || src.size().height != 480)
        {
            LOG_ERROR << "size error";
            cv::waitKey(1000);
            continue;
        }
        if(!src.empty() && buffer_.get_headIdx()!=buffindex)
        {
            int findEnemy;
            buffindex = buffer_.get_headIdx();
            Arm.loadImg(src);
            Arm.setEnemyColor(BLUE);
            findEnemy=Arm.detect();
            if(findEnemy==0)
            {
                LOG_INFO << "not find enemy ，picture index = " << buffer_.get_headIdx();
            }
            std::vector<cv::Point2f>  t =Arm.getArmorVertex();
            cv::Rect r(t[0].x,t[0].y,t[1].x-t[0].x,t[2].y-t[1].y);
            cv::rectangle(src, r, Scalar(0, 255, 255), 3);
            cv::imshow("a",src);
            cv::waitKey(4);
        }
        // cout << src ;
        // DLOG_INFO << src.channels();
        // cv::imshow("111", src);
        // cv::Mat dst  = src.clone();
        // cv::imshow("img1", dst);
        // cv::waitKey(1);
        // cv::waitKey(6);
        // DLOG_INFO << num;
        // buffer_.ReadComplete(num);
        // Arm.loadImg(src);
        // Arm.setEnemyColor(BLUE);
        // int find_flag = Arm.detect();

        // if (find_flag != 0)
        // {
        //     std::vector<cv::Point2f> Points = Arm.getArmorVertex();
        //     cv::Point aimPoint;
        //     aimPoint.x = aimPoint.y = 0;

        //     for (const auto &point : Points)
        //     {
        //         aimPoint.x += point.x;
        //         aimPoint.y += point.y;
        //     }
        //     aimPoint.x = aimPoint.x / 4;
        //     aimPoint.y = aimPoint.y / 4;

        //     // sendBoxPosition(aimPoint);
        // }
        // else
        // {
        //     DLOG_INFO << "can't find enemy";
        // }
    }
}
