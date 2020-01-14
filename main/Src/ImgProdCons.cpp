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
    camera_wrapper_.Init();

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
    int i = 0;
    while (1)
    {
        camera_wrapper_.Read(src);
        // cv::imshow("111", src);
        // cv::waitKey(4);
        LOG_ERROR << src.size();
        buffer_.ImgEnterBuffer(src);

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
    }
}

void ImgProdCons::Sense()
{
}

void ImgProdCons::Consume()
{
    Mat src;
    while (1)
    {
        int num = buffer_.GetImage(src);
        DLOG_INFO << src.size();
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
