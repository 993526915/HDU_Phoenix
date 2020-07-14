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
#include "log.h"
#include "ImgProdCons.h"
#include "img_buffer.h"

using namespace std;
using namespace cv;

ImgProdCons::ImgProdCons()
{
    _task = Serial::AUTO_SHOOT;
    bool err = serial.InitPort();
    if(err == Serial::USB_CANNOT_FIND)
    {
        LOG_ERROR << "USB_CANNOT_FIND";
    }
}

 ImgProdCons* ImgProdCons::getInstance()
{
    instance = new ImgProdCons();
    return instance;
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
            try
            {
                 buffer_.ImgEnterBuffer(src);
                
            }
            catch (...)
            {
                std::cout << "照片读如出错" << std::endl;
                throw;
            }
        }
	}
	while (!mycamera.closeStream());
	endmain_flag = 1;
}
void ImgProdCons::Sense()
{
    while(1)
    {
        unsigned char buff[2];
        fd_set fdRead;
        FD_ZERO(&fdRead);
        FD_SET(serial.getFd(),&fdRead);
        int fdReturn = select(serial.getFd()+1,&fdRead,0,0,nullptr);
        if(fdReturn <0)
        {
            cout << "select 失败"<<endl;
            continue;
        }
        if(FD_ISSET(serial.getFd(),&fdRead))
        {
            bool is_read=serial.ReadData(buff,2);
            if(is_read==false)
            {
                cout  << "读取串口失败" << endl;
                continue;
            }
            int mode = (int)buff[1];
            switch (mode)
            {
                case 1:
                {
                    _task = Serial::AUTO_SHOOT;
                    break;
                }
                case 2:
                {
                    _task = Serial::AUTO_SHOOT;
                    break;
                }
            }
        }
    }
}

void ImgProdCons::Consume()
{
    Mat src;
    int buffindex;
    Arm.setEnemyColor(BLUE);
    while (1)
    {
        switch (_task)
        {
            case Serial::NO_TASK:
            {
                LOG_INFO << "NO_TASK";
            }
            break;
            case Serial::AUTO_SHOOT:
            {
                //LOG_WARNING<<"AUTO_SHOOT";
            }
            break;
            default:
            {
                cout<<"UNKNOW_MODE"<<endl;
            }
            break;
        }
        try{
                buffer_.GetImage(src);
        }catch(...){
                std::cout << "读取相机图片出错" << std::endl;
        }
        if (src.size().width != 640 || src.size().height != 480)
        {
            //LOG_ERROR << "size error";
            cv::waitKey(1000);
            continue;
        }
        if(!src.empty() && buffer_.get_headIdx()!=buffindex)
        {
            buffindex = buffer_.get_headIdx();
            if(_task == Serial::AUTO_SHOOT)
            {
                int findEnemy;
                Arm.loadImg(src);
                findEnemy=Arm.detect();
                if(findEnemy==ArmorDetector::ARMOR_NO)
                {
                        //LOG_WARNING << "not find enemy ，picture index = " << buffer_.get_headIdx();
                }
                else
                {
                        std::vector<cv::Point2f>  t =Arm.getArmorVertex();
                        cv::Rect r(t[0].x,t[0].y,t[1].x-t[0].x,t[2].y-t[1].y);
                        cv::rectangle(src, r, Scalar(0, 255, 255), 3);
                        serial.sendBoxPosition(Arm,serial);
                }
                cv::imshow("a",src);
                cv::waitKey(6);
            }   
        }
    }
}

thread ImgProdCons::ConsumeThread()
{
    return thread(&ImgProdCons::Consume ,this);
}

thread ImgProdCons::ProduceThread()
{
    return thread(&ImgProdCons::Produce ,this);
}

thread ImgProdCons::SenseThread()
{
    return thread(&ImgProdCons::Sense ,this);
}