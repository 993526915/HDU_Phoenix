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
    sem_init(&sem_pro , 0 , 0 );
    sem_init(&sem_com , 0 , 1 );
    _task = Serial::AUTO_SHOOT;
    _shootTask = Serial::ARMOR_SHOOT;
    bool err = serial.InitPort();
    if(err == Serial::USB_CANNOT_FIND)
    {
        LOG_ERROR << "USB_CANNOT_FIND";
    }
}
ImgProdCons::~ImgProdCons()
{
    sem_destroy(&sem_pro);
    sem_destroy(&sem_com);
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
		//DLOG_INFO << "Video Run";
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
            sem_wait(&sem_com);
            try
            {
                buffer_.ImgEnterBuffer(src);
            }
            catch (...)
            {
                std::cout << "照片读如出错" << std::endl;
                throw;
            }
            sem_post(&sem_pro);
        }
	}
	while (!mycamera.closeStream());
	endmain_flag = 1;
}
unsigned short int  decode(unsigned char *buff)
{
    if(buff[0] == 'a'  && buff[3] == 'b')
    {
        return (unsigned short int )(buff[2] << 8 | buff[1]);  
    }
}
void ImgProdCons::Sense()
{
    while(1)
    {
        unsigned char buff[4];
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
            bool is_read=serial.ReadData(buff,4);
           // std::cout << decode(buff) << std::endl;
            // if(buff[0]!= 'a' || buff[2] != 'b')
            // {
            //     std::cout << "串口接收数据错误"  << std::endl;
            //     continue;
            // }
            //puts((const char *)buff);
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
                    _shootTask = Serial::BUFF_SHOOT;
                    break;
                }
                case 2:
                {
                    _task = Serial::AUTO_SHOOT;
                    _shootTask = Serial::ARMOR_SHOOT;
                    break;
                }
                case 3:
                {
                     _task = Serial::NO_TASK;
                    _shootTask = Serial::NO_SHOOT;
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
        sem_wait(&sem_pro);
        try{
                buffer_.GetImage(src);
        }catch(...){ 
                std::cout << "读取相机图片出错" << std::endl;
                exit(-1);
        }
        sem_post(&sem_com);
        cv::imshow("a",src);
        cv::waitKey(1);  
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
                if(_shootTask == Serial::ARMOR_SHOOT)
                {

                    //std::cout << "change mode to ARMOR_SHOOT"  << std:: endl;

                    int findEnemy;
                    Arm.loadImg(src);
                    findEnemy=Arm.detect();
                    if(findEnemy==ArmorDetector::ARMOR_NO)
                    {
                            //LOG_WARNING << "not find enemy ，picture index = " << buffer_.get_headIdx();
                            uint8_t buff[11];
                            buff[0] = 's';
                            for(int i=1; i<10;i++)
                            {
                                buff[i] = '0';
                            }
                            buff[10] = 'e';
                            serial.WriteData(buff, sizeof(buff));
                    }
                    else
                    {
                            std::vector<cv::Point2f>  t =Arm.getArmorVertex();
                            cv::Rect r(t[0].x,t[0].y,t[1].x-t[0].x,t[2].y-t[1].y);
                            cv::rectangle(src, r, Scalar(0, 255, 255), 3);
                            serial.sendBoxPosition(Arm,serial,1);
                    }
                    
                }
                else if(_shootTask == Serial::BUFF_SHOOT)
                {

                   // std::cout << "change mode to BUFF_SHOOT"  << std:: endl;

                }
            }
            else if(_task == Serial::NO_TASK)
            {

                 //std::cout << "change mode to NO_TASK"  << std:: endl;

            } 
            // cv::imshow("a",src);
            // cv::waitKey(10);  
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