#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include <typeinfo>
#include<string>
#include "thread"

#include "ArmorDector.h"
#include "general.h"
#include "ArmorDector.h"
#include "opencv_extend.h"
#include "log.h"
#include "ImgProdCons.h"
#include "img_buffer.h"
#include"general.h"
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
    kalmanFilterInit();
    anti_kalmanPoint = Point2f(0,0);
    //初始化相机参数
    p4psolver.SetCameraMatrix(1351.6,1355.0,344.9,239.8);
     //设置畸变参数
    p4psolver.SetDistortionCoefficients(-0.1274 , 3.5841 , 0,0,0);
    //small armor
    p4psolver.Points3D.push_back(cv::Point3f(0, 0, 0));		//P1三维坐标的单位是毫米
    p4psolver.Points3D.push_back(cv::Point3f(135, 0, 0));	//P2
    p4psolver.Points3D.push_back(cv::Point3f(60, 135, 0));	//P3
    p4psolver.Points3D.push_back(cv::Point3f(0, 60, 0));	//P4
    //cout << "装甲版世界坐标 = " << endl << p4psolver.Points3D << endl;
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
    // cv::VideoWriter out;
	// out.open(
	// 	"/home/nuc/HDU_Phoenix/my_video3.avi", //输出文件名
	// 	VideoWriter::fourcc('M','J','P','G'), // MPEG-4 编码
	// 	100, // 帧率 (FPS)
	// 	cv::Size( 640, 480 ), // 单帧图片分辨率为 640x480
	// 	true // 只输入彩色图
	// );
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
        switch (_task)
        {
            case  Serial::AUTO_SHOOT:
                putText( src, "AUTO_SHOOT", Point(10,30),
		            FONT_HERSHEY_SIMPLEX,0.5, Scalar (0,255,0),2);
                    switch(_shootTask)
                    {
                        case Serial::ARMOR_SHOOT:
                            putText( src, "ARMOR_SHOOT", Point(10,50),
		                        FONT_HERSHEY_SIMPLEX,0.5, Scalar (80,150,80),2);
                            break;
                        case Serial::BUFF_SHOOT:
                            putText( src, "BUFF_SHOOT", Point(10,50),
                                    FONT_HERSHEY_SIMPLEX,0.5, Scalar (80,150,80),2);
                            break;
                    }
                    break;
            case Serial::NO_SHOOT :
                putText( src, "NO_TASK", Point(10,30),
		            FONT_HERSHEY_SIMPLEX,0.5, Scalar (255,0,0),2);
                    break;
        }
        cv::imshow("a",src);
        cv::waitKey(10);
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
                    int type = Arm.getArmorType();
                    findEnemy=Arm.detect();
                    if(findEnemy==ArmorDetector::ARMOR_NO)
                    {
                            // //哨兵
                            // uint8_t buff[11];
                            // buff[0] = 's';
                            // for(int i=1; i<10;i++)
                            // {
                            //     buff[i] = '0';
                            // }
                            // buff[10] = 'e';
                            // serial.WriteData(buff, sizeof(buff));

                            //步兵
                            uint8_t buff[11];
                            buff[0] = 's';
                            for(int i=1; i<16;i++)
                            {
                                buff[i] = '0';
                            }
                            buff[16] = 'e';
                            serial.WriteData(buff, sizeof(buff));
                    }
                    else
                    {
                            Point offset = cv::Point(0,0);
                            std::vector<cv::Point2f>  t =Arm.getArmorVertex();
                            cv::Rect r(t[0].x,t[0].y,t[1].x-t[0].x,t[2].y-t[1].y);
                            changeArmorMode(Arm ,type);
                            p4psolver.Points2D.push_back(t[0]);	//P1
                            p4psolver.Points2D.push_back(t[1]);	//P2
                            p4psolver.Points2D.push_back(t[2]);	//P3
                            p4psolver.Points2D.push_back(t[3]);	//P4
                            //cout << "test1:图中特征点坐标 = " << endl << p4psolver.Points2D << endl;
                            if (p4psolver.Solve(PNPSolver::METHOD::CV_P3P) == 0)
		                            cout <<  "目标距离  =   " << -p4psolver.Position_OcInW.z / 1000 << "米" << endl ;
                            double distance  = -p4psolver.Position_OcInW.z / 1000;
                            p4psolver.Points2D.clear();
                            serial.sendBoxPosition(Arm,serial,1,offset);

                            //debug
                            string dis = "distance : ";
                            switch(Arm.getArmorType())
                            {
                                case BIG_ARMOR:
                                    putText( src, "BIG_ARMOR", Point(100,460),
		                                FONT_HERSHEY_SIMPLEX,0.7, Scalar (0,255,255),2);
                                    break;
                                case SMALL_ARMOR:
                                    putText( src, "SMALL_ARMOR", Point(100,460),
		                                FONT_HERSHEY_SIMPLEX,0.7, Scalar (0,255,255),2);
                                    break;
                            }
                            Point2f center = Arm.getCenterPoint(Arm);
                            Point2f predictPoint = kalmanPredict(center ,measureNum);
                            if(abs(predictPoint .x - center.x) <30)
                            {
                                 if((center.x + 0.5*(center.x - predictPoint.x))<=640
                                        || (center.x + 0.5*(center.x - predictPoint.x))>=0)//Prevent Anti-kal out of Mat
                                {
                                    if(abs(center.x - predictPoint.x) > 3)//When points are closed, no Anti-kalman to reduce shaking
                                        anti_kalmanPoint.x = center.x + 0.5*(center.x - predictPoint.x);
                                    else
                                    {
                                        anti_kalmanPoint.x = center.x;
                                        anti_kalmanPoint.y = center.y;
                                    }
                                }
                                else
                                {
                                    anti_kalmanPoint.x = center.x;
                                    anti_kalmanPoint.y = center.y;
                                }
                                if(anti_kalmanPoint .x ==0 || anti_kalmanPoint.y == 0)
                                {
                                    anti_kalmanPoint.x = center.x;
                                    anti_kalmanPoint.y = center.y;
                                }
                                circle(src,anti_kalmanPoint, 3, Scalar(0, 255, 0), 3);
                            }
                             //circle(src,predictPoint, 3, Scalar(0, 255, 0), 3);
                            circle(src,center, 2, Scalar(255, 0, 255), 2);
                            dis += to_string(distance);
                            putText( src, dis.c_str(), Point(300,460),
		                        FONT_HERSHEY_SIMPLEX,0.7, Scalar (0,0,255),3);
                            cv::rectangle(src, r, Scalar(0, 255, 255), 3);
                    }
                    
                }
                else if(_shootTask == Serial::BUFF_SHOOT)
                {
                   // Detect detect;
                   if( detect.detect_new(src) == false)
                   {
                       continue;
                   }
                }
            }
            else if(_task == Serial::NO_TASK)
            {

                 //std::cout << "change mode to NO_TASK"  << std:: endl;

            } 
        }
    }
}
void ImgProdCons::changeArmorMode(ArmorDetector  &Arm , int type)
{
     if(Arm.getArmorType() != type)
     {
        switch(Arm.getArmorType())
        {
            case SMALL_ARMOR:
                std::cout << "change" << endl;
                    p4psolver.Points3D.clear();
                    p4psolver.Points3D.push_back(cv::Point3f(0, 0, 0));		//P1三维坐标的单位是毫米
                    p4psolver.Points3D.push_back(cv::Point3f(135, 0, 0));	//P2
                    p4psolver.Points3D.push_back(cv::Point3f(60, 135, 0));	//P3
                    p4psolver.Points3D.push_back(cv::Point3f(0, 60, 0));	//P4
                    break;
            case BIG_ARMOR:
                std::cout << "change" << endl;
                p4psolver.Points3D.clear();
                p4psolver.Points3D.push_back(cv::Point3f(0, 0, 0));		//P1三维坐标的单位是毫米
                p4psolver.Points3D.push_back(cv::Point3f(230, 0, 0));	//P2
                p4psolver.Points3D.push_back(cv::Point3f(60, 230, 0));	//P3
                p4psolver.Points3D.push_back(cv::Point3f(0, 60, 0));	//P4
                break;
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
void ImgProdCons::kalmanFilterInit()
{  
    KF.init(stateNum,measureNum); 
    KF.transitionMatrix = (Mat_<float>(4, 4) <<1,0,1,0,0,1,0,1,0,0,1,0,0,0,0,1);  //转移矩阵A
	setIdentity(KF.measurementMatrix);                                             //测量矩阵H
	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
	setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
	rng.fill(KF.statePost,RNG::UNIFORM,0,640);   //初始状态值x(0)
	measurement = Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义
}
Point2f  ImgProdCons::kalmanPredict(Point nowCenter,int measureNum)
{
    //2.kalman prediction
	Mat prediction = KF.predict();
	Point predict_pt = Point(prediction.at<float>(0),prediction.at<float>(1) );   //预测值(x',y')
 
	//3.update measurement
	measurement.at<float>(0) = (float)nowCenter.x;
	measurement.at<float>(1) = (float)nowCenter.y;		
 
	//4.update
	KF.correct(measurement);
    return predict_pt;
}
