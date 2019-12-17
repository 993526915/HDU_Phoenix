#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <typeinfo>
#include "Camera/camera.h"
#include "Armor/Inc/ArmorDector.h"
#include "General/Inc/General.h"
#include "Armor/Inc/ArmorDector.h"
#include "opencv_extend.h"
#include "Serials/serial.h"

static bool sendTarget(Serial &serial, double x, double y, double z, uint16_t shoot_delay)
{
    static short x_tmp, y_tmp, z_tmp;
    uint8_t buff[10];

// #ifdef WITH_COUNT_FPS
//     static time_t last_time = time(nullptr);
//     static int fps;
//     time_t t = time(nullptr);
//     if (last_time != t)
//     {
//         last_time = t;
//         cout << "Armor: fps:" << fps << ", (" << x << "," << y << "," << z << ")" << endl;
//         fps = 0;
//     }
//     fps += 1;
// #endif

    x_tmp = static_cast<short>(x * (32768 - 1) / 100);
    y_tmp = static_cast<short>(y * (32768 - 1) / 100);
    z_tmp = static_cast<short>(z * (32768 - 1) / 1000);

    buff[0] = 's';
    buff[1] = static_cast<char>((x_tmp >> 8) & 0xFF);
    buff[2] = static_cast<char>((x_tmp >> 0) & 0xFF);
    buff[3] = static_cast<char>((y_tmp >> 8) & 0xFF);
    buff[4] = static_cast<char>((y_tmp >> 0) & 0xFF);
    buff[5] = static_cast<char>((z_tmp >> 8) & 0xFF);
    buff[6] = static_cast<char>((z_tmp >> 0) & 0xFF);
    buff[7] = static_cast<char>((shoot_delay >> 8) & 0xFF);
    buff[8] = static_cast<char>((shoot_delay >> 0) & 0xFF);
    buff[9] = 'e';
    //    if(buff[7]<<8 | buff[8])
    //        cout << (buff[7]<<8 | buff[8]) << endl;
    return serial.WriteData(buff, sizeof(buff));
}


cv::Mat img = cv::Mat(480,640,CV_8UC3,(0,0,0));
Mycamera mycamera;
ArmorDetector Arm;
Serial serial(115200);
int main()
{
    mycamera.openWorkThread();
    bool err = serial.InitPort();
    
    while (1)
    {
        std::cout<<"main thread"<<std::endl;
        mycamera.getiamge().copyTo(img);
        cv::imshow("src1",img);
        Arm.loadImg(img);
		//ÉžÑ¡µÆ¹Ü
		Arm.setEnemyColor(BLUE);
		int k = Arm.detect();
        cv::Point aimPoint = Arm.getCenterPoint();
        cout << "PPPPPPPPPPPPPPP:" << aimPoint.x << "            "<< aimPoint.y << std::endl;
        // cout << "EEEEEEEEEEEEEEE:" << 


        sendTarget(serial,aimPoint.x,aimPoint.y,0,0);
        if (cv::waitKey(10)>=0)
		{
			break;
		}
        if(mycamera.endmain_flag == 1)
        {
                break;
        }
    }
}
