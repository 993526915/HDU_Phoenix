#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <typeinfo>
#include "../Camera/camera.h"

cv::Mat img = cv::Mat(480,640,CV_8UC3,(0,0,0));
Mycamera mycamera;
int main()
{
        mycamera.openWorkThread();
        while (1)
        {
                std::cout<<"main thread"<<std::endl;
                std::cout<<src<<std::endl;
                mycamera.getiamge().copyTo(img);
                cv::imshow("src1",img);
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
