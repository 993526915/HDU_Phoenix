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
#include <opencv4/opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "ArmorDector.h"
#include "general.h"
#include "ArmorDector.h"
#include "opencv_extend.h"
#include "serial.h"
#include "log.h"
#include "ImgProdCons.h"

int main(int argc, char const *argv[])
{
    ImgProdCons imgProdCons;
    GLogWrapper glog(argv[0]);
    imgProdCons.Init();
    // cv::waitKey(4000);
    // imgProdCons.Produce();
    std::thread *produceThread  = new std::thread(&ImgProdCons::Produce, &imgProdCons);
    std::thread *consumeThread =  new std:: thread(&ImgProdCons::Consume, &imgProdCons);
    // //std::thread senseThread(&ImgProdCons::sense, &imgProdCons);

    produceThread->join();
    consumeThread->join();
    // senseThread.join();

    return 0;
} 
