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
ImgProdCons* ImgProdCons::instance = nullptr;
int main(int argc, char const *argv[])
{
    ImgProdCons *imgProdCons = ImgProdCons::getInstance();
    GLogWrapper glog(argv[0]);
    // cv::waitKey(4000);
    // imgProdCons.Produce();
    std::thread produceThread  =  imgProdCons->ProduceThread();
    std::thread consumeThread = imgProdCons->ConsumeThread();
    std::thread senseThread    = imgProdCons->SenseThread();

    produceThread.join();
    consumeThread.join();
   senseThread.join();

    return 0;
} 
