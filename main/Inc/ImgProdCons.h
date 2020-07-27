#ifndef IMGPRODCONS_H_
#define IMGPRODCONS_H_

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include <typeinfo>
#include<semaphore.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "ArmorDector.h"
#include "general.h"
#include "ArmorDector.h"
#include "opencv_extend.h"
#include "serial.h"
#include "log.h"
#include "img_buffer.h"
#include "camera.h"
#include"serial.h"
#include"Buff.h"

class ImgProdCons
{
public:

    static ImgProdCons* getInstance();

    /*
     * @Brief: Receive self state from the serail port, update task mode if commanded
     */
    void Sense();

	/*
    * @Brief: keep reading image from the camera into the buffer
	*/
	void Produce();

	/*
    * @Brief: run tasks
	*/
	void Consume();

    thread ProduceThread();

    thread ConsumeThread();

    thread SenseThread();
private:

    ImgProdCons();
    ~ImgProdCons();

private:
    static ImgProdCons * instance ;
    /* Camera */
    Mycamera mycamera;

    /*Buffer*/
    ImgBuffer buffer_;

    /* Serial */
    Serial serial;

    /* Angle solver */
    //std::unique_ptr<AngleSolver> _solverPtr;

    /* Armor detector */
    ArmorDetector Arm;
    //Buff detector
    Detect detect;

    sem_t sem_pro;
    sem_t sem_com;

     volatile  int _task;
     volatile int _shootTask;


};
#endif // !IMGPRODCONS_H_

