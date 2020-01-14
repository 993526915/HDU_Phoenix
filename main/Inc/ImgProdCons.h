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
#include "camera_wrapper.h"


class ImgProdCons
{
public:
    ImgProdCons();
    ~ImgProdCons() {}

    /*
     * @Brief: Initialize all the modules
     */
	void Init();

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

private:
    /* Camera */
    CameraWrapper camera_wrapper_;

    /*Buffer*/
    ImgBuffer buffer_;

    /* Serial */
    //Serial serial_;

    /* Angle solver */
    //std::unique_ptr<AngleSolver> _solverPtr;

    /* Armor detector */
    ArmorDetector Arm;

    /*Rune detector*/
    //std::unique_ptr<RuneDetector> _runeDetectorPtr;

};

#endif // !IMGPRODCONS_H_

