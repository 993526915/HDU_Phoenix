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

#include "camera.h"
#include "ArmorDector.h"
#include "general.h"
#include "ArmorDector.h"
#include "opencv_extend.h"
#include "serial.h"
#include "log.h"

#define IMAGE_CENTER_X 327
#define IMAGE_CENTER_Y 230
#define FOCUS_PIXAL 1269
#define PI (3.14159265459)

cv::Mat img = cv::Mat(480, 640, CV_8UC3, (0, 0, 0));
Mycamera mycamera;
ArmorDetector Arm;
Serial serial(115200);

static bool sendTarget(Serial &serial, float x, float y)
{
    static short x_tmp, y_tmp, z_tmp;
    uint8_t buff[10];

    union f_data {
        float temp;
        unsigned char fdata[4];
    } float_data_x, float_data_y;

    float_data_x.temp = x;
    float_data_y.temp = y;

    buff[0] = 's';
    buff[1] = static_cast<char>(float_data_x.fdata[0]);
    buff[2] = static_cast<char>(float_data_x.fdata[1]);
    buff[3] = static_cast<char>(float_data_x.fdata[2]);
    buff[4] = static_cast<char>(float_data_x.fdata[3]);
    buff[5] = static_cast<char>(float_data_y.fdata[0]);
    buff[6] = static_cast<char>(float_data_y.fdata[1]);
    buff[7] = static_cast<char>(float_data_y.fdata[2]);
    buff[8] = static_cast<char>(float_data_y.fdata[3]);
    buff[9] = 'e';

    return serial.WriteData(buff, sizeof(buff));
}

bool sendBoxPosition(cv::Point point)
{
    float dx = point.x - IMAGE_CENTER_X;
    float dy = point.y - IMAGE_CENTER_Y;
    float yaw = atan(dx / FOCUS_PIXAL) * 180 / PI;
    float pitch = atan(dy / FOCUS_PIXAL) * 180 / PI;
    DLOG_INFO << "  "
              << " yaw: " << yaw << " pitch " << pitch;
    return sendTarget(serial, yaw, pitch);
}

int main(int argc, char const *argv[])
{
    mycamera.openWorkThread();
    bool err = serial.InitPort();
    GLogWrapper glog(argv[0]);
    while (err)
    {
        DLOG_WARNING << "can't open dev/usb";
    }

    DLOG_INFO << "arm begin";

    while (1)
    {
        mycamera.getiamge().copyTo(img);
        if (img.size().width != 640 || img.size().height != 480)
        {
            LOG_ERROR << "size error";
            continue;
        }
        cv::imshow("src1", img);

        Arm.loadImg(img);
        Arm.setEnemyColor(BLUE);
        int find_flag = Arm.detect();

        if (find_flag != 0)
        {
            std::vector<cv::Point2f> Points = Arm.getArmorVertex();
            cv::Point aimPoint;
            aimPoint.x = aimPoint.y = 0;

            for (const auto &point : Points)
            {
                aimPoint.x += point.x;
                aimPoint.y += point.y;
            }
            aimPoint.x = aimPoint.x / 4;
            aimPoint.y = aimPoint.y / 4;

            sendBoxPosition(aimPoint);
        }
        else
        {
            DLOG_INFO << "can't find enemy";
        }

        if (cv::waitKey(10) >= 0)
        {
            break;
        }
        if (mycamera.endmain_flag == 1)
        {
            break;
        }
    }
}
