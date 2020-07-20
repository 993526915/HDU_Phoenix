//
// Created by yons on 2020/5/21.
//
#ifndef HDU_BUFF_BUFF_H
#define HDU_BUFF_BUFF_H
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <thread>
#include <mutex>
#include <string.h>

using namespace std;
using namespace cv;

#define NO_TARGET -1
#define MAX_NUM 921600

//#define GET_ROI

enum detectMode {
    RED_ANCLOCK = 3,
    BLUE_ANCLOCK = 4,
    RED_CLOCK = 5,
    BLUE_CLOCK = 6,
    RED_STATIC = 7,
    BLUE_STATIC = 8
};


class Detect {
    enum binaryMode {
        BGR = 1,
        HSV = 2,
        BGR_useG = 3,
        OTSU = 4,
        GRAY = 5,
        YCrCb = 6,
        LUV = 7,
    };

    enum predictMode {
        FIT_CIRCLE = 1,
        PUSH_CIRCLE = 2,
        TANGENT = 3
    };

    struct armorData {
        Point2f armorCenter;
        Point2f R_center;



        float angle;
        int quadrant;
        bool isFind;
        armorData() {
            armorCenter = cv::Point2f(0, 0);
            R_center = cv::Point2f(0, 0);
            angle = 0;
            quadrant = 0;
            isFind = 0;// 0: 未识别，1: 全部识别到
        }
    };

    struct switchParam {
        bool debug;
        bool use_yolo;
        bool use_lenet;
        switchParam() {
            debug = 1;
            use_yolo = 0;
            use_lenet = 0;
        }
    };

    struct DectParam {
        int bMode;
        int pMode;
        cv::Mat element;
        int  radius;
        float noise_point_area;
        int flabellum_area_min;
        float flabellum_whrio_min;
        float flabellum_whrio_max;
        float armor_whrio_min;
        float armor_whrio_max;
        float armor_rev_thres;
        int armor_area_min;
        int cutLimitedTime;
        float preAngle;
        DectParam() {
            // radius
            radius = 148;
            // mode
            //bMode = BGR_useG;
            bMode=OTSU;
            //bMode = LUV;
            pMode = TANGENT;
            // getArmorCenter
            element = getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
            noise_point_area = 800;//200
            flabellum_area_min = 2000;// standard:7000
            flabellum_whrio_min = 1.5;
            flabellum_whrio_max = 2.7;//standard:2
            armor_whrio_min = 1.5;
            armor_whrio_max = 2.7;// standard:2
            armor_rev_thres = 0.3;// standard: 0.0x
            armor_area_min = 300;
            // cutLimit
            cutLimitedTime = 40;// 400ms
            // predict
            preAngle = CV_PI / 7.8;
        }
    };

private:
    // param
    DectParam param;
    switchParam sParam;
    vector<Point2f> fan_armorCenters;

    // init
    int mode= RED_CLOCK;
    Mat debug_src;
    armorData lastData;
    armorData lostData;
    uint frame_cnt = 0;
    bool dirFlag;

    //deep learning
    //dnn::Net yolo_lite;
    dnn::Net lenet;

private:
    float distance(const Point2f pt1, const Point2f pt2) {
        return sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y));
    }
    bool makeRectSafe(const cv::Rect rect, const cv::Size size);
    bool circleLeastFit(const std::vector<cv::Point2f> &points, cv::Point2f &R_center,float &radius);
    bool change_angle(const int quadrant, const float angle, float &tran_angle);
public:
    Detect(){}
    bool setBinary(const cv::Mat src, cv::Mat &binary, int bMode);
    void isCut(const armorData new_data, int &status);
    bool getArmor(vector<vector<Point> >& final_contours, size_t& i, bool& findFlag, RotatedRect& final_squa, Point2f offset); //返回最后旋转矩阵final_squa

    bool getArmorCenter_new(const cv::Mat src, const int bMode,armorData &data ,cv::Point2f offset = cv::Point2f(0, 0));
    void detect_new(const Mat frame);
};


#endif //HDU_BUFF_BUFF_H
