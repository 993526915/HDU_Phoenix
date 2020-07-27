//
// Created by yons on 2020/5/21.
//
#ifndef HDU_BUFF_BUFF_H
#define HDU_BUFF_BUFF_H
#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <thread>
#include <mutex>
#include <string.h>

#include"Timestamp.h"
using namespace std;
using namespace cv;

#define NO_TARGET -1
#define MAX_NUM 921600

const string lenet_model_file = "/home/qianchen/CLionProjects/HDU_buff/lenet/lenet_iter_80000加了负样本.caffemodel";
const string lenet_txt_file = "/home/qianchen/CLionProjects/HDU_buff/lenet/deploy.prototxt";

//#define GET_ROI

//用于选择我方的颜色
enum detectMode {
    RED_ANCLOCK = 3,
    BLUE_ANCLOCK = 4,
    RED_CLOCK = 5,
    BLUE_CLOCK = 6,
    RED_STATIC = 7,
    BLUE_STATIC = 8
};

//主类
class Detect {
    //二值化模式
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
        Point2f preArmorCenter;
        Point2f predictCenter;
        float runTime;
        float radius;
        double preAngularVelocity;
        float angle;
        int quadrant;
        bool isFind;
        armorData() {
            armorCenter = cv::Point2f(0, 0);
            R_center = cv::Point2f(0, 0);
            preArmorCenter = cv::Point2f(0,0);
            predictCenter = cv::Point2f(0,0);
            angle = 0;
            radius =0;
            preAngularVelocity = 0;
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
            noise_point_area = 200;//200
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
  //Time
    CELLTimestamp _tTime;
    // param
    DectParam param;
    switchParam sParam;
    vector<Point2f> fan_armorCenters; // 用来拟合椭圆的装甲板点集
    // init
    int mode= RED_CLOCK;
    Mat debug_src;
    armorData lostData;
    uint frame_cnt = 0;
    bool dirFlag;

private:
    bool isClockwise(armorData &data);
    float distance(const Point2f pt1, const Point2f pt2) {
        return sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y));
    }
    double calAngle(double time ,double t);
    double getAcceleration(double t);
    double factTime(double angularVelocity);
    double countRotationAngle(Point2f nowCenter , Point2f preCenter , double radius , Point2f  roundCenter);
    bool makeRectSafe(const cv::Rect rect, const cv::Size size);
    bool circleLeastFit(std::vector<cv::Point2f> &points, cv::Point2f &R_center,float &radius);
public:
    Detect(){}
    bool setBinary(cv::Mat src, cv::Mat &binary, int bMode);
    bool getArmorCenter_new( cv::Mat &src, const int bMode,armorData &data ,cv::Point2f offset = cv::Point2f(0, 0),const int classiMode=1);
    bool detect_new( Mat  &frame);
    Point2f preArmorCentor(armorData &data , double time,int pMode);
    Point2f nextCoordinate(Point2f nowCenter ,Point2f R_Center,double increaseAngle ,bool isClockwise,int pMode);

    armorData data;
};


#endif //HDU_BUFF_BUFF_H
