#ifndef __CAMERA__H
#define __CAMERA__H
#include <mutex>
#include <iostream>
#include "MvCameraControl.h"
#include <thread>
#include "log.h"


extern cv::Mat src;
#define MAX_IMAGE_DATA_SIZE (40 * 1024 * 1024)
#define off 0 //关闭
#define on 1  //开启
using namespace std;

/* 
参数设置:默认参数，可以在yml文件中读取最新的文件参数
 */
struct Camera_param
{
  //picture size
  int weight = 640;
  int height = 480; //相机固定无法更改
  //相机参数设置
  float MyAcquisitionFrameRate = 600.00;                           //设置帧率
  enum MvGvspPixelType MyPixelFormat = PixelType_Gvsp_BGR8_Packed; //设置像素格式
  unsigned int MyTriggerMode = 0;                                  //0是关闭模式1是开启模式
  float MyExposureTime = 120000.0;                                  //设置曝光度
  int MyGainAuto = 0;                                              // 关闭
  bool MyEnumeration = off;                                        //关闭自动增益
  float MyGain = 0.0;                                              //自动增益设定值
  bool MyGammaEnable = on;                                        //关闭伽马矫正
  bool MyGamma = 0.0;                                              //伽马矫正设定值
  bool MySharpnessEnable = off;                                    //清晰度使能
  unsigned int MySharpness = 0;                                    //清晰度设定值
  bool MyHueEnable = off;                                          //设置色相
  unsigned int MyHue = 0;                                          //色相值设定
  bool MySaturationEnable = off;                                   //饱和度使能
  unsigned int MySaturation = 0;                                   //饱和度设定

//相机内参
double fx =1351.6;
double fy = 1355.0;
double u0 = 344.9;
double v0 = 239.8;
//镜头畸变参数
double k1 = -0.1274;
double k2 = 3.5841;
double p1 = 0.0;
double p2 = 0;
double k3 = 0;

};

class Mycamera
{
public:
  int endmain_flag; //test use
public:
       Mycamera();
       ~Mycamera();
       bool open();           //开启相机，设置相机参数，创建句柄
      bool isOpened() const; //只读是否开启成功
       bool setVideoGamma();
  /* 
        Biref：设置伽马矫正
         */
      bool setVideoExposureTime();
  /* 
        Biref：设置曝光时间
         */
      bool setVideoFrameRate();
  /* 
        Biref：设置帧率
         */

      bool   setVideoHeight();

      bool   setVideoWidth();

      bool setVideoTriggerMode();
  /* 
        Biref：设置触发方式
         */
  // bool setVideoFdormat();//
       bool setVideoparam();
  /* 
        Biref：设置相机参数
         */
      bool startStream(); //取流
  /* 
        Biref：开始取视频
        */
      bool closeStream(); //断流
  /* 
        Biref:停止取视频
         */
      bool destoryVideo();
      /* 
        Biref:停止取视频
         */

      cv::Mat getiamge();
  /* 
        Biref:主线程获取图片
         */
      bool getVideoimage();
  /* 
        
         */
      bool rgbtocv();
private:
      std::thread *camear_thread_ = NULL;
      int nRet;                  //flag
      bool cameraisopen;         //判断相机是否已经开启
      void *handle;              //相机句柄
      cv::Mat image;             //转换后的图片信息
      MV_CC_DEVICE_INFO_LIST stDeviceList;
      MV_FRAME_OUT_INFO_EX stImageInfo;
      Camera_param camera_param; //相机
      unsigned char *pData = NULL;
      unsigned int nDataSize;            //最大尺寸
      unsigned char *pDataForRGB = NULL; //相机图片转化为RGB格式
      unsigned char *pDataForSaveImage = NULL;
      unsigned int nIndex;
      bool breakflag;
      cv::Mat src;
      std::mutex lock_;
};

#endif