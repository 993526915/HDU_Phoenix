#ifndef CAMERA_WRAPPER_H_
#define CAMERA_WRAPPER_H_

#include "opencv2/opencv.hpp"
#include "MvCameraControl.h"

#define MAX_IMAGE_DATA_SIZE (40 * 1024 * 1024)
#define off 0 //关闭
#define on 1  //开启

/* 
参数设置:默认参数，可以在yml文件中读取最新的文件参数
 */
struct Camera_param
{
	//picture size
	int weight = 640;
	int height = 480; //相机固定无法更改
	//相机参数设置
	float MyAcquisitionFrameRate = 600.00;							 //设置帧率
	enum MvGvspPixelType MyPixelFormat = PixelType_Gvsp_BGR8_Packed; //设置像素格式
	unsigned int MyTriggerMode = 0;									 //0是关闭模式1是开启模式
	float MyExposureTime = 12000.0;									 //设置曝光度
	int MyGainAuto = 0;												 // 关闭
	bool MyEnumeration = off;										 //关闭自动增益
	float MyGain = 0.0;												 //自动增益设定值
	bool MyGammaEnable = off;										 //关闭伽马矫正
	bool MyGamma = 0.0;												 //伽马矫正设定值
	bool MySharpnessEnable = off;									 //清晰度使能
	unsigned int MySharpness = 0;									 //清晰度设定值
	bool MyHueEnable = off;											 //设置色相
	unsigned int MyHue = 0;											 //色相值设定
	bool MySaturationEnable = off;									 //饱和度使能
	unsigned int MySaturation = 0;									 //饱和度设定
};

class CameraWrapper
{
private:
	int nRet;
	void *handle;
	Camera_param camera_param; //相机
	unsigned int nIndex;

public:
	CameraWrapper();
	~CameraWrapper();

	bool Init();
	bool Read(cv::Mat &src);
	bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
	bool DestoryVideo();

	bool SetVideoparam();

	bool SetVideoGamma();
	bool SetVideoExposureTime();
	bool SetVideoTriggerMode();
};

#endif
