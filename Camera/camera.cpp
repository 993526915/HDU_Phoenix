#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include "camera.h"
#include <boost/bind.hpp>   
#include <boost/function.hpp>   
#include<iostream>

using namespace boost;

Mycamera::Mycamera(/* args */)
{
        nRet = MV_OK;//flag
        stImageInfo = {0};
        pData = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
    	nDataSize = MAX_IMAGE_DATA_SIZE;//最大尺寸
	pDataForRGB = (unsigned char*)malloc(stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);//相机图片转化为RGB格式
	pDataForSaveImage = NULL;
        nIndex = 0;
        cameraisopen = false;
        breakflag = false;
        endmain_flag = 0;
}
/* 
设置伽马矫正，如果设置将对应的注释去掉
 */
bool Mycamera::setVideoGamma()
{
        nRet = MV_CC_SetGammaSelector(handle,camera_param.MyGammaEnable);
        if (MV_OK != nRet)
        {
                std::cout<<"setVideoGamma fail!"  <<nRet<<"\n";
                return false;
        }
        else
        {
                nRet = MV_CC_SetGamma(handle,camera_param.MyGamma);
                if (MV_OK != nRet)
                {
                        std::cout<<"setVideoGamma fail!\n";
                        return false;
                }
        }
        return true;
        
}
/* 
自动设置帧率
 */
bool Mycamera::setVideoExposureTime()
{
        nRet = MV_CC_SetExposureTime(handle,camera_param.MyExposureTime);
        if (MV_OK != nRet)
        {
                std::cout<<"MV_CC_SetExposureTime fail!\n";
                return false;
        }
        return true;
}
/* bool Mycamera::setVideoFrameRate()
{
        nRet = MV_CC_SetFloatValue(handle,"AcquisitionFrameRate",camera_param.MyAcquisitionFrameRate);
        if (MV_OK != nRet)
        {
                printf("MV_CC_SetAcquisitionFrameRate fail!\n", nRet);
                return false;
        }
        return true;
}
 */
bool Mycamera::setVideoTriggerMode()
{
        //设置触发方式，一般为关闭
	nRet = MV_CC_SetTriggerMode(handle, 0);
        if (MV_OK != nRet)
        {
               std::cout<<"MV_CC_SetTriggerMode fail! nRet [%x]\n";
                return false;
        }
        return true;
}
bool Mycamera::setVideoparam()
{
        //触发方式
        if(!setVideoTriggerMode())
        {
                return false;
        }
         //设置曝光
        if(!setVideoExposureTime())
        {
                return false;
        }
         //设置伽马变换
        /* if(!setVideoGamma())
        {
                return false;
        } */
        return true;
}
bool Mycamera::open()
{
        cameraisopen = false;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);//查找设备
        if (MV_OK != nRet)
        {
                std::cout<<"MV_CC_EnumDevices fail! nRet [%x]\n";
                return false;
        }
        /* //枚举设备,直接选择第一个设备,后面可以改成可,暂时驱动号为0，即nIndex可选
        if (stDeviceList.nDeviceNum > 0)
        {
                for (int i = 0; i < stDeviceList.nDeviceNum; i++)
                {
                        printf("[device %d]:\n", i);
                        MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                }
        }
        */
       //此时强制定义为0
        if (stDeviceList.nDeviceNum > 0)
        {
                nIndex = 0;
        }
        else
        {
                cout<<"can't find device"<<endl;
                return  false;
        }
        //创建句柄
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
                std::cout<<"MV_CC_CreateHandle fail! nRet [%x]\n";
                return false;
        }
        // 打开设备
	// open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
               std::cout<<"MV_CC_OpenDevice fail! nRet [%x]\n";
                return false;
         }
        if(!setVideoparam())
        {
                return false;
        }
        cameraisopen = true;
        return true;
}
bool Mycamera::isOpened() const
{
        if(cameraisopen == true)
        {
                return true;
        }
        else
        {
                return false;
        }
        
}
bool Mycamera::startStream()
{
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
                std::cout<<"MV_CC_StartGrabbing fail! nRet [%x]\n";
                return false;
        }
        return true;
}
bool Mycamera::closeStream()
{
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
                std::cout<<"MV_CC_StopGrabbing fail! nRet [%x]\n";
                return false;
        }
        return true;
}
bool Mycamera::destoryVideo()
{
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
               std::cout<<"MV_CC_DestroyHandle fail! nRet [%x]\n";
                return false;
        }
	
	if (pData)
	{
		free(pData);	
		pData = NULL;
	}
	if (pDataForRGB)
	{
		free(pDataForRGB);
		pDataForRGB = NULL;
	}
	if (pDataForSaveImage)
	{
		free(pDataForSaveImage);
		pDataForSaveImage = NULL;
	}
        return true;
}
bool Mycamera::rgbtocv()
{
        //way 1.
        image = cv::Mat(480,640,CV_8UC3,pDataForRGB);
        if (image.empty())
        {
                cout<<"转换失败"<<endl;
                return false;
        }
        return true;
       /*  //way 2.
        imag =  */
}
cv::Mat Mycamera::getiamge()
{
        return image;
}
bool Mycamera::getVideoimage()
{
        nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 1000);
        pDataForRGB = (unsigned char*)malloc(stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);
        if (NULL == pDataForRGB)
	{
                breakflag = true;
		return false;
	}
        std::cout<<nDataSize<<endl;
        if (nRet == MV_OK)
	{
		std::cout<<"Now you GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n\n"<<
		stImageInfo.nWidth<< stImageInfo.nHeight<< stImageInfo.nFrameNum<<std::endl;
	}
        else
        {
                cout<<"get picture from video failed"<<endl;
                return false;
        }
        if(pDataForRGB==NULL)
        {
                cout<<"内存申请失败"<<endl;
                return false;
        }
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
        cout<<stImageInfo.nWidth<<" "<<stImageInfo.nHeight<<" "<<stImageInfo.nFrameLen<<endl;
        stConvertParam.nWidth = stImageInfo.nWidth;
	stConvertParam.nHeight = stImageInfo.nHeight;
	stConvertParam.pSrcData = pData;
	stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
	stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
	stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
	stConvertParam.pDstBuffer = pDataForRGB;
	stConvertParam.nDstBufferSize = stImageInfo.nWidth * stImageInfo.nHeight *  4 + 2048;
	nRet = MV_CC_ConvertPixelType(handle, &stConvertParam);
        if (MV_OK != nRet)
	{
		std::cout<<"MV_CC_ConvertPixelType fail! nRet [%x]\n";
		return false;
	}
        cout<<"error is not in here"<<std::endl;
        return true;
}
void Mycamera::ThreadVideo()
{
        endmain_flag = 0;
        //打开
        if(!open())
        {
                return;
        }
        //设置相机参数
        if(!setVideoparam())
        {
                return;
        }
        //不断读取图片
         if(!startStream())
        {
                return;
        }
        while (1)
        {
                if(breakflag==true)
                {
                        break;
                }
                if(!getVideoimage())
                {
                        return;
                }
                if(!rgbtocv())
                {
                        return;
                }
                cv::imshow("res",image);
                if (cv::waitKey(10)>=0)
		{
			break;
		}

        }
        if(!closeStream())
        {
                return;
        }
        endmain_flag = 1;
        breakflag = false;
}
void Mycamera::openWorkThread()
{       
        std::cout<<"open camera thread"<<std::endl;
        camear_thread_ = NULL;
	camear_thread_ =new std::thread(boost::bind(&Mycamera::ThreadVideo,this));
}
void Mycamera::closeWorkThread()
{
        if(camear_thread_!=NULL)
        {
                delete camear_thread_;
        } 
}
Mycamera::~Mycamera()
{
        std::cout<<"end"<<std::endl;
}


