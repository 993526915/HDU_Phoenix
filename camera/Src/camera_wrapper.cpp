#include "camera_wrapper.h"
#include "log.h"

CameraWrapper::CameraWrapper() : nRet(MV_OK), nIndex(0), handle(NULL)
{
}

CameraWrapper::~CameraWrapper()
{
    DestoryVideo();
}

bool CameraWrapper::Init()
{
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
        return false;
    }

    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                break;
            }
            PrintDeviceInfo(pDeviceInfo);
        }
    }
    else
    {
        printf("Find No Devices!\n");
        return false;
    }

    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
        return false;
    }

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
        return false;
    }

    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
        return false;
    }

    // if (!SetVideoparam())
    // {
    // 	return false;
    // }

    // 开始取流
    // start grab image
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
        return false;
    }
    printf("camera is OK!");
}

bool CameraWrapper::Read(cv::Mat &src)
{
    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned char *pData = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
    unsigned int nDataSize = MAX_IMAGE_DATA_SIZE;
    unsigned char *pDataForBGR = NULL;
    unsigned char *pDataForSaveImage = NULL;

    nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 1000);
    if (MV_OK != nRet)
    {
        printf("MV_CC_GetOneFrameTimeout fail! nRet [%x]\n", nRet);
        return false;
    }

    pDataForBGR = (unsigned char *)malloc(stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);

    // 像素格式转换
    // convert pixel format
    MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
    // 从上到下依次是：图像宽，图像高，输入数据缓存，输入数据大小，源像素格式，
    // 目标像素格式，输出数据缓存，提供的输出缓冲区大小
    // Top to bottom are：image width, image height, input data buffer, input data size, source pixel format,
    // destination pixel format, output data buffer, provided output buffer size
    stConvertParam.nWidth = stImageInfo.nWidth;
    stConvertParam.nHeight = stImageInfo.nHeight;
    stConvertParam.pSrcData = pData;
    stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
    stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
    stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    stConvertParam.pDstBuffer = pDataForBGR;
    stConvertParam.nDstBufferSize = stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048;
    nRet = MV_CC_ConvertPixelType(handle, &stConvertParam);
    if (MV_OK != nRet)
    {
        printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
        return false;
    }
    // std::cout << stImageInfo.nWidth << "   " << stImageInfo.nHeight << std::endl;
    src = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForBGR);
    // cv::Mat dst  = src.clone();
    // cv::imshow("img1", src);
    // cv::waitKey(1);
    free(pData);
    free(pDataForBGR);
    return true;
}

bool CameraWrapper::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("%s\n", "The Pointer of pstMVDevInfoList is NULL!");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        // 打印当前相机ip和用户自定义名字
        // print current ip and user defined name
        printf("%s %x\n", "nCurrentIp:", pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                 //当前IP
        printf("%s %s\n\n", "chUserDefinedName:", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName); //用户定义名
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }
    return true;
}

bool CameraWrapper::DestoryVideo()
{
    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet)
    {
        DLOG_ERROR << "MV_CC_StopGrabbing fail! nRet [%x]\n";
        return false;
    }

    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
        return -1;
    }

    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet)
    {
        DLOG_ERROR << "MV_CC_DestroyHandle fail! nRet [%x]\n";
        return false;
    }
    return true;
}

bool CameraWrapper::SetVideoparam()
{
    //触发方式
    if (!SetVideoTriggerMode())
    {
        return false;
    }
    //设置曝光
    if (!SetVideoExposureTime())
    {
        return false;
    }
    //设置伽马变换
    if (!SetVideoGamma())
    {
        return false;
    }
    return true;
}

bool CameraWrapper::SetVideoGamma()
{
    nRet = MV_CC_SetGammaSelector(handle, camera_param.MyGammaEnable);
    if (MV_OK != nRet)
    {
        DLOG_ERROR << "setVideoGamma fail!" << nRet << "\n";
        return false;
    }
    else
    {
        nRet = MV_CC_SetGamma(handle, camera_param.MyGamma);
        if (MV_OK != nRet)
        {
            DLOG_ERROR << "setVideoGamma fail!\n";
            return false;
        }
    }
    return true;
}

bool CameraWrapper::SetVideoExposureTime()
{
    nRet = MV_CC_SetExposureTime(handle, camera_param.MyExposureTime);
    if (MV_OK != nRet)
    {
        DLOG_ERROR << "MV_CC_SetExposureTime fail!\n";
        return false;
    }
    return true;
}

bool CameraWrapper::SetVideoTriggerMode()
{
    //设置触发方式，一般为关闭
    nRet = MV_CC_SetTriggerMode(handle, 0);
    if (MV_OK != nRet)
    {
        DLOG_ERROR << "MV_CC_SetTriggerMode fail! nRet [%x]\n";
        return false;
    }
    return true;
}