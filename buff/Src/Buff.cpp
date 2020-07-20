#include "Buff.h"



/// \brief 保证用 rect 截图安全
/// \param rect 图中的roi范围
/// \param size 图的大小
bool Detect::makeRectSafe(const cv::Rect rect, const cv::Size size) {
    if (rect.x < 0)
        return false;
    if (rect.x + rect.width > size.width)
        return false;
    if (rect.y < 0)
        return false;
    if (rect.y + rect.height > size.height)
        return false;
    if (rect.width <= 0 || rect.height <= 0)
        return false;
    return true;
}

/// \brief 根据点集使用最小二乘法拟合圆
/// \param points 点集
/// \param R_center 圆心
bool Detect::circleLeastFit(const std::vector<cv::Point2f> &points, cv::Point2f &R_center,float &radius) {
    float center_x = 0.0f;
    float center_y = 0.0f;
    //float radius = 0.0f;
    if (points.size() < 3) {
        return false;
    }

    double sum_x = 0.0f, sum_y = 0.0f;
    double sum_x2 = 0.0f, sum_y2 = 0.0f;
    double sum_x3 = 0.0f, sum_y3 = 0.0f;
    double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

    int N = points.size();
    for (int i = 0; i < N; i++) {
        double x = points[i].x;
        double y = points[i].y;
        double x2 = x * x;
        double y2 = y * y;
        sum_x += x;
        sum_y += y;
        sum_x2 += x2;
        sum_y2 += y2;
        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x1y2 += x * y2;
        sum_x2y1 += x2 * y;
    }

    double C, D, E, G, H;
    double a, b, c;

    C = N * sum_x2 - sum_x * sum_x;
    D = N * sum_xy - sum_x * sum_y;
    E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
    G = N * sum_y2 - sum_y * sum_y;
    H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
    a = (H * D - E * G) / (C * G - D * D);
    b = (H * C - E * D) / (D * D - G * C);
    c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

    center_x = a / (-2);
    center_y = b / (-2);
    radius = sqrt(a * a + b * b - 4 * c) / 2;
    R_center = cv::Point2f(center_x, center_y);
    return true;
}

/// \brief 根据象限把旋转矩形的角度转为360
/// \param quadrant 象限
/// \param angle 原始角度
/// \param tran_angle 转化的角度
bool Detect::change_angle(const int quadrant, const float angle, float &tran_angle) {
    if (quadrant == 1) {
        tran_angle = angle;
    }
    else if (quadrant == 2) {
        tran_angle = 90 + 90 - angle;
    }
    else if (quadrant == 3) {
        tran_angle = 180 + angle;
    }
    else if (quadrant == 4) {
        tran_angle = 270 + 90 - angle;
    }
    else {
        std::cout << "象限为0" << std::endl;
        return false;
    }
    return true;
}

/// \brief 二值化图像
/// \param src 原图
/// \param binary 得到的二值图
/// \param bMode 二值方法
bool Detect::setBinary(const cv::Mat src, cv::Mat &binary, int bMode) {
    if (src.empty() || src.channels() != 3) return false;
    cv::Mat gray, gray_binary, tempBinary;

    if (bMode == BGR) {
        // 灰度阈值二值
        cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        threshold(gray, gray_binary, 50, 255, cv::THRESH_BINARY);
        imshow("grayBinary", gray_binary);

        // 红蓝通道相减
        std::vector<cv::Mat> splited;
        split(src, splited);
        if (mode == RED_CLOCK || mode == RED_ANCLOCK || mode == RED_STATIC) {
            subtract(splited[2], splited[0], tempBinary);//红-蓝
            threshold(tempBinary, tempBinary, 70, 255, THRESH_BINARY);
        }
        else if (mode == BLUE_CLOCK || mode == BLUE_ANCLOCK || mode == BLUE_STATIC) {
            subtract(splited[0], splited[2], tempBinary);//蓝-红
            threshold(tempBinary, tempBinary, 70, 255, cv::THRESH_BINARY);
        }
        else {
            return false;
        }
        //dilate(tempBinary, tempBinary, getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1)),Point(-1,-1),3);
        imshow("tempBinary", tempBinary);
        // mask 操作
        binary = tempBinary & gray_binary;
    }
    else if (bMode == HSV) {// 如果明的话是v通道，暗的话可以直接用灰度图

        // 亮度图
        cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        threshold(gray, gray_binary, 80, 255, cv::THRESH_BINARY);

        // 颜色阈值分割
        cv::Mat imgHSV;
        cvtColor(src, imgHSV, cv::COLOR_BGR2HSV);
        if (mode == RED_ANCLOCK || mode == RED_CLOCK || mode == RED_STATIC) {
            cv::Mat temp;
            inRange(imgHSV, Scalar(0, 60, 80), Scalar(25, 255, 255), temp);
            inRange(imgHSV, Scalar(156, 60, 80), Scalar(181, 255, 255), tempBinary);
            tempBinary = temp | tempBinary;
        }
        else if (mode == BLUE_ANCLOCK || mode == BLUE_CLOCK || mode == BLUE_STATIC) {
            inRange(imgHSV, cv::Scalar(35, 46, 80), cv::Scalar(99, 255, 255), tempBinary);
        }
        else {
            return false;
        }
        imshow("tempBinary", tempBinary);
        dilate(tempBinary, tempBinary, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        // mask 操作
        binary = tempBinary & gray_binary;
    }
    else if (bMode == BGR_useG) {
        cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        threshold(gray, gray_binary, 80, 255, THRESH_BINARY);
        imshow("gray_binary", gray_binary);

        // 与绿通道相减
        std::vector<cv::Mat> splited;
        split(src, splited);
        //Mat tmp_tmp_binary;
        if (mode == RED_CLOCK || mode == RED_ANCLOCK || mode == RED_STATIC) {
            subtract(splited[2], splited[1], tempBinary);
             imshow("before binary", tempBinary);
            //threshold(tempBinary, tmp_tmp_binary, 150, 255, THRESH_BINARY);
            threshold(tempBinary, tempBinary, 80, 255, THRESH_BINARY);
        }
        else if (mode == BLUE_CLOCK || mode == BLUE_ANCLOCK || mode == BLUE_STATIC) {
            subtract(splited[0], splited[1], tempBinary);
            imshow("before binary", tempBinary);
            //threshold(tempBinary, tmp_tmp_binary, 120, 255, THRESH_BINARY);
            threshold(tempBinary, tempBinary, 80, 255, THRESH_BINARY);
        }
        else {
            return false;
        }
        //imshow("tmp_tmp_binary", tmp_tmp_binary);
        dilate(tempBinary, tempBinary, getStructuringElement(MORPH_RECT, Size(5, 5)));
        imshow("tempBinary", tempBinary);
        // mask 操作
        binary = tempBinary & gray_binary;
        //erode(binary,binary,getStructuringElement(MORPH_RECT, Size(3, 3)));
        //morphologyEx(binary, binary, MORPH_OPEN,getStructuringElement(MORPH_RECT, Size(3, 3)));
    }
    else if (bMode == OTSU) {
        // 大津算法
        cvtColor(src, gray, COLOR_BGR2GRAY);
        double test = threshold(gray, tempBinary, 0, 255, THRESH_OTSU);// 可以得出一个阈值
        cout << "test:" << test << endl;
        binary = tempBinary;
    }
    else if (bMode == GRAY) {
        // 灰度阈值
        cvtColor(src, gray, COLOR_BGR2GRAY);
        threshold(gray, gray_binary, 40, 255, THRESH_BINARY);
        binary = gray_binary;
    }
    else if (bMode == YCrCb) {

        Mat Ycrcb;
        cvtColor(src, Ycrcb, COLOR_BGR2YCrCb);
        vector<Mat> splited;
        split(Ycrcb, splited);

        // 亮度图
        threshold(splited[0], gray_binary, 60, 255, THRESH_BINARY);

        // cr和cb通道
        if (mode == RED_CLOCK || mode == RED_ANCLOCK || mode == RED_STATIC) {
            subtract(splited[1], splited[2], tempBinary);
            threshold(tempBinary, tempBinary, 20, 255, THRESH_BINARY);
        }
        else if (mode == BLUE_CLOCK || mode == BLUE_ANCLOCK || mode == BLUE_STATIC) {
            subtract(splited[2], splited[1], tempBinary);
            threshold(tempBinary, tempBinary, 40, 255, THRESH_BINARY);
        }
        else {
            return false;
        }

        dilate(tempBinary, tempBinary, getStructuringElement(MORPH_RECT, Size(3, 3)));
        // mask 操作
        binary = tempBinary & gray_binary;
    }
    else if (bMode == LUV) {
        Mat luv;
        cvtColor(src, luv, COLOR_BGR2Luv);
        vector<Mat> splited;
        split(luv, splited);

        // 亮度图
        threshold(splited[0], gray_binary, 60, 255, THRESH_BINARY);

        // 颜色阈值
        if (mode == RED_ANCLOCK || mode == RED_CLOCK || mode == RED_STATIC) {
            threshold(splited[2], tempBinary, 160, 255, THRESH_BINARY);
        }
        else if (mode == BLUE_ANCLOCK || mode == BLUE_CLOCK || mode == BLUE_STATIC) {
            threshold(splited[1], tempBinary, 70, 255, THRESH_BINARY_INV);

        }
        else {
            return false;
        }
        imshow("tempBinary", tempBinary);
        //dilate(tempBinary, tempBinary, getStructuringElement(MORPH_RECT, Size(3, 3)));

        // mask操作
        binary = gray_binary & tempBinary;
    }
    else {
        return false;
    }

    return true;
}



/// 新版本，思想是希望把已经击打过的装甲板中心也用在椭圆拟合上
/// \brief 检测装甲板_mode2
/// \param src 原图
/// \param bMode 二值方法
/// \param data 装甲板信息-
/// \param offset ROI的偏移
bool Detect::getArmorCenter_new(const Mat src, const int bMode, armorData &data,Point2f offset)
{
    /********************************* 二值化 ************************************/
    Mat binary;
    if (setBinary(src, binary, bMode) == false)
        return false;
    dilate(binary, binary, param.element);// 膨胀程度

    /********************************尝试用轮廓匹配的方式去找矩形**********************************/
    vector<vector<Point > > armorContours;
    vector<Vec4i> armorHierarchy;
    findContours(binary, armorContours, armorHierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    int armorContours_size = armorContours.size();
    //int findCount[armorContours_size]={0};
    int *findCount = new int[armorContours_size]();
    //memset(findCount, 0, sizeof(findCount));

    for (size_t i = 0; i < armorContours_size; ++i)
    {
        // 选择有父轮廓的
        if (armorHierarchy[i][3] != -1)//armorHierarchy[i]是有父轮廓的轮廓
        {// 可以尝试加入0个的
            // 去掉噪点
            if ((contourArea(armorContours[i]) > param.noise_point_area)&&(armorHierarchy[i][3]>=0)) {
                findCount[armorHierarchy[i][3]]++;//armorHierarchy[i][3]是父轮廓
            }
        }
    }

    //cout<<"findCount:("<<armorContours_size<<")"<<endl;
    // for (size_t i = 0; i < armorContours_size; ++i)
    //     cout<<findCount[i]<<" ";


    Point2f center;
    drawContours(src,armorContours,0,Scalar(0,255,0));
    static int  center_index=0;
    for (size_t i = 0; i < armorContours_size; ++i)
    {
        double contour_area=contourArea(armorContours[i]);
        if(contour_area<2000) continue;
        if(findCount[i]==1)
        {
            int son=armorHierarchy[i][2];

            //调试
            if((son!=-1)&&(armorContours[son].size()>2))
            {
                RotatedRect son_rect=minAreaRect(armorContours[son]);
                Point2f p[4];
                son_rect.points(p);
                for(int j=0; j<4; j++)
                    {
                        line(src, p[j],p[(j+1)%4], Scalar(0, 0, 255), 2, 8);  //绘制最小外接矩形每条边
                    }
                if(son_rect.center==Point2f(0,0)) return false;
                data.armorCenter=son_rect.center+offset;
            }

            if(data.armorCenter!=Point2f(0,0))
            {
                if(fan_armorCenters.size()<50)
                {
                    fan_armorCenters.push_back(data.armorCenter);
                }
                else if(fan_armorCenters.size()>=50)
                {
                    if(center_index==50)center_index=0;
                    fan_armorCenters[center_index]=data.armorCenter;
                    center_index++;
                }
            }

        }
        else if(findCount[i]>2)
        {
            drawContours(src,armorContours,i,Scalar(0,255,0));
            vector<int> sons;
            int final_son = armorHierarchy[i][2];
            while(final_son!=-1)
            {
                //调试
                cout<<"final_son:"<<final_son<<endl;
                //drawContours(src,armorContours,final_son,Scalar(0,255,255));


                sons.push_back(final_son);
                final_son = armorHierarchy[sons.back()][0];
            }
//            for(int it =0;it<sons.size();it ++) cout<<sons[it]<<" ";  //子轮廓序号
            RotatedRect FatherRect = minAreaRect(armorContours[i]);
            if(FatherRect.size.width*FatherRect.size.height<5000) continue;

            vector<RotatedRect> SonsRect;
            for(int s=0;s<sons.size();s++)
            {
                RotatedRect SonRect=minAreaRect(armorContours[sons[s]]);
                SonsRect.push_back(SonRect);
            }

            //调试
            Point2f p[4];
            FatherRect.points(p);
            for(int j=0; j<4; j++)
            {
                line(src, p[j],p[(j+1)%4], Scalar(0, 0, 255), 2, 8);  //绘制最小外接矩形每条边
            }


            //调试
            cout<<"父亲的角度："<<"("<<FatherRect.angle<<")"<<endl;
            cout<<"差值："<<endl;
            for(int s=0;s<sons.size();s++)
            {
                Point2f p2[4];
                SonsRect[s].points(p2);
                for(int j=0; j<4; j++)
                {
                    line(src, p2[j],p2[(j+1)%4], Scalar(0, 255, 255), 2, 8);  //绘制最小外接矩形每条边
                }
                cout<<"("<<SonsRect[s].angle<<"-"<<FatherRect.angle<<")"<<abs (SonsRect[s].angle-FatherRect.angle)<<endl;
            }

            vector<float> Sons_whratio;
            for(int j=0;j<SonsRect.size();j++)
            {
                float a=SonsRect[j].size.height;
                float b=SonsRect[j].size.width;
                if(a<b) swap(a,b);
                Sons_whratio.push_back(a/b);
            }

            for(int j=0;j<Sons_whratio.size();j++)
            {
                if((Sons_whratio[j]>1)&&(Sons_whratio[j]<2.5))
                {
                    if(fan_armorCenters.size()<50)
                    {
                        fan_armorCenters.push_back(SonsRect[j].center+offset);
                    }
                    else if(fan_armorCenters.size()>=50)
                    {
                        if(center_index==50)center_index=0;
                        fan_armorCenters[center_index]=SonsRect[j].center+offset;
                        center_index++;
                    }

                }
            }
        }
        else
        {
            continue;
        }

    }

    /*********************拟合圆************************/
    float radius;
    circleLeastFit(fan_armorCenters,data.R_center,radius);
    // for(int j=0;j<fan_armorCenters.size();j++)
    //     cout<<"fan_armorCenters"<<fan_armorCenters[j]<<" ";

    // cout<<"R_center"<<data.R_center<<endl;
    // cout<<"last_center"<<lastData.armorCenter;
    //if (sParam.debug) {
        circle(src, data.armorCenter, 5, Scalar(255, 255, 255), 2);
        // circle(src, lastData.R_center, 5, Scalar(255, 255, 255), 2);
        // circle(src, lastData.R_center, radius, Scalar(20, 100, 100), 2);
    //}

   imshow("father_son",src);
    delete []findCount;

    return true;
}

void Detect::detect_new(const Mat frame)
{
    Mat src = frame;

    // setImage
    Point2f offset = Point2f(0, 0);

    // detect the armor
        armorData data ;
        if(getArmorCenter_new(src, 3, data, offset)==true)
            lastData=data;
        //std::cout << lastData.armorCenter.x << " " << lastData.armorCenter.y<<endl;
}