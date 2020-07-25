#include "Buff.h"


#define PI 3.14159265

enum classiMode {
    mySonNum = 1,
    useLenet
};


/// \brief 计算旋转角度
/// \param nowCenter 当前识别中心
/// \param preCenter 上一次识别中心
/// \param radius 拟合圆的半径
/// \param roundCenter 拟合圆的中心
double Detect::countRotationAngle(Point2f nowCenter , Point2f preCenter , double radius , Point2f  roundCenter)
{
    double dis = distance(nowCenter ,preCenter );
    double cos = (2*radius*radius - dis * dis ) / (2 * radius*radius);
    return acos(cos);
}
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
bool Detect::circleLeastFit(const std::vector<cv::Point2f> &points, cv::Point2f &R_center, float &radius) {
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


/// \brief 设置roi
/// \param src 原图
/// \param dect_src roi 区域
/// \param offset roi 偏移
bool Detect::setImage(const cv::Mat src, cv::Mat &dect_src, cv::Point2f &offset) {
    if (lastData.isFind == false) {
        dect_src = src;
    } else {
        float scale = 1.5;
        float lu_x = lastData.R_center.x - param.radius * scale;//转变为左上角的点
        float lu_y = lastData.R_center.y - param.radius * scale;
        cv::Rect2f rect(lu_x, lu_y, param.radius * 2 * scale, param.radius * 2 * scale);
        if (makeRectSafe(rect, src.size()) == false) {
            dect_src = src;
            offset = cv::Point2f(0, 0);
        } else {
            dect_src = src(rect);
            offset = rect.tl();//返回左上角的点
        }
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
        // Mat element = getStructuringElement(MORPH_RECT, Size(10, 10));
	    // dilate(gray_binary, gray_binary, element);
        imshow("grayBinary", gray_binary);
        // 红蓝通道相减
        std::vector<cv::Mat> splited;
        split(src, splited);
        if (mode == RED_CLOCK || mode == RED_ANCLOCK || mode == RED_STATIC) {
            subtract(splited[2], splited[0], tempBinary);//红-蓝
            threshold(tempBinary, tempBinary, 70, 255, THRESH_BINARY);
        } else if (mode == BLUE_CLOCK || mode == BLUE_ANCLOCK || mode == BLUE_STATIC) {
            subtract(splited[0], splited[2], tempBinary);//蓝-红
            threshold(tempBinary, tempBinary, 70, 255, cv::THRESH_BINARY);
        } else {
            return false;
        }
        //dilate(tempBinary, tempBinary, getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1)),Point(-1,-1),3);
        imshow("tempBinary", tempBinary);
        // mask 操作
        binary = tempBinary & gray_binary;
    } else if (bMode == HSV) {// 如果明的话是v通道，暗的话可以直接用灰度图

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
        } else if (mode == BLUE_ANCLOCK || mode == BLUE_CLOCK || mode == BLUE_STATIC) {
            inRange(imgHSV, cv::Scalar(35, 46, 80), cv::Scalar(99, 255, 255), tempBinary);
        } else {
            return false;
        }
        imshow("tempBinary", tempBinary);
        dilate(tempBinary, tempBinary, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        // mask 操作
        binary = tempBinary & gray_binary;
    } else if (bMode == BGR_useG) {
        cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        threshold(gray, gray_binary, 80, 255, THRESH_BINARY);
        imshow("gray_binary", gray_binary);

        // 与绿通道相减
        std::vector<cv::Mat> splited;
        split(src, splited);
        //Mat tmp_tmp_binary;
        if (mode == RED_CLOCK || mode == RED_ANCLOCK || mode == RED_STATIC) {
            subtract(splited[2], splited[1], tempBinary);
            //threshold(tempBinary, tmp_tmp_binary, 150, 255, THRESH_BINARY);
            threshold(tempBinary, tempBinary, 80, 255, THRESH_BINARY);
        } else if (mode == BLUE_CLOCK || mode == BLUE_ANCLOCK || mode == BLUE_STATIC) {
            subtract(splited[0], splited[1], tempBinary);
            //threshold(tempBinary, tmp_tmp_binary, 120, 255, THRESH_BINARY);
            threshold(tempBinary, tempBinary, 80, 255, THRESH_BINARY);
        } else {
            return false;
        }
        //imshow("tmp_tmp_binary", tmp_tmp_binary);
        dilate(tempBinary, tempBinary, getStructuringElement(MORPH_RECT, Size(5, 5)));
        imshow("tempBinary", tempBinary);
        // mask 操作
        binary = tempBinary & gray_binary;
        //erode(binary,binary,getStructuringElement(MORPH_RECT, Size(3, 3)));
        //morphologyEx(binary, binary, MORPH_OPEN,getStructuringElement(MORPH_RECT, Size(3, 3)));
    } else if (bMode == OTSU) {
        // 大津算法
        cvtColor(src, gray, COLOR_BGR2GRAY);
        double test = threshold(gray, tempBinary, 0, 255, THRESH_OTSU);// 可以得出一个阈值
        cout << "test:" << test << endl;
        binary = tempBinary;
    } else if (bMode == GRAY) {
        // 灰度阈值
        cvtColor(src, gray, COLOR_BGR2GRAY);
        threshold(gray, gray_binary, 40, 255, THRESH_BINARY);
        binary = gray_binary;
    } else if (bMode == YCrCb) {

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
        } else if (mode == BLUE_CLOCK || mode == BLUE_ANCLOCK || mode == BLUE_STATIC) {
            subtract(splited[2], splited[1], tempBinary);
            threshold(tempBinary, tempBinary, 40, 255, THRESH_BINARY);
        } else {
            return false;
        }

        dilate(tempBinary, tempBinary, getStructuringElement(MORPH_RECT, Size(3, 3)));
        // mask 操作
        binary = tempBinary & gray_binary;
    } else if (bMode == LUV) {
        Mat luv;
        cvtColor(src, luv, COLOR_BGR2Luv);
        vector<Mat> splited;
        split(luv, splited);

        // 亮度图
        threshold(splited[0], gray_binary, 60, 255, THRESH_BINARY);

        // 颜色阈值
        if (mode == RED_ANCLOCK || mode == RED_CLOCK || mode == RED_STATIC) {
            threshold(splited[2], tempBinary, 160, 255, THRESH_BINARY);
        } else if (mode == BLUE_ANCLOCK || mode == BLUE_CLOCK || mode == BLUE_STATIC) {
            threshold(splited[1], tempBinary, 70, 255, THRESH_BINARY_INV);

        } else {
            return false;
        }
        imshow("tempBinary", tempBinary);
        //dilate(tempBinary, tempBinary, getStructuringElement(MORPH_RECT, Size(3, 3)));

        // mask操作
        binary = gray_binary & tempBinary;
    } else {
        return false;
    }

    return true;
}

/// 新版本，思想是希望把已经击打过的装甲板中心也用在椭圆拟合上
/// \brief 检测装甲板_mode2
/// \param src 原图
/// \param bMode 二值方法
/// \param data 装甲板信息
/// \param offset ROI的偏移
bool Detect::getArmorCenter_new(const Mat src, const int bMode, armorData &data, Point2f offset, const int classiMode) {
    /********************************* 二值化 ************************************/
    Mat binary;
    if (setBinary(src, binary, bMode) == false)
        return false;
    // Mat element = getStructuringElement(MORPH_RECT, Size(20, 20));
	// dilate(binary, binary, element);
    dilate(binary, binary, param.element);// 膨胀程度
    if (sParam.debug)
        imshow("binary", binary);

    /******************************* 反转二值图 *************************************/
    vector<vector<Point> > armorContours;
    vector<Vec4i> armorHierarchy;
    findContours(binary, armorContours, armorHierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    int armorContours_size = armorContours.size();
    //int findCount[armorContours_size]={0};
    int *findCount = new int[armorContours_size]();

    /*********************************筛选条件1：只有存在子轮廓的轮廓才可能是扇叶***************************************/

    for (size_t i = 0; i < armorContours_size; ++i) {
        // 选择有父轮廓的
        if (armorHierarchy[i][3] != -1)//armorHierarchy[i]是有父轮廓的轮廓
        {// 可以尝试加入0个的
            // 去掉噪点
            if ((contourArea(armorContours[i]) > param.noise_point_area) && (armorHierarchy[i][3] >= 0)) {
                // armorHierarchy[i][3]是父轮廓，也就是其父轮廓的值++
                // 最后findCount就代表了该父轮廓所拥有的子轮廓数目
                findCount[armorHierarchy[i][3]]++;
            }
        }
    }
    /* //输出每个轮廓的子轮廓数目
     * cout<<"findCount:("<<armorContours_size<<")"<<endl;
    for (size_t i = 0; i < armorContours_size; ++i)
        cout<<findCount[i]<<" ";
     */

    /*********************************分类模式1：根据子轮廓的数目进行分类*******************************/
    /*适用于符文扇叶完整，并且光照条件良好情况
     * 优点：速度快
     * 缺点：没有考虑语义信息，鲁棒性不够
     * */

    Point2f center;
     Point2f predictAimPoint;
    static int center_index = 0; // 用于表示所找到的装甲板中心的下标（个数）
    int hammerToFall;//用于统计本帧中是否存在锤子
    if (classiMode==mySonNum) {
        for (size_t i = 0; i < armorContours_size; ++i)// 遍历每个轮廓
        {
            double contour_area = contourArea(armorContours[i]);
            if (contour_area < 2000) continue; // 如果面积太小了，那就直接不管了

            //如果子轮廓只有一个，大概率是个锤子
            if (findCount[i] == 1) {
                hammerToFall++;

                int son = armorHierarchy[i][2];//子轮廓的ID

                //调试
                if ((son != -1) && (armorContours[son].size() > 2))//子轮廓ID不为-1，并且该轮廓点集数目必须大于2(才可以获得外接矩形)
                {
                    RotatedRect son_rect = minAreaRect(armorContours[son]);//锤子中子轮廓（装甲板）的外接矩形
                    if (son_rect.center == Point2f(0, 0)) return false;
                    Point2f p[4];//装甲板四个点
                    son_rect.points(p);
                    for (int j = 0; j < 4; j++)//画出装甲板矩形
                    {
                        line(src, p[j], p[(j + 1) % 4], Scalar(0, 0, 255), 2, 8);  //绘制最小外接矩形每条边
                    }
                    data.preArmorCenter  =  data.armorCenter;
                    data.armorCenter = son_rect.center + offset;//在原图中的坐标
                    data.runTime = _tTime.getElapsedTimeInMilliSec();
                    predictAimPoint = preArmorCentor(data ,0.7);
                    data.predictCenter = predictAimPoint;
                    _tTime.update();
                    std::cout << "运行时间   " << data. runTime << endl;
                    // std::cout << "原来" << data.preArmorCenter.x << "    " << data.preArmorCenter .y <<endl;
                    // std::cout << "现在" << data.armorCenter.x << "    " << data.armorCenter .y <<endl;
                    std::cout << "预测   " << data.predictCenter<<endl;
                }

                //将找到的点放入点集中为后面拟合椭圆做准备
                if (data.armorCenter != Point2f(0, 0)) {
                    if (fan_armorCenters.size() < 50) {
                        fan_armorCenters.push_back(data.armorCenter);
                    } else if (fan_armorCenters.size() >= 50) {
                        if (center_index == 50)center_index = 0;
                        fan_armorCenters[center_index] = data.armorCenter;
                        center_index++;
                    }
                }

            } else if (findCount[i] > 2)//子轮廓>2，有可能是六边形
            {
                /*//用来画六边形的轮廓
                drawContours(src,armorContours,i,Scalar(0,255,0));*/
                RotatedRect FatherRect = minAreaRect(armorContours[i]);//有可能是六边形的轮廓
                if (FatherRect.size.width * FatherRect.size.height < 5000) continue;//六边形外接矩形面积不能太小


                vector<int> sons;
                int final_son = armorHierarchy[i][2];
                //把所有子轮廓找到，然后把其下标放入sons
                while (final_son != -1) {
                    /*//画出所有子轮廓
                    cout<<"final_son:"<<final_son<<endl;
                    drawContours(src,armorContours,final_son,Scalar(0,255,255));*/

                    //子轮廓都放入sons中
                    sons.push_back(final_son);
                    final_son = armorHierarchy[sons.back()][0];
                }
                //for(int it =0;it<sons.size();it ++) cout<<sons[it]<<" ";  //子轮廓序号

                vector<RotatedRect> SonsRect;
                for (int s = 0; s < sons.size(); s++) {
                    RotatedRect SonRect = minAreaRect(armorContours[sons[s]]);
                    SonsRect.push_back(SonRect);
                }

                //调试：画出六边形外接矩形
                Point2f p[4];
                FatherRect.points(p);
                for (int j = 0; j < 4; j++) {
                    line(src, p[j], p[(j + 1) % 4], Scalar(0, 0, 255), 2, 8);  //绘制最小外接矩形每条边
                }


                //调试
                // cout << "父亲的角度：" << "(" << FatherRect.angle << ")" << endl;
                // cout << "差值：" << endl;//认为父轮廓的角度应该是相近的
                int qualifiedSubAngle = 0;//如果有子轮廓和父轮廓角度的差值小于5就++
                for (int s = 0; s < sons.size(); s++) {
                    //画出子轮廓
                    Point2f p2[4];
                    SonsRect[s].points(p2);
                    for (int j = 0; j < 4; j++) {
                        line(src, p2[j], p2[(j + 1) % 4], Scalar(0, 255, 255), 2, 8);  //绘制最小外接矩形每条边
                    }

                    // 角度差统计
                    // if (abs(SonsRect[s].angle - FatherRect.angle) < 5.0) qualifiedSubAngle++;
                    // cout << "(" << SonsRect[s].angle << "-" << FatherRect.angle << ")"
                    //      << abs(SonsRect[s].angle - FatherRect.angle) << endl;
                }
                //
                if (qualifiedSubAngle < 2) continue;

                vector<float> Sons_whratio;//子轮廓的长宽比
                for (int j = 0; j < SonsRect.size(); j++) {
                    float a = SonsRect[j].size.height;
                    float b = SonsRect[j].size.width;
                    if (a < b) swap(a, b);
                    Sons_whratio.push_back(a / b);
                }

                /*//调试：写出每个子轮廓的长宽比
                cout<<"ratios:"<<endl;
                for(auto ratio : Sons_whratio)
                    cout<<ratio<<" ";*/

                //利用子轮廓的长宽比（过于简单）来区分装甲板和其他两个扁长的矩形
                for (int j = 0; j < Sons_whratio.size(); j++) {
                    if ((Sons_whratio[j] > 1) && (Sons_whratio[j] < 2.5)) {
                        if (fan_armorCenters.size() < 50) {
                            fan_armorCenters.push_back(SonsRect[j].center + offset);
                        } else if (fan_armorCenters.size() >= 50) {
                            if (center_index == 50)center_index = 0;
                            fan_armorCenters[center_index] = SonsRect[j].center + offset;
                            center_index++;
                        }

                    }

                }

            } else {
                continue;
            }

        }
    }
    /*********************************分类模式2：利用Lenet进行分类*******************************/
    else if (classiMode==useLenet){

    }
    if (hammerToFall == 0) {
        cout << endl;
        cout << "本帧中没有找到锤子!" << endl;
        return false;
    }//不再进行圆的拟合

    /*********************拟合圆************************/
    float radius;
    circleLeastFit(fan_armorCenters, data.R_center, radius);
    if (sParam.debug) {
        circle(src, data.armorCenter, 5, Scalar(255, 255, 255), 2);
        circle(src, data.preArmorCenter, 5, Scalar(255, 0, 255), 2);
        circle(src, predictAimPoint, 5, Scalar(255, 255, 0), 2);
        circle(src, lastData.R_center, 5, Scalar(255, 255, 255), 2);
        circle(src, lastData.R_center, radius, Scalar(20, 100, 100), 2);
    }
    imshow("father_son", src);
    delete[]findCount;
    return true;
}
void Detect::detect_new(const Mat frame) {
    Mat src = frame;
    // setImage
    Point2f offset = Point2f(0, 0);
    // detect the armor
    if (getArmorCenter_new(src, 1, data, offset,mySonNum) == true)
        lastData = data;
}

// double retTime(double angularVelocity)
// {
//     return 
// }

Point2f  Detect::preArmorCentor(armorData &data , double time)
{
    double increaseAngle ;
    Point2f aimPoint;
    double radious  =  distance(data.armorCenter,data.R_center);
    double angle = countRotationAngle(data.armorCenter ,data.preArmorCenter , radious ,data.R_center);
    double angularVelocity = angle / data.runTime  * 1000;
    // std::cout << "angularVelocity :    "  << angularVelocity  <<endl;
    increaseAngle = (time * angularVelocity);
    while(increaseAngle > 2*PI)
    {
        increaseAngle -=  (2 * PI);
    }
    std::cout << "increaseAngle :    "  << angularVelocity  <<endl;
    if(isClockwise(data))
    {
        std::cout << "isClockwise   " << endl;
        aimPoint = nextCoordinate(data.armorCenter , data.R_center , increaseAngle , true);
    }
    else
    {
        aimPoint = nextCoordinate(data.armorCenter , data.R_center , increaseAngle , false);
    }
    return aimPoint;
}

bool Detect::isClockwise(armorData &data)
{
    Point2f preCenter = data.preArmorCenter;
    Point2f nowCenter = data.armorCenter;
    Point2f R_Center = data.R_center;
    if(nowCenter.x <= R_Center.x  && nowCenter.y <= R_Center.y )  //第一向限
    {
            if(preCenter.x < nowCenter.x && preCenter.y > nowCenter.y)
            {
                return true;
            }
            return false;
    }
    else if(nowCenter.x >= R_Center.x  && nowCenter.y <= R_Center.y )  //第二向限
    {
        if(preCenter.x < nowCenter.x && preCenter.y < nowCenter.y)
        {
            return true;
        }
        return false;
    }
    else if(nowCenter.x >= R_Center.x  && nowCenter.y >= R_Center.y )  //第三向限
    {
        if(preCenter.x > nowCenter.x  &&  preCenter.y < nowCenter.y)
        {
            return true;
        }
        return false;
    }
    else if(nowCenter.x <= R_Center.x  && nowCenter.y >= R_Center.y )  //第四向限
    {
        if(preCenter.x > nowCenter.x  &&  preCenter.y > nowCenter.y)
        {
            return true;
        }
        return false;
    }
}

Point2f Detect::nextCoordinate(Point2f nowCenter ,Point2f R_Center,double increaseAngle ,bool isClockwise)
{
    Point aimPoint = Point2f(0, 0);
    if( !isClockwise )
    {
        aimPoint.x = (nowCenter.x - R_Center.x) * cos(-increaseAngle) - (nowCenter.y - R_Center.y) * sin(-increaseAngle) + R_Center.x;
        aimPoint.y = (nowCenter.x - R_Center.x) * sin(-increaseAngle) + (nowCenter.y - R_Center.y) * cos(-increaseAngle) + R_Center.y;
    }
    else
    {
        aimPoint.x = (nowCenter.x - R_Center.x) * cos(increaseAngle) - (nowCenter.y - R_Center.y) * sin(increaseAngle) + R_Center.x;
        aimPoint.y = (nowCenter.x - R_Center.x) * sin(increaseAngle) + (nowCenter.y - R_Center.y) * cos(increaseAngle) + R_Center.y;
    }
    return aimPoint;
}