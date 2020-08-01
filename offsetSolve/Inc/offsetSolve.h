#include <iostream>
#include"opencv4/opencv2/opencv.hpp"
using namespace std;
using namespace cv;
#define FOCUS_PIXAL 1269
#define BULLET_SPEED 12
class offsetSolve
{
public:
    offsetSolve(Point2f center ,double dis);
    ~offsetSolve(){}
    Point2f getoffset();
private:
    Point2f _offset;
    Point2f _centerPoint;
    double _delayTime;
    double _distance;
};