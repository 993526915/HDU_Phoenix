#include"offsetSolve.h"

offsetSolve::offsetSolve(Point2f center ,double dis )
{
    _offset = Point2f(0,0);
    _centerPoint = center;
     _distance = dis;
    _delayTime = dis / BULLET_SPEED;
}



Point2f offsetSolve::getoffset()
{
    _offset.x = 10;
    _offset.y =  _distance *  50;
    return _offset;
}