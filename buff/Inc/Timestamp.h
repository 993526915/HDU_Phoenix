#ifndef _TIMESTAMP_H_
#define  _TIMESTAMP_H_
#include <chrono>
using namespace std::chrono;
class CELLTimestamp
{
public:
    CELLTimestamp() 
    {
      update();
    }
    ~CELLTimestamp() {}
    /*
      更新时间
    */
    void update()
    {
      _begin = high_resolution_clock::now();
    }
    /*
      获取当前秒
    */
    double getElapsedSecond()
    {
      return this->getElapsedTimeInMicroSec() * 0.000001;
    }
    /*
      获取当前毫秒
    */
    double getElapsedTimeInMilliSec()
    {
      return this->getElapsedTimeInMicroSec() * 0.001;
    }
    /*
      获取当前微妙
    */
    long long  getElapsedTimeInMicroSec()
    {
      return duration_cast<microseconds>(high_resolution_clock::now() - _begin).count();
    }
protected:
	time_point<high_resolution_clock> _begin;
private:
};

#endif