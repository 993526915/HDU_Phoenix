#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include"opencv2/opencv.hpp"
#include"ArmorDector.h"

typedef __uint8_t uint8_t;
typedef __uint16_t uint16_t;
typedef __uint32_t uint32_t;
typedef __uint64_t uint64_t;

#define IMAGE_CENTER_X 327
#define IMAGE_CENTER_Y 230
#define FOCUS_PIXAL 1269
#define PI (3.14159265459)


struct recvData
{
    int  mode;
};
class Serial 
{
public:
    // /*
    //  * @Breif:所需控制模式
    //  */
    enum TaskMode
    {
        NO_TASK         =   (uint8_t)(0x00),    //手动控制
        AUTO_SHOOT      =   (uint8_t)(0x03)     //自动射击
    };
    // /* @Brief:
    //  *      SYSTEM_ERROR:   System error catched. May be caused by wrong port number,
    //  *                      fragile connection between Jetson and STM, STM shutting
    //  *                      down during communicating or the sockets being suddenly
    //  *                      plugged out.
    //  *      OJBK:         Everything all right
    //  *      PORT_OCCUPIED:  Fail to close the serial port
    //  *      READ_WRITE_ERROR: Fail to write to or read from the port
    //  *      CORRUPTED_FRAME: Wrong frame format
    //  *      TIME_OUT:       Receiving time out
    //  */
    enum ErrorCode
    {
        SYSTEM_ERROR    = 1,
        OJBK            = 0,
        PORT_OCCUPIED   = -1,
        READ_WRITE_ERROR= -2,
        CORRUPTED_FRAME = -3,
        TIME_OUT        = -4,
        USB_CANNOT_FIND=-5
    };
    Serial();
    //Serial(int nSpeed = 115200, char nEvent = 'N', int nBits = 8, int nStop = 1);
    ~Serial();

    bool InitPort(int nSpeed = 115200, char  nEvent = 'N', int nBits = 8, int nStop = 1);
//    int GetBytesInCOM() const ;
    bool WriteData(const unsigned char* pData, unsigned int length);
    bool ReadData(unsigned char* buffer, unsigned int length);
    static bool sendTarget(Serial &serial, float x, float y);
    bool sendBoxPosition( ArmorDetector &Arm,Serial &serial);
    int getFd()
    {
        return fd;
    }

private:
    int fd;
    int nSpeed;
    char nEvent;
    int nBits;
    int nStop;
    int mode ;
    int set_opt(int fd, int nSpeed, char nEvent, int nBits, int nStop);
};

#endif /* _SERIAL_H_ */
