#include <serial.h>
//#include <options.h>
#include <iostream>

//#define LOG_LEVEL LOG_NONE
//#include <log.h>
#include <string.h>

using namespace std;

string get_uart_dev_name()
{
    FILE *ls = popen("ls /dev/car --color=never", "r");
    char name[20] = {0};
    fscanf(ls, "%s", name);
    return name;
}
Serial::Serial():nSpeed(115200), nEvent('N'), nBits(8), nStop(1)
{
    if (false)
    {
        // LOGM("Wait for serial be ready!");
        while (!InitPort(nSpeed, nEvent, nBits, nStop))
            ;
        // LOGM("Port set successfully!");
    }
    else
    {
        if (InitPort(nSpeed, nEvent, nBits, nStop))
        {
            // LOGM("Port set successfully!");
        }
        else
        {
            std::cout << ("Port set fail!");
        }
    }
}
// Serial::Serial(int nSpeed, char nEvent, int nBits, int nStop) : nSpeed(nSpeed), nEvent(nEvent), nBits(nBits), nStop(nStop)
// {
//     if (false)
//     {
//         // LOGM("Wait for serial be ready!");
//         while (!InitPort(nSpeed, nEvent, nBits, nStop))
//             ;
//         // LOGM("Port set successfully!");
//     }
//     else
//     {
//         if (InitPort(nSpeed, nEvent, nBits, nStop))
//         {
//             // LOGM("Port set successfully!");
//         }
//         else
//         {
//             std::cout << ("Port set fail!");
//         }
//     }
// }

Serial::~Serial()
{
    close(fd);
    fd = -1;
}

bool Serial::InitPort(int nSpeed, char nEvent, int nBits, int nStop)
{
    string name = get_uart_dev_name();
    if (name == "")
    {
        std::cout << "no name " << std::endl;
        return Serial::USB_CANNOT_FIND;
    }
    if ((fd = open(name.data(), O_RDWR)) < 0)
    {
        return Serial::USB_CANNOT_FIND;
    }
    if (set_opt(fd, nSpeed, nEvent, nBits, nStop) < 0)
    {
       return Serial::USB_CANNOT_FIND;
    }
    return true;
}

//int GetBytesInCOM() const {
//
//}

bool Serial::WriteData(const unsigned char *pData, unsigned int length)
{
    int cnt = 0, curr = 0;
    if (fd <= 0)
    {
        if (false)
        {
            InitPort(nSpeed, nEvent, nBits, nStop);
        }
        return false;
    }
    while ((curr = write(fd, pData + cnt, length - cnt)) > 0 && (cnt += curr) < length)
        ;
    if (curr < 0)
    {
        std::cout << ("Serial offline!");
        close(fd);
        if (false)
        {
            InitPort(nSpeed, nEvent, nBits, nStop);
        }
        return false;
    }
    return true;
}

bool Serial::ReadData(unsigned char *buffer, unsigned int length)
{
    int cnt = 0, curr = 0;
    while ((curr = read(fd, buffer + cnt, length - cnt)) > 0 && (cnt += curr) < length)
        ;
    if (curr < 0)
    {
        std::cout << ("Serial offline!");
        close(fd);
        if (false)
        {
            InitPort(nSpeed, nEvent, nBits, nStop);
        }
        return false;
    }
    return true;
}

int Serial::set_opt(int fd, int nSpeed, char nEvent, int nBits, int nStop)
{
    termios newtio{}, oldtio{};
    if (tcgetattr(fd, &oldtio) != 0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    default:
        break;
    }

    switch (nEvent)
    {
    case 'O': //奇校验
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E': //偶校验
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N': //无校验
        newtio.c_cflag &= ~PARENB;
        break;
    default:
        break;
    }

    switch (nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }

    if (nStop == 1)
    {
        newtio.c_cflag &= ~CSTOPB;
    }
    else if (nStop == 2)
    {
        newtio.c_cflag |= CSTOPB;
    }

    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");

    return 0;
}
bool Serial::sendTarget(Serial &serial, float x, float y)
{
    static short x_tmp, y_tmp, z_tmp;
    uint8_t buff[10];

    union f_data {
        float temp;
        unsigned char fdata[4];
    } float_data_x, float_data_y;

    float_data_x.temp = x;
    float_data_y.temp = y;

    buff[0] = 's';
    buff[1] = static_cast<char>(float_data_x.fdata[0]);
    buff[2] = static_cast<char>(float_data_x.fdata[1]);
    buff[3] = static_cast<char>(float_data_x.fdata[2]);
    buff[4] = static_cast<char>(float_data_x.fdata[3]);
    buff[5] = static_cast<char>(float_data_y.fdata[0]);
    buff[6] = static_cast<char>(float_data_y.fdata[1]);
    buff[7] = static_cast<char>(float_data_y.fdata[2]);
    buff[8] = static_cast<char>(float_data_y.fdata[3]);
    buff[9] = 'e';

    return serial.WriteData(buff, sizeof(buff));
}

bool Serial::sendBoxPosition( ArmorDetector &Arm,Serial &serial)
{
    std::vector<cv::Point2f> Points = Arm.getArmorVertex();
    cv::Point aimPoint;
    aimPoint.x = aimPoint.y = 0;
     for (const auto &point : Points)
     {
        aimPoint.x += point.x;
        aimPoint.y += point.y;
    }
    aimPoint.x = aimPoint.x / 4;
    aimPoint.y = aimPoint.y / 4;
    float dx = aimPoint.x - IMAGE_CENTER_X;
    float dy = aimPoint.y - IMAGE_CENTER_Y;
    float yaw = atan(dx / FOCUS_PIXAL) * 180 / PI;
    float pitch = atan(dy / FOCUS_PIXAL) * 180 / PI;
    std::cout<< "  "
              << " yaw: " << yaw << " pitch " << pitch<<std::endl;
    return sendTarget(serial, yaw, pitch);
}

// bool Serial::get_data()
// {
//     char buff[10];
//     bool is_ok = ReadData(buff,8);
//     if(!is_ok)
//     {
//         return false;
//     }
//     if(buff[0] != 's' || buff[2] !=  'e')
//     {

//     }
// }