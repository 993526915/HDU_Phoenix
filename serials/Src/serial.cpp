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

Serial::Serial(int nSpeed, char nEvent, int nBits, int nStop) : nSpeed(nSpeed), nEvent(nEvent), nBits(nBits), nStop(nStop)
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
        return false;
    }
    if ((fd = open(name.data(), O_RDWR)) < 0)
    {
        return false;
    }
    if (set_opt(fd, nSpeed, nEvent, nBits, nStop) < 0)
    {
        return false;
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
