#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <netinet/in.h>
#include "rs485Service.h"

#define RS485COM "/dev/ttyAMA1"

Rs485Service::Rs485Service()
{
    m_devFd = -1;
}

Rs485Service::~Rs485Service()
{
}

Rs485Service& Rs485Service::Get()
{
    static Rs485Service Rs485server;
    return Rs485server;
}

int Rs485Service::InitRs485Dev(uint32 bound)
{
    system("himm 0x1204008C 1");
    system("himm 0x12040094 1");
    
    system("himm 0x12040090 0");
    system("echo 42 > /sys/class/gpio/export");
    system("echo out > /sys/class/gpio/gpio42/direction");
    system("echo 0 > /sys/class/gpio/gpio42/value");
    
    m_devFd = open(RS485COM, O_RDWR | O_NOCTTY); //加上O_NDELAY，就变为了非阻塞
    if (m_devFd != -1)
    {
        struct termios cfg;
        
        memset(&cfg, 0, sizeof(cfg));
        tcgetattr(0, &cfg);
        switch(bound)
        {
            case 19200:
                cfsetispeed(&cfg, B19200);
                cfsetispeed(&cfg, B19200);
                m_bound = 19200;
                break;
            case 9600:
                cfsetispeed(&cfg, B9600);
                cfsetispeed(&cfg, B9600);
                m_bound = 9600;
                break;
            default:
                cfsetispeed(&cfg, B19200);
                cfsetispeed(&cfg, B19200);
                m_bound = 19200;
                break;
        }
        cfg.c_oflag &= ~(ONLCR);
        cfg.c_oflag &= (OCRNL);
        cfg.c_iflag &= (INLCR);
        
        cfg.c_cflag |= CLOCAL | CREAD; //使能串口输入
        //cfg.c_lflag |= ICANON; //标准模式
        cfg.c_lflag &= ~ICANON;//原始模式
        cfg.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
        //8bit数据
        cfg.c_cflag &= ~CSIZE;
        cfg.c_cflag |= CS8;
        //1bit停止位
        cfg.c_cflag &= ~CSTOPB;
        //无校验
        cfg.c_cflag &= ~PARENB;
        //禁用硬件流控制：
        cfg.c_cflag &= ~CRTSCTS;
        
      //  cfg.c_cc[VTIME] = 1; //设置超时时间，如果采用非阻塞模式则不设置
        cfg.c_cc[VMIN] = 1;     //设置最小接收的数据长度

        //清楚输入输出缓冲区
        tcflush(m_devFd, TCIOFLUSH);
        tcsetattr(m_devFd, TCSANOW, &cfg);
        return OPEN_485_SUCCESS;
    }
    else
    {
        printf("open dev err.\n");
        return OPEN_485_FAIL;
    }
}

void Rs485Service::UninitRs485Dev()
{
    if (m_devFd != -1)
    {
        struct termios cfg;
        memset(&cfg, 0, sizeof(cfg));
        tcgetattr(0, &cfg);
        cfg.c_cc[VTIME] = 1;
        tcflush(m_devFd, TCIOFLUSH);
        tcsetattr(m_devFd, TCSANOW, &cfg);
        close(m_devFd);
    }
}

int Rs485Service::Rs485Read(ubyte* buf, uint32 size)
{
    int rlen = -1;
    if (m_devFd != -1)
    {
        rlen = read(m_devFd, buf, size);
        tcflush(m_devFd, TCIOFLUSH);
    }
    return rlen;
}

int Rs485Service::Rs485Write(const ubyte* data, uint32 len)
{
    int wlen = -1;

    //485是半双工，置为发送状态
    system("echo 1 > /sys/class/gpio/gpio42/value");   
    if (m_devFd != -1)
    {     
        wlen = write(m_devFd, data, len);
    }

   //等待数据输出完毕
    tcdrain(m_devFd);
    //清空输入输出缓冲区
    tcflush(m_devFd, TCIOFLUSH);
    //485是半双工，置为接收状态
    system("echo 0 > /sys/class/gpio/gpio42/value");
    
    return wlen;
}

