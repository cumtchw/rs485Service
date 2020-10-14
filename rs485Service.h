#ifndef __RS485_SERVER_H__
#define __RS485_SERVER_H__

#include "dataType.h"

typedef enum
{
    OPEN_485_SUCCESS = 0,
    OPEN_485_FAIL,  
}Open485State;

class Rs485Service
{
    
private:
    Rs485Service();
    ~Rs485Service();
    
public:
    static Rs485Service& Get();
    
public:
    int Rs485Read(ubyte *buf, uint32 size);
    int Rs485Write(const ubyte *data, uint32 len);
    int InitRs485Dev(uint32 bound);
    void UninitRs485Dev();
    
private:
    int m_devFd;
    int m_bound;
    int32_t m_485WriteState; //true 485处于写状态
};

#endif
