#include <stdlib.h>
#include <stdio.h>

#include <unistd.h>

#include "log_util.h"

#include "rs232.h"
#include "autonavi_parser.h"


int main(int argc, char *argv[])
{
    LOC_LOGD("gps.default.so loaded.\n");

    char buffer[RBUFSIZE+4];
    const char *remain = 0;
    size_t remainSize = 0;
    
    memset(buffer, '\0', sizeof(buffer));

    char device_name[] = "ttymxc4";
    int cport_nr = RS232_GetPortnr(device_name); // /dev/ttymxc4
    
    int bdrate = 115200;
    char mode[] = {'8', 'N', '1', 0};
    
    if(RS232_OpenComport(cport_nr, bdrate, mode))
    {
        LOC_LOGE("Can not open comport!\n");
        return(1);
    }
    
    int cport = RS232_GetPortFD(cport_nr);
    while (1) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(cport, &rfds);
        struct timeval tv;
        tv.tv_usec = 200000; // 100000us = 100ms = 0.1s
        if(select(cport+1, &rfds, NULL, NULL, &tv)) {
            int avalibleSize = RBUFSIZE - remainSize;
            int readSize = RS232_PollComport(cport_nr, (unsigned char*)(buffer + remainSize), avalibleSize);
            //printf("avalibleSize=%d readSize=%d\n", avalibleSize, readSize);
            if (readSize <= 0) {
                LOC_LOGE("RS232_PollComport() error\n");
                break;
            }

            int realSize = remainSize + readSize;
            buffer[realSize] = '\0';
            LOC_LOGI("%s\n", buffer);
        
            // 解析缓冲区，并通过回调向外传递结果；
            remainSize = parseBuffer(realSize, buffer, callback, &remain);
            //printf("remain=%s\n", remain);
            //printf("remain_strlen=%d remainSize=%d\n", strlen(remain), remainSize);
            
            // copy遗留非完整行 将其copy到buffer的头部，
            if (remainSize > 0) {
                memcpy(buffer, remain, remainSize);
            }
        } else {
            LOC_LOGE("select() error\n");
            break;
        }
    }
    
    RS232_CloseComport(cport_nr);
    
    return 0;
}
