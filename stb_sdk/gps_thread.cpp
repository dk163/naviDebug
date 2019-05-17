#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <assert.h>

#include "gps_thread.h"

#include "stb_hw_if.h"

#include "rs232.h"
#include "autonavi_parser.h"

///////////////////////////////////////////////////////////////////////////////
// Control support

//! Signals to any waiting threads the last command has completed
/*!
  \param pControlThreadInfo    : Pointer to main thread data
  \param result                : Last command result to make available to interested parties
*/
static void signal_cmd_complete(ControlThreadInfo* pControlThreadInfo, int result)
{
    assert(pControlThreadInfo);
    //UBX_LOG(LCAT_DEBUG, "Signal to waiting threads the result of the last command: %i", result);
    pthread_mutex_lock(&pControlThreadInfo->threadCmdCompleteMutex);
    pControlThreadInfo->cmdResult = result;
    pthread_cond_signal(&pControlThreadInfo->threadCmdCompleteCond);
    pthread_mutex_unlock(&pControlThreadInfo->threadCmdCompleteMutex);
    //UBX_LOG(LCAT_DEBUG, "Signaling of result completed. Wait for main-thread");

    // Only if the result was positive, the main-thread will signal
    // that it is ready
    if(result==1)
    {
        //UBX_LOG(LCAT_DEBUG, "Wait for ubx-main to signalise that everything is ready for normal operation");
        pthread_mutex_lock(&pControlThreadInfo->threadMainReadyMutex);
        while(!pControlThreadInfo->mainReady)
            pthread_cond_wait(&pControlThreadInfo->threadMainReadyCond,
                              &pControlThreadInfo->threadMainReadyMutex);
        pthread_mutex_unlock(&pControlThreadInfo->threadMainReadyMutex);
        //UBX_LOG(LCAT_DEBUG, "Initialisation complete. main-thread seems to be ready. Continue.");
    }
    else
    {
        //UBX_LOG(LCAT_ERROR, "Signaled error to main-thread. Exit");
    }
}

static void handle_init(ControlThreadInfo* pControlThreadInfo)
{
	assert(pControlThreadInfo);
	// Signal main 
	//UBX_LOG(LCAT_VERBOSE, "(%u): Init state. Signal main-thread to continue", (unsigned int) pthread_self());
    pControlThreadInfo->gpsState = GPS_STOPPED;
	signal_cmd_complete(pControlThreadInfo, 1);
	//UBX_LOG(LCAT_VERBOSE, "Signaled main-thread to continue");
}

static void releaseGpsThreadResources(ControlThreadInfo* pControlThreadInfo)
{
    if (pControlThreadInfo->cmdPipes[0] != -1)
        close(pControlThreadInfo->cmdPipes[0]);
    pControlThreadInfo->cmdPipes[0] = -1;
    if (pControlThreadInfo->cmdPipes[1] != -1)
        close(pControlThreadInfo->cmdPipes[1]);
    pControlThreadInfo->cmdPipes[1] = -1;
}

///////////////////////////////////////////////////////////////////////////////
//! Main gps driver control thread
/*!
  \param pThreadData    : Pointer to thread data
*/
void stb_thread(void *pThreadData)
{
    ControlThreadInfo* pState = (ControlThreadInfo*) pThreadData;
    
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
        //LOC_LOGE("Can not open comport!\n");
        return;
    }
    
    int cport = RS232_GetPortFD(cport_nr);

    handle_init(pState);
    
    while (1) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(cport, &rfds);
        struct timeval tv;
        tv.tv_usec = 200000; // 200000us = 200ms
        if(select(cport+1, &rfds, NULL, NULL, &tv)) {
            int avalibleSize = RBUFSIZE - remainSize;
            int readSize = RS232_PollComport(cport_nr, (unsigned char*)(buffer + remainSize), avalibleSize);
            //printf("avalibleSize=%d readSize=%d\n", avalibleSize, readSize);
            if (readSize <= 0) {
                //LOC_LOGE("RS232_PollComport() error\n");
                break;
            }

            int realSize = remainSize + readSize;
            buffer[realSize] = '\0';
            //LOC_LOGI("%s\n", buffer);
        
            // 解析缓冲区，并通过回调向外传递结果；
            remainSize = parseBuffer(realSize, buffer, callback, &remain);
            //printf("remain=%s\n", remain);
            //printf("remain_strlen=%d remainSize=%d\n", strlen(remain), remainSize);
            
            // copy遗留非完整行 将其copy到buffer的头部，
            if (remainSize > 0) {
                memcpy(buffer, remain, remainSize);
            }
        } else {
            //LOC_LOGE("select() error\n");
            break;
        }
    }
    
    RS232_CloseComport(cport_nr);
    
    releaseGpsThreadResources(pState);
    return;
}

///////////////////////////////////////////////////////////////////////////////
//! Injects a time into the receiver
/*!
  \param timeUtcGps        : Utc time
  \param timeReference    : Time reference
  \param uncertainty    : Uncertaincy
*/
void gps_state_inject_time(GpsUtcTime timeUtcGps, int64_t timeReference, int uncertainty)
{
    return;
}

///////////////////////////////////////////////////////////////////////////////
//! Injects a location into the receiver
/*!
  \param latitude    : Latitude part of location
  \param longitude    : Longitude part of location
  \param accuracy    : Accuracy of location
*/
void gps_state_inject_location(double latitude, double longitude, float accuracy)
{
    return;
}

///////////////////////////////////////////////////////////////////////////////
//! Deletes aiding data in the receiver
/*!
  \param flags    : Flags indicating which data to delete
*/
void gps_state_delete_aiding_data(GpsAidingData flags)
{
    return;
}

///////////////////////////////////////////////////////////////////////////////
//! Sets the interval for the driver to report to the framework
/*!
  \param min_interval    : Reporting interval in ms
*/
void gps_state_set_interval(uint32_t min_interval)
{
    return;
}

///////////////////////////////////////////////////////////////////////////////
//! Releases any resources in the main thread's control data
/*!
  \param pControlThreadInfo    : Pointer to main thread data
*/
void controlThreadInfoRelease(ControlThreadInfo* pControlThreadInfo)
{
    pthread_mutex_destroy(&pControlThreadInfo->threadCmdCompleteMutex);
    pthread_cond_destroy(&pControlThreadInfo->threadCmdCompleteCond);
    pthread_mutex_destroy(&pControlThreadInfo->threadDataAccessMutex);
    pthread_mutex_destroy(&pControlThreadInfo->threadMainReadyMutex);
    pthread_cond_destroy(&pControlThreadInfo->threadMainReadyCond);
}

