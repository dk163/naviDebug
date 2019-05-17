#ifndef __GPS_THREAD_H__
#define __GPS_THREAD_H__

#include <pthread.h>
#include "stb_hw_if.h"

///////////////////////////////////////////////////////////////////////////////
// Definitions & Types

typedef  enum {
	CMD_NONE,
	CMD_START_SI,
	CMD_STOP_SI,
	CMD_START_NI,
	CMD_STOP_NI
#ifndef ANDROID_BUILD
    , CMD_EXIT
#endif
} THREAD_CMDS;

typedef enum
{
	GPS_UNKNOWN,
	GPS_STARTED,
	GPS_STOPPING,
	GPS_STOPPED
} GPS_THREAD_STATES;

typedef struct {                                        //!< Main Gps (control) thread data structure containing
                                                        //!< thread state & communication info
    GPS_THREAD_STATES       gpsState;                   //!< Main state of Gps driver
    int                     cmdPipes[2];                //!< Handles to command communication pipes to main thread
    int                     cmdResult;                  //!< Communication command result field
    bool                    mainReady;                  //!< Communication main thread ready
    pthread_cond_t          threadMainReadyCond;        //!< Condition used in thread synchronisation
    pthread_mutex_t         threadMainReadyMutex;       //!< Mutex used in thread synchronisation
    pthread_cond_t          threadCmdCompleteCond;      //!< Condition used in thread synchronisation
    pthread_mutex_t         threadCmdCompleteMutex;     //!< Mutex used in thread synchronisation
    int                        clientCount;             //!< Count of how many 'clients' are using the driver
                                                        //!< Should the framework + number of NI sessions active
    pthread_mutex_t            threadDataAccessMutex;   //!< Mutex to control the access to data in this structure
} ControlThreadInfo;

typedef void (* requestStart)(void* pContext);		//!< Function prototype for 'Start' request event hander
typedef void (* requestStop)(void* pContext);		//!< Function prototype for 'Stop' request event handler

typedef struct 							//!< Event handler interface
{
    requestStart requestStart_cb;		//!< 'Start' request event handler
    requestStop requestStop_cb;			//!< 'Stop' request event handler
} GpsControlEventInterface;

///////////////////////////////////////////////////////////////////////////////
// API

void stb_thread(void *pThreadData);

void gps_state_inject_time(GpsUtcTime timeUtcGps, int64_t timeReference, int uncertainty);
void gps_state_inject_location(double latitude, double longitude, float accuracy);
void gps_state_delete_aiding_data(GpsAidingData flags);
void gps_state_set_interval(uint32_t min_interval);
void gps_state_agps_injectData(const char* data, int length);

void controlThreadInfoInit(ControlThreadInfo* pControlThreadInfo);
void controlThreadInfoRelease(ControlThreadInfo* pControlThreadInfo);
bool controlThreadInfoSendCmd(ControlThreadInfo* pControlThreadInfo, THREAD_CMDS cmd);
//void controlThreadInfoSetIF(ControlThreadInfo* pControlThreadInfo, CGpsIf* pInterface);

#ifndef UNDEBUG
extern pthread_t g_gpsDrvMainThread;
#endif

#endif /* __GPS_THREAD_H__ */
