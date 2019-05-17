#include <errno.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <new>
#include "ubx_log.h"

#ifndef ANDROID_BUILD
// Needed for Linux build
#include <malloc.h>
#endif

#include "stb_hw_if.h"

#include "gps_thread.h"

static ControlThreadInfo  s_controlThreadInfo = {
    gpsState:               GPS_UNKNOWN,
    cmdPipes:               { -1, -1 },
    cmdResult:              0,
    mainReady:              false,
    threadMainReadyCond:    PTHREAD_COND_INITIALIZER,
    threadMainReadyMutex:   PTHREAD_MUTEX_INITIALIZER,
    threadCmdCompleteCond:  PTHREAD_COND_INITIALIZER,
    threadCmdCompleteMutex: PTHREAD_MUTEX_INITIALIZER,
    clientCount:            0,
    threadDataAccessMutex:  PTHREAD_MUTEX_INITIALIZER
};
static pthread_t s_mainControlThread = (pthread_t)NULL;


/*******************************************************************************
 * HAL MODULE
 ******************************************************************************/
hw_module_t HAL_MODULE_INFO_SYM = {
    tag:                    HARDWARE_MODULE_TAG,
    version_major:          2,
    version_minor:          0,
    id:                     GPS_HARDWARE_MODULE_ID,
    name:                   "STB Module",
    author:                 "Autonavi CN, Inc.",
    methods:                &StbHwIf::stb_module_methods,
    dso:                    NULL,
    reserved:               {0}
};

struct hw_module_methods_t StbHwIf::stb_module_methods =
{
    open: StbHwIf::openStb
};

int StbHwIf::openStb(const struct hw_module_t* module,
                         char const* name,
                         struct hw_device_t** device)
{
    ((void) (name));
    struct gps_device_t *dev = new gps_device_t;
    memset(dev, 0, sizeof(*dev));
    dev->common.tag           = HARDWARE_DEVICE_TAG;
    dev->common.version       = 0;
    dev->common.module        = const_cast<struct hw_module_t*>(module);
    dev->common.close         = StbHwIf::closeStb;
    dev->get_gps_interface    = StbHwIf::getInterface;
    *device = (struct hw_device_t*) (void *) dev;

    return 0;
}

int StbHwIf::closeStb(struct hw_device_t* device)
{
    delete device;
    return 0;
}


/*******************************************************************************
 * INTERFACE
 ******************************************************************************/

static StbHwIf s_myIf;

const GpsInterface StbHwIf::s_interface = {
    size:                  sizeof(GpsInterface),
    init:                  StbHwIf::init,
    start:                 StbHwIf::start,
    stop:                  StbHwIf::stop,
    cleanup:               StbHwIf::cleanup,
    inject_time:           StbHwIf::injectTime,
    inject_location:       StbHwIf::injectLocation,
    delete_aiding_data:    StbHwIf::deleteAidingData,
    set_position_mode:     StbHwIf::setPositionMode,
    get_extension:         StbHwIf::getExtension,
};

StbHwIf::StbHwIf()
{
    m_ready = false;
    m_mode = GPS_POSITION_MODE_MS_BASED;
    m_lastStatusValue = GPS_STATUS_NONE;
    m_capabilities = 0;
    memset(&m_callbacks,0,sizeof(m_callbacks));
}

const GpsInterface *StbHwIf::getInterface(struct gps_device_t* /*dev*/)
{
   return &s_interface;
}

StbHwIf* StbHwIf::getInstance()
{
    return &s_myIf;
}

GpsPositionMode StbHwIf::getMode(void) const
{
    return m_mode;
};

uint32_t StbHwIf::getCapabilities(void) const
{
    return m_capabilities;
};

// driver version
static const char *GetVersion(void)
{
    return "0.5";
}

int StbHwIf::init(GpsCallbacks* callbacks)
{
    UBX_LOG(LCAT_DEBUG, "before init, s_myIf.m_ready=%d", s_myIf.m_ready);
    
    if (s_myIf.m_ready)
    {
        UBX_LOG(LCAT_ERROR, "already initialized");
        return 0;    // Report success since we are already initialised
    }

    //UBX_LOG(LCAT_VERBOSE, "Driver version %s", GetVersion());
    //UBX_LOG(LCAT_VERBOSE, "libMGA version %s", mgaGetVersion());
    //UBX_LOG(LCAT_VERBOSE, "libParser version %s", CParserBuffer::GetVersion());
    UBX_LOG(LCAT_VERBOSE, "stb_hw init");

#if (PLATFORM_SDK_VERSION > 8 /* >2.2 */)
    //UBX_LOG(LCAT_VERBOSE, "");
    memcpy(&s_myIf.m_callbacks, callbacks,
                (callbacks->size < sizeof(GpsCallbacks)) ? callbacks->size : sizeof(GpsCallbacks));
    if (callbacks->size != sizeof(GpsCallbacks))
        //UBX_LOG(LCAT_WARNING, "callback size %zd != %zd", callbacks->size, sizeof(GpsCallbacks));
#endif

    //UBX_LOG(LCAT_DEBUG, "(%u): Initializing - pid %i", (unsigned int) pthread_self(), getpid());

#if (PLATFORM_SDK_VERSION > 8 /* >2.2 */)
    //s_myIf.m_capabilities = GPS_CAPABILITY_SCHEDULING;
 #if (PLATFORM_SDK_VERSION >= 14 /* >=4.0 */)
    //s_myIf.m_capabilities |= GPS_CAPABILITY_ON_DEMAND_TIME;
 #endif
 #ifdef SUPL_ENABLED
    //s_myIf.m_capabilities |=  GPS_CAPABILITY_MSB | GPS_CAPABILITY_MSA;
 #endif
    //UBX_LOG(LCAT_VERBOSE, "set_capabilities=%d(%s)", s_myIf.m_capabilities, _LOOKUPSTRX(s_myIf.m_capabilities, GpsCapabilityFlags));
    s_myIf.m_callbacks.set_capabilities_cb(s_myIf.m_capabilities);
    s_mainControlThread = s_myIf.m_callbacks.create_thread_cb("gps thread", stb_thread, &s_controlThreadInfo);
#endif
    // Wait for ubx-thread to signal that the setup has been finished
    // and it is ready to run
    //UBX_LOG(LCAT_DEBUG, "Wait for ubx-thread to get ready");

    pthread_mutex_lock(&s_controlThreadInfo.threadCmdCompleteMutex);
    while(!s_controlThreadInfo.cmdResult)
        pthread_cond_wait(&s_controlThreadInfo.threadCmdCompleteCond,
                          &s_controlThreadInfo.threadCmdCompleteMutex);
    s_myIf.m_ready = (s_controlThreadInfo.cmdResult == 1);

    pthread_mutex_unlock(&s_controlThreadInfo.threadCmdCompleteMutex);
    //UBX_LOG(LCAT_DEBUG, "Initialized complete: result %i. ubx-thread %sready", s_myIf.m_ready, s_myIf.m_ready?"":"NOT ");
    if (!s_myIf.m_ready)
    {
        // Init failed -  release resources
        //UBX_LOG(LCAT_ERROR, "Initialisation failed! Freeing the ressources");
        controlThreadInfoRelease(&s_controlThreadInfo);
    }
    else // If everything is fine, tell stb_thread to continue
    {
        // Tell the ubx-thread that the main-thread is ready
        //UBX_LOG(LCAT_DEBUG, "Signal to the waiting ubx-thread that main-thread is ready");
        pthread_mutex_lock(&s_controlThreadInfo.threadMainReadyMutex);
        gpsStatus(GPS_STATUS_ENGINE_OFF);
        s_controlThreadInfo.mainReady=true;
        pthread_cond_signal(&s_controlThreadInfo.threadMainReadyCond);
        pthread_mutex_unlock(&s_controlThreadInfo.threadMainReadyMutex);
        //UBX_LOG(LCAT_DEBUG, "Signaling of readyness completed. Return normally and let ubx-thread continue");
    }
    UBX_LOG(LCAT_DEBUG, "after init, s_myIf.m_ready=%d", s_myIf.m_ready);
    return s_myIf.m_ready ? 0 : 1;
}

int StbHwIf::start(void)
{
    return 0;
}

int StbHwIf::stop(void)
{
    return 0;
}

void StbHwIf::cleanup(void)
{
    return;
}

int StbHwIf::injectTime(GpsUtcTime timeGpsUtc, int64_t timeReference, int uncertainty)
{
    return 0;
}

int StbHwIf::injectLocation(double latitude, double longitude, float accuracy)
{
    return 0;
}

void StbHwIf::deleteAidingData(GpsAidingData flags)
{
    return;
}

int StbHwIf::setPositionMode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
            uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time)
{
    return 0;
}


const void* StbHwIf::getExtension(const char* name)
{
    return NULL;
}

///////////////////////////////////////////////////////////////////////////////
// operations

void StbHwIf::gpsStatus(GpsStatusValue gpsStatusValue)
{
    if (!s_myIf.m_ready)
    {
        //UBX_LOG(LCAT_ERROR, "class not initialized");
        return;
    }

    if (gpsStatusValue == s_myIf.m_lastStatusValue)
    {
        /*
        UBX_LOG(LCAT_VERBOSE, "current GPS status: (still) %d (%s)",
             gpsStatusValue ,_LOOKUPSTR(gpsStatusValue, GpsStatusValue));
        */
    }
    else
    {
        /*
        UBX_LOG(LCAT_VERBOSE, "GPS status change: %d (%s) => %d (%s)",
             s_myIf.m_lastStatusValue,
             _LOOKUPSTR(s_myIf.m_lastStatusValue, GpsStatusValue),
             gpsStatusValue,
             _LOOKUPSTR(gpsStatusValue, GpsStatusValue));
        */
        s_myIf.m_lastStatusValue = gpsStatusValue;
        if (gpsStatusValue == GPS_STATUS_SESSION_END)
        {
            GpsSvStatus svStatus;
            memset(&svStatus, 0, sizeof(GpsSvStatus));
            svStatus.size = sizeof(GpsSvStatus);
            s_myIf.m_callbacks.sv_status_cb(&svStatus);
        }
        GpsStatus gpsStatusVar;
        gpsStatusVar.size = sizeof(gpsStatusVar);
        gpsStatusVar.status = gpsStatusValue;
        s_myIf.m_callbacks.status_cb(&gpsStatusVar);
    }
}

#if (PLATFORM_SDK_VERSION > 8 /* >2.2 */)
void StbHwIf::nmea(const char* data, int length)
{
    if (!s_myIf.m_ready)
    {
        //UBX_LOG(LCAT_ERROR, "class not initialized");
        return;
    }
    if (s_myIf.m_callbacks.nmea_cb != NULL)
    {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        GpsUtcTime gpsUtcTime = (long long) tv.tv_sec * 1000 + (long long) tv.tv_usec / 1000;
        s_myIf.m_callbacks.nmea_cb(gpsUtcTime, data, length);
    }
}
#endif

#if (PLATFORM_SDK_VERSION >= 14 /* >=4.0 */)
void StbHwIf::requestUtcTime(void)
{
    if (!s_myIf.m_ready)
    {
        //UBX_LOG(LCAT_ERROR, "class not initialized");
        return;
    }
    if (s_myIf.m_callbacks.request_utc_time_cb != NULL)
    {
        //UBX_LOG(LCAT_VERBOSE, "");
        s_myIf.m_callbacks.request_utc_time_cb();
    }
}
#endif

///////////////////////////////////////////////////////////////////////////////
// Debug / Testing support

#ifndef ANDROID_BUILD

extern "C" void endControlThread(void)
{
    //UBX_LOG(LCAT_DEBUG, "Send thread exit command");
    bool ok = controlThreadInfoSendCmd(&s_controlThreadInfo, CMD_EXIT);
    pthread_join(s_mainControlThread, NULL);
    s_mainControlThread = NULL;
    //UBX_LOG(LCAT_DEBUG, "Thread exited ok=%d", ok);
}
#endif

