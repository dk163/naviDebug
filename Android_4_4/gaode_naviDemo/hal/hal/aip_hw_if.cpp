#include <errno.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <new>
#include "aip_log.h"

#ifndef ANDROID_BUILD
// Needed for Linux build
#include <malloc.h>
#endif

#include "aip_hw_if.h"

#include "gps_thread.h"


/*******************************************************************************
 * HAL MODULE
 ******************************************************************************/
hw_module_t HAL_MODULE_INFO_SYM = {
    tag:                    HARDWARE_MODULE_TAG,
    version_major:          2,
    version_minor:          0,
    id:                     GPS_HARDWARE_MODULE_ID,
    name:                   "AIP200 Module",
    author:                 "Autonavi CN, Inc.",
    methods:                &AipHwIf::aip_module_methods,
    dso:                    NULL,
    reserved:               {0}
};

struct hw_module_methods_t AipHwIf::aip_module_methods =
{
    open: AipHwIf::openAip
};

int AipHwIf::openAip(const struct hw_module_t* module,
                         char const* name,
                         struct hw_device_t** device)
{
    ((void) (name));
    struct gps_device_t *dev = new gps_device_t;
    memset(dev, 0, sizeof(*dev));
    dev->common.tag           = HARDWARE_DEVICE_TAG;
    dev->common.version       = 0;
    dev->common.module        = const_cast<struct hw_module_t*>(module);
    dev->common.close         = AipHwIf::closeAip;
    dev->get_gps_interface    = AipHwIf::getInterface;
    *device = (struct hw_device_t*) (void *) dev;

    return 0;
}

int AipHwIf::closeAip(struct hw_device_t* device)
{
    delete device;
    return 0;
}


/*******************************************************************************
 * INTERFACE
 ******************************************************************************/

static AipHwIf s_myIf;

const GpsInterface AipHwIf::s_interface = {
    size:                  sizeof(GpsInterface),
    init:                  AipHwIf::init,
    start:                 AipHwIf::start,
    stop:                  AipHwIf::stop,
    cleanup:               AipHwIf::cleanup,
    inject_time:           AipHwIf::injectTime,
    inject_location:       AipHwIf::injectLocation,
    delete_aiding_data:    AipHwIf::deleteAidingData,
    set_position_mode:     AipHwIf::setPositionMode,
    get_extension:         AipHwIf::getExtension,
};

AipHwIf::AipHwIf()
{
    m_mode = GPS_POSITION_MODE_MS_BASED;
    m_capabilities = 0;
    memset(&m_callbacks, 0, sizeof(m_callbacks));

    m_gpsState.init = false;
    m_gpsState.thread = pthread_t(0);
    m_gpsState.ctl_fd[0] = -1;
    m_gpsState.ctl_fd[1] = -1;
    m_gpsState.status.size = sizeof(GpsStatus);
    m_gpsState.status.status = GPS_STATUS_NONE;
}

const GpsInterface *AipHwIf::getInterface(struct gps_device_t* /*dev*/)
{
   return &s_interface;
}

AipHwIf *AipHwIf::getInstance()
{
    return &s_myIf;
}

GpsPositionMode AipHwIf::getMode(void) const
{
    return m_mode;
};

uint32_t AipHwIf::getCapabilities(void) const
{
    return m_capabilities;
};

// driver version
static const char *GetVersion(void)
{
    return "0.5";
}

int AipHwIf::init(GpsCallbacks* callbacks)
{
    if (s_myIf.m_gpsState.init) {
        AIP_LOGE("aip_hw already initialized");
        return 0; // 返回成功，因为实际已经初始化完成
    }

    AIP_LOGV("aip_hw initialize");

    memcpy(&s_myIf.m_callbacks, callbacks,
                (callbacks->size < sizeof(GpsCallbacks)) ? callbacks->size : sizeof(GpsCallbacks));
    if (callbacks->size != sizeof(GpsCallbacks)) {
        AIP_LOGW("callback size %zd != %zd", callbacks->size, sizeof(GpsCallbacks));
    }

    /* 模组暂时不支持 capabilities */
    /* 
    s_myIf.m_capabilities = GPS_CAPABILITY_SCHEDULING;
    s_myIf.m_callbacks.set_capabilities_cb(s_myIf.m_capabilities);
    */

    if (gps_state_init() < 0) {
        return -1;
    }
    
    return 0;
}

int AipHwIf::start(void)
{
    if (!s_myIf.m_gpsState.init) {
        AIP_LOGE("start() called with uninitialized state!");
        return -1;
    }

    gps_state_start();
    return 0;
}

int AipHwIf::stop(void)
{
    if (!s_myIf.m_gpsState.init) {
        AIP_LOGE("stop() called with uninitialized state!");
        return -1;
    }

    gps_state_stop();
    return 0;
}

void AipHwIf::cleanup(void)
{
    if (!s_myIf.m_gpsState.init) {
        AIP_LOGE("cleanup() called with uninitialized state!");
        return;
    }

    gps_state_done();
    return;
}

int AipHwIf::injectTime(GpsUtcTime timeGpsUtc, int64_t timeReference, int uncertainty)
{
    return gps_inject_time(timeGpsUtc, timeReference, uncertainty);
}

int AipHwIf::injectLocation(double latitude, double longitude, float accuracy)
{
    return gps_inject_location(latitude, longitude, accuracy);
}

void AipHwIf::deleteAidingData(GpsAidingData flags)
{
    return gps_delete_aiding_data(flags);
}

int AipHwIf::setPositionMode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
            uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time)
{
    return gps_set_position_mode(mode, recurrence, min_interval, preferred_accuracy, preferred_time);
}


const void* AipHwIf::getExtension(const char* name)
{
    return NULL;
}

