#ifndef __AIP_HW_IF_H__
#define __AIP_HW_IF_H__

#include <hardware/gps.h>
#include "gps_thread.h"

///////////////////////////////////////////////////////////////////////////////



class AipHwIf
{
public:
    AipHwIf();
    static struct hw_module_methods_t aip_module_methods;
    static const GpsInterface *getInterface(struct gps_device_t* /*dev*/);

    static AipHwIf* getInstance(void);

    GpsPositionMode getMode(void) const;
    uint32_t getCapabilities(void) const;

private:
    // interface hw module
    static int openAip(const struct hw_module_t* module,
                            char const* name, struct hw_device_t** device);
    static int closeAip(struct hw_device_t* device);
    
    // interface
    static int  init(GpsCallbacks* callbacks);
    static int  start(void);
    static int  stop(void);
    static void cleanup(void);
    static int  injectTime(GpsUtcTime timeGpsUtc,
                int64_t timeReference, int uncertainty);
    static int  injectLocation(double latitude, double longitude, float accuracy);
    static void deleteAidingData(GpsAidingData f);
    static int  setPositionMode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
                                uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time);
    static const void* getExtension(const char* name);
    
    // variables    
    static const GpsInterface s_interface;
    GpsPositionMode m_mode;
    uint32_t m_capabilities;
    
public:
    GpsCallbacks m_callbacks;
    GpsState m_gpsState;
};

#endif /* __AIP_HW_IF_H__ */
