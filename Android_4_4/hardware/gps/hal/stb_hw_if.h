#ifndef __STB_HW_IF_H__
#define __STB_HW_IF_H__

#include <hardware/gps.h>

///////////////////////////////////////////////////////////////////////////////

class StbHwIf
{
public:
    StbHwIf();
    static struct hw_module_methods_t stb_module_methods;
    static const GpsInterface *getInterface(struct gps_device_t* /*dev*/);

    static StbHwIf* getInstance(void);

    GpsPositionMode getMode(void) const;
    uint32_t getCapabilities(void) const;
    
    // operations
    static void gpsStatus(GpsStatusValue gpsStatusValue);
    static void nmea(const char* data, int length);
#if (PLATFORM_SDK_VERSION >= 14 /* =4.0 */)
    static void requestUtcTime(void);
#endif

private:
    // interface hw module
    static int openStb(const struct hw_module_t* module,
                            char const* name, struct hw_device_t** device);
    static int closeStb(struct hw_device_t* device);
    
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
    bool m_ready;
    GpsPositionMode m_mode;
    GpsStatusValue m_lastStatusValue;
    uint32_t m_capabilities;
    
public:
    GpsCallbacks m_callbacks;
};

#endif /* __STB_HW_IF_H__ */
