#ifndef __GPS_THREAD_H__
#define __GPS_THREAD_H__

#include <pthread.h>
#include <hardware/gps.h>

typedef struct {
    bool                    init;
    pthread_t               thread;
    int                     ctl_fd[2];
    GpsStatus               status;
} GpsState;

// device control
int gps_state_init();
int gps_state_start();
int gps_state_stop();
void gps_state_done();

// main thread
void aip_thread(void *pThreadData);

// gps api
int gps_inject_time(GpsUtcTime timeUtcGps, int64_t timeReference, int uncertainty);
int gps_inject_location(double latitude, double longitude, float accuracy);
void gps_delete_aiding_data(GpsAidingData flags);

int gps_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence, 
        uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time);


#endif /* __GPS_THREAD_H__ */
