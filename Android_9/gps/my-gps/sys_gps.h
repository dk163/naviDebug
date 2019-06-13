/******************************************************************************

  Copyright (C), 2019-2029, DCN Co., Ltd.

 ******************************************************************************
  File Name     : sys_gps.h
  Version       : Initial Draft
  Author        : Kang
  Created       : 2019/6/13
  Last Modified :
  Description   : sys_gps.c header file
  Function List :
  History       :
  1.Date        : 2019/6/1
    Author      : Kang
    Modification: Created file

******************************************************************************/


#ifndef __SYS_GPS_H__
#define __SYS_GPS_H__

#include <errno.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <math.h>
#include <time.h>
#include <semaphore.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <sys/inotify.h>
#include <poll.h>

#define  LOG_TAG   "sys_gnss"

#include <cutils/log.h>
#include <cutils/sockets.h>
#include <cutils/properties.h>
#include <hardware/gps.h>


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

// GPS driver version	
#define	SYS_GNSS_DRIVER_LIB_VERSION	"SYS_GNSS_LIB_V0.0.1"

//	GPS Baudrate = 115200	==>		GPS_DEVICE_BAUDRATE = B115200
#define GPS_DEVICE_BAUDRATE		B115200

// This "path" for your UART port
#define GPS_DEVICE_PATH		"/dev/ttyUSB1"

#define	 NMEA_MAX_SIZE 168

#define	 CMD_MAX_LEN 166

//#define  GPS_DEBUG //debug flag

#define  AIP_LOGI(...)   ALOGI(__VA_ARGS__)
#define  AIP_LOGE(...)   ALOGE(__VA_ARGS__)

#ifdef GPS_DEBUG
#define D(format,...) ALOGD("(%s:%d) "format" ", __FUNCTION__, __LINE__, ##__VA_ARGS__) 
#else
#  define  D(...)   ((void)0)
#endif


#define GPS_STATUS_CB(_cb, _s)    \
  if ((_cb).status_cb) {          \
    GpsStatus gps_status;         \
    gps_status.status = (_s);     \
    (_cb).status_cb(&gps_status); \
    AIP_LOGI("gps status callback: 0x%x", _s); \
  }


enum {
  STATE_QUIT  = 0,
  STATE_INIT  = 1,
  STATE_START = 2
};

typedef struct
{
    int valid;
    double systime;
    GpsUtcTime timestamp;
} AthTimemap_t;

typedef struct {
    int     pos;
    int     overflow;
    int     utc_year;
    int     utc_mon;
    int     utc_day;
    int     utc_diff;
    GpsLocation  fix;	
	GnssSvStatus gnss_sv_info;
    int     sv_status_changed;
    char    in[ NMEA_MAX_SIZE+1 ];
    int     gsa_fixed;
    AthTimemap_t timemap;
} NmeaReader;

typedef struct {
    int                     init;
    int                     fd;
    GpsCallbacks            callbacks;
    pthread_t               thread;
    int                     control[2];
    int                     fix_freq;
    sem_t                   fix_sem;
    int                     first_fix;
    NmeaReader              reader;
} GpsState;


static GpsCallbacks* g_gpscallback = 0;
static GpsState  _gps_state[1];
static GpsState *gps_state = _gps_state;
static int lastcmd = 0;
static int started    = 0;
static const char prop[] = GPS_DEVICE_PATH;

static char buff[800];

static int16_t gnss_GPS_satellite_used_cnt = 0;
static int16_t gnss_Other_satellite_used_cnt = 0;
static int16_t gnss_GPS_used_prn_buf[GPS_MAX_SVS];
static int16_t gnss_Other_used_prn_buf[GPS_MAX_SVS];


//NMEA TOKEN
typedef struct {
    const char*  p;
    const char*  end;
} Token;

#define MAX_NMEA_TOKENS     32

typedef struct {
    int     count;
    Token   tokens[ MAX_NMEA_TOKENS ];
} NmeaTokenizer;


static int epoll_ctrlfd;
static int gps_fd;
static int control_fd ;
static NmeaReader  *reader;
static int nn;
static int ret;

//function definition
static int close_gps(struct hw_device_t *device);
static double convert_from_hhmm( Token  tok );
static int epoll_deregister( int  epoll_fd, int  fd );
static int epoll_register( int  epoll_fd, int  fd );
static int gps_checkstate(GpsState *s);
static void gps_closetty(GpsState *s);
static unsigned char gps_dev_calc_nmea_csum(char *msg);
static void gps_dev_send(int fd, char *msg);
static int gps_opentty(GpsState *state);
static void gps_sleep(GpsState *state);
static void gps_state_done( GpsState*  s );
static void gps_state_init( GpsState*  state );
static void gps_state_lock_fix(GpsState *state);
static void gps_state_start( GpsState*  s );
static void gps_state_stop( GpsState*  s );
static void gps_state_thread( void*  arg );
static void gps_state_unlock_fix(GpsState *state);
static void gps_state_update_fix_freq(GpsState *s, int fix_freq);
static void gps_wakeup(GpsState *state);
static const GpsInterface* gps__get_gps_interface(struct gps_device_t* dev);
static unsigned char isValid_nmea_cmd_checksum( const char *recv_cmd );
static void nmea_reader_addc( NmeaReader* r, int  c );
static int nmea_reader_get_timestamp(NmeaReader*  r, Token  tok, time_t *timestamp);
static void nmea_reader_init( NmeaReader*  r );
static void nmea_reader_parse( NmeaReader* r );
static int nmea_reader_update_accuracy( NmeaReader* r, Token accuracy );
static int nmea_reader_update_altitude( NmeaReader*  r,
                                      Token        altitude,
                                      Token        units );
static int nmea_reader_update_bearing( NmeaReader* r, Token bearing );
static int nmea_reader_update_cdate( NmeaReader*  r, Token  tok_d, Token tok_m, Token tok_y );
static int nmea_reader_update_date( NmeaReader*  r, Token  date, Token  mtime );
static int nmea_reader_update_latlong( NmeaReader*  r,
                                     Token        latitude,
                                     char         latitudeHemi,
                                     Token        longitude,
                                     char         longitudeHemi );
static int nmea_reader_update_speed( NmeaReader* r, Token speed );
static int nmea_reader_update_time( NmeaReader*  r, Token  tok );
static int nmea_reader_update_timemap( NmeaReader* r, Token systime_tok, Token timestamp_tok);
static void nmea_reader_update_utc_diff( NmeaReader*  r );
static Token nmea_tokenizer_get( NmeaTokenizer*  t, int  index );
static int nmea_tokenizer_init( NmeaTokenizer*  t, const char*  p, const char*  end );
static int open_gps(const struct hw_module_t* module, char const* name, struct hw_device_t** device);
static double str2float( const char*  p, const char*  end );
static int str2int( const char*  p, const char*  end );
static void sys_gps_cleanup(void);
static void sys_gps_delete_aiding_data(GpsAidingData flags);
static const void* sys_gps_get_extension(const char* name);
static int sys_gps_init(GpsCallbacks* callbacks);
static int sys_gps_inject_location(double latitude, double longitude, float accuracy);
static int sys_gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty);
static void sys_gps_set_fix_frequency(int freq);
static int sys_gps_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
         									  uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time);
static int sys_gps_start();
static int sys_gps_stop();

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __SYS_GPS_H__ */

