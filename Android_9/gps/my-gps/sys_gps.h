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


/****************************************************
*	First to change "Baudrate"、"Uart-Port Path"	*
****************************************************/
//
// GPS driver version
//	
#define	SYS_GNSS_DRIVER_LIB_VERSION	"SYS_GNSS_LIB_V0.0.1" 
//
// *Change Baudrate for your GPS device :
//
//	GPS Baudrate = 9600		==>		GPS_DEVICE_BAUDRATE = B9600
// 	GPS Baudrate = 19200	==>		GPS_DEVICE_BAUDRATE = B19200
//	GPS Baudrate = 38400	==>		GPS_DEVICE_BAUDRATE = B38400
//	GPS Baudrate = 57600	==>		GPS_DEVICE_BAUDRATE = B57600
//	GPS Baudrate = 115200	==>		GPS_DEVICE_BAUDRATE = B115200
//
#define GPS_DEVICE_BAUDRATE		B115200
//
// This "path" for your UART port number ==> mapping to "ttymxcX" ( X maybe is 1、2、3 ... )。
//
#define GPS_DEVICE_PATH		"/dev/ttyUSB1"

#define		NMEA_MAX_SIZE  	168

#define		CMD_MAX_LEN		166

//#define  GPS_DEBUG //debug flag
#undef	 GPS_DEBUG_TOKEN	

#define  AIP_LOGI(...)   ALOGI(__VA_ARGS__)
#define  AIP_LOGE(...)   ALOGE(__VA_ARGS__)

#ifdef GPS_DEBUG
//#  define  D(...)   ALOGD(__VA_ARGS__)
//#define D(format,...) ALOGD(__FILE__"(%s:%d)"format"", __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define D(format,...) ALOGD("(%s:%d) "format" ", __FUNCTION__, __LINE__, ##__VA_ARGS__) 

#define D(format,...) ALOGD("(%s:%d)"format"", __FUNCTION__, __LINE__, ##__VA_ARGS__) 

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
//    pthread_t		    nmea_thread;//kang del
    int                     control[2];
    int                     fix_freq;
    sem_t                   fix_sem;
    int                     first_fix;
    NmeaReader              reader;
} GpsState;


static GpsCallbacks* g_gpscallback = 0;

static int gps_opentty(GpsState *state);
static void	gps_closetty(GpsState *state);
static void	gps_wakeup(GpsState *state);
static void	gps_sleep(GpsState *state);
static int gps_checkstate(GpsState *state);
static GpsState  _gps_state[1];
static GpsState *gps_state = _gps_state;
static int lastcmd = 0;
static int started    = 0;
static const char prop[] = GPS_DEVICE_PATH;

static unsigned char isValid_nmea_cmd_checksum( const char *recv_cmd );

static char buff[800];

static int16_t gnss_GPS_satellite_used_cnt = 0;
static int16_t gnss_Other_satellite_used_cnt = 0;
static int16_t gnss_GPS_used_prn_buf[GPS_MAX_SVS];
static int16_t gnss_Other_used_prn_buf[GPS_MAX_SVS];


/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   T O K E N I Z E R                     *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

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

#endif /* __SYS_GPS_H__ */
