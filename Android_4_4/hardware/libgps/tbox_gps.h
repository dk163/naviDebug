#ifndef __TBOX_GPS_H__
#define __TBOX_GPS_H__

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

#include <cutils/log.h>
#include <cutils/sockets.h>
#include <cutils/properties.h>
#include <hardware/gps.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define  LOG_TAG  "tbox_gps"

#ifndef TBOX_UDP_SERVER_PORT
#define TBOX_UDP_SERVER_PORT	8001
#endif

/* Nmea Parser stuff */
#define  GPS_NMEA_MAX_SIZE  83

#define GPS_MSG_FIELD_SIZE (32)	  /* max msg field size */
#define GPS_MSG_RXBUF_SIZE (2048) /* max msg buffer size */ 

#define GPS_MSG_TBOX_BUF_SIZE (1024) /* max tbox msg buffer size */ 

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
    GpsSvStatus  sv_status;
    int     sv_status_changed;
    char    in[ GPS_NMEA_MAX_SIZE+1 ];
    int     gsa_fixed;
    AthTimemap_t timemap;
} NmeaReader;

typedef struct {
    int                     init;
    int                     fd;
	int 					test_count;
    GpsCallbacks            callbacks;
    pthread_t		    	nmea_thread;
    int                     fix_freq;
    int                     first_fix;
    NmeaReader              reader;
	int 					one_frame_count;
	char 					one_frame_nmea[GPS_MSG_FIELD_SIZE][GPS_NMEA_MAX_SIZE];
	int 					raw_used;
	char 					rawbuf[GPS_MSG_RXBUF_SIZE];
	char 					tboxbuf[GPS_MSG_TBOX_BUF_SIZE];
} GpsState;


#endif /* __TBOX_GPS_H__ */
