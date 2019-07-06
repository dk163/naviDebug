/*
* Copyright (C) 2011 The Android Open Source Project
* Copyright (C) 2011-2013 Freescale Semiconductor, Inc.
* Copyright (C) 2014 Unicore Communications, Inc.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
#include <errno.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <math.h>
#include <time.h>
#include <semaphore.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <sys/inotify.h>
#include <poll.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#define  LOG_TAG  "bd_gps"

#include <log/log.h>
#include <cutils/sockets.h>
#include <cutils/properties.h>
#include <hardware/gps.h>

/* for UM220 RNSS devices*/
#define UM220_GPS
#undef  UM220_GPSXtra
#undef  UM220_GPSNi

//#define GPS_DEVICE	"/dev/ttyS0"
#define GPS_DEVICE	 "/dev/ttyLP2"
static int current_bps = 9600;

// Dynamic log level verbosity
#define LOG_VERBOSE	5
#define LOG_DEBUG	4
#define LOG_INFO	3
#define LOG_ERROR	2
#define LOG_WARN	1
#define LOG_NONE	0
static int log_level = LOG_VERBOSE;
static int initdone = 0;
static int glo_or_bds = -1;

static const int cfg_inproto = 0x83;	// RTCM3.2 | NMEA | UNICORE
static const int cfg_outproto = 0x03;	// DEBUG | NMEA | UNICORE

//#define  VERBOSE
#define  GPS_DEBUG //debug
//#define  CONCAT_NMEA
//#define ENABLE_UNDULATION
#undef	 GPS_DEBUG_TOKEN	/* print out NMEA tokens */

#define  DFR(...)   ALOGD(__VA_ARGS__)

#ifdef GPS_DEBUG
#  define  D(...)   ALOGD(__VA_ARGS__)
#else
#  define  D(...)   ((void)0)
#endif

#define GPS_STATUS_CB(_cb, _s)    \
  if ((_cb).status_cb) {          \
    GpsStatus gps_status;         \
    gps_status.status = (_s);     \
    (_cb).status_cb(&gps_status); \
    DFR("gps status callback: 0x%x", _s); \
  }

/* Nmea Parser stuff */
#define  NMEA_MAX_SIZE  256
#define	MAX_SVS	256

enum {
	STATE_QUIT  = 0,
	STATE_INIT  = 1,
	STATE_START = 2
};

/*
    GNSS_CONSTELLATION_GPS = 1,
    GNSS_CONSTELLATION_SBAS = 2,
    GNSS_CONSTELLATION_GLONASS = 3,
    GNSS_CONSTELLATION_QZSS = 4,
    GNSS_CONSTELLATION_BEIDOU = 5,
    GNSS_CONSTELLATION_GALILEO = 6,
*/
enum {
	RNSS_GPS = GNSS_CONSTELLATION_GPS,
	RNSS_BDS = GNSS_CONSTELLATION_BEIDOU, 
	RNSS_GLO = GNSS_CONSTELLATION_GLONASS, 
};

typedef struct
{
	int valid;
	double systime;
	GpsUtcTime timestamp;
} UmTimemap_t;

typedef struct {
	int     pos;
	int     overflow;
	int     utc_year;
	int     utc_mon;
	int     utc_day;
	int     utc_diff;
	GpsLocation  fix;
	GnssSvInfo sv_list[MAX_SVS];
	GnssSvInfo sv_list_tmp[MAX_SVS];
	int     sv_status_changed;
	int     sv_status_commit;
	char    in[NMEA_MAX_SIZE+1];
	int     gsa_fixed;
	UmTimemap_t timemap;
	int     nmea_mode;
} NmeaReader;

typedef struct {
	int                     init;
	int                     fd;
	GpsCallbacks            callbacks;
	pthread_t               thread;
	pthread_t               nmea_thread;
	pthread_t               tmr_thread;
	int                     control[2];
	int                     bps;
	int                     fix_freq;
	int                     fix_freq_pending;
	sem_t                   fix_sem;
	int                     first_fix;
	NmeaReader              reader;
#ifdef UM220_GPSXtra
	int			xtra_init;
	GpsXtraCallbacks	xtra_callbacks;
#endif
#ifdef UM220_GPSNi
	int                     ni_init;
	GpsNiCallbacks          ni_callbacks;
	GpsNiNotification       ni_notification;
#endif
	int			rnss_mode;
	struct timespec		ts_last_report;
	struct timespec		ts_last_active;
} GpsState;


GpsCallbacks* g_gpscallback = 0;

void gps_state_lock_fix(GpsState *state) {
	int ret;
	do {
		ret=sem_wait(&state->fix_sem);
	} while (ret < 0 && errno == EINTR);
	if (ret < 0) {
		D("Error in GPS state lock:%s\n", strerror(errno));
	}
}

void gps_state_unlock_fix(GpsState *state) {
	if (sem_post(&state->fix_sem) == -1)
	{
		if(errno == EAGAIN)
			if(sem_post(&state->fix_sem)== -1)
				D("Error in GPS state unlock:%s\n", strerror(errno));
	}
}

int 	gps_opentty(GpsState *state);
void 	gps_closetty(GpsState *state);
void 	gps_wakeup(GpsState *state);
void 	gps_sleep(GpsState *state);
int 	gps_checkstate(GpsState *state);

static GpsState  _gps_state[1];
static GpsState *gps_state = _gps_state;
static int sleep_lock = 0;
static int lastcmd = 0;
static int bGetFormalNMEA = 1;
static int started    = 0;
static int continue_thread = 1;
static int bOrionShutdown = 0;

static char isNmeaThreadExited = 0;
static char isTimerThreadExited = 0;
static char isStateThreadExited = 0;

static void gps_dev_send(int fd, char *msg);
static unsigned char bd_check_eor(char *buf, int len);
static void gps_nmea_thread( void*  arg );
static void gps_timer_thread( void*  arg );
static void set_rnss_mode(int fd, char mode);
static void set_fix_freq(int fd, int freq);
static void check_rnss_mode(NmeaReader* r);
static void init_serial(int fd, int bps);
static void set_baud_rate(int fd, int bps);
static void init_cmds(GpsState *state);
static unsigned int time_diff(struct timespec *tv1, struct timespec *tv2);
extern double get_geoid(double lat, double lon);

int is_valid_fd(int fd)
{
	return fcntl(fd, F_GETFD) != -1 || errno != EBADF;
}
/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****                 U T I L I T I E S                     *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

#ifdef UM220_GPSNi
static void um_send_ni_notification(NmeaReader* r)
{
	//D("um_send_ni_notification is called");
	if (gps_state->ni_init && gps_state->ni_callbacks.notify_cb)
	{
		memset(&gps_state->ni_notification, 0, sizeof(sizeof(gps_state->ni_notification)));
		gps_state->ni_notification.size                     = sizeof(GpsNiNotification);
		gps_state->ni_notification.notification_id          = 0xABBA;
		gps_state->ni_notification.ni_type                  = GPS_NI_TYPE_UMTS_CTRL_PLANE; // AM TBD: is it correct?
		gps_state->ni_notification.notify_flags             = 0; // no notification & no verification is needed
		gps_state->ni_notification.timeout                  = 0; // no timeout limit
		gps_state->ni_notification.default_response         = GPS_NI_RESPONSE_NORESP;
		gps_state->ni_notification.requestor_id[0]          = 0;
		gps_state->ni_notification.text[0]                  = 0;
		gps_state->ni_notification.requestor_id_encoding    = GPS_ENC_NONE;
		gps_state->ni_notification.text_encoding            = GPS_ENC_NONE;
		if (r->timemap.valid && r->timemap.timestamp == r->fix.timestamp)
		{
			sprintf(gps_state->ni_notification.extras, "pps=%10.10lf", r->timemap.systime);
			r->timemap.valid = 0;
		}
		else
		{
			gps_state->ni_notification.extras[0] = 0;
		}

		gps_state->ni_callbacks.notify_cb(&gps_state->ni_notification);
	}
}
#else // #ifdef UM220_GPSNi
#   define um_send_ni_notification(r)
#endif // #ifdef UM220_GPSNi

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

#define  MAX_NMEA_TOKENS  32

typedef struct {
	int     count;
	Token   tokens[ MAX_NMEA_TOKENS ];
} NmeaTokenizer;

static int nmea_tokenizer_init( NmeaTokenizer*  t, const char*  p, const char*  end )
{
	int    count = 0;

	// the initial '$' is optional
	if (p < end && p[0] == '$')
		p += 1;

	// remove trailing newline
	if (end > p && end[-1] == '\n') {
		end -= 1;
		if (end > p && end[-1] == '\r')
			end -= 1;
	}

	// get rid of checksum at the end of the sentecne
	if (end >= p+3 && end[-3] == '*') {
		end -= 3;
	}

	while (p < end) {
		const char*  q = p;

		q = memchr(p, ',', end-p);
		if (q == NULL)
			q = end;

		if (count < MAX_NMEA_TOKENS) {
			t->tokens[count].p   = p;
			t->tokens[count].end = q;
			count += 1;
		}

		if (q < end)
			q += 1;

		p = q;
	}

	// $BDGGA,093256.122,3037.511084,N,10402.236998,E,1,05,8.697,483.933,M,0,M,,*6C
	//                                                                         ^^
	// last field empty?
	if(end[-1] == ',') {
		t->tokens[count].p   = end;
		t->tokens[count].end = end;
		count += 1;
	}

	t->count = count;
	return count;
}

static Token nmea_tokenizer_get( NmeaTokenizer*  t, int  index )
{
	Token  tok;
	static const char*  dummy = "";

	if (index < 0 || index >= t->count) {
		tok.p = tok.end = dummy;
	} else
		tok = t->tokens[index];

	return tok;
}


static int str2int( const char*  p, const char*  end )
{
	int   result = 0;
	int   len    = end - p;

	if (len == 0) {
		return -1;
	}

	for ( ; len > 0; len--, p++ )
	{
		int  c;

		if (p >= end)
			goto Fail;

		c = *p - '0';
		if ((unsigned)c >= 10)
			goto Fail;

		result = result*10 + c;
	}
	return  result;

Fail:
	return -1;
}

static double str2float( const char*  p, const char*  end )
{
	int   len    = end - p + 1;
	char  temp[32];

	if (len == 0) {
		return -1.0;
	}

	if (len >= (int)sizeof(temp))
		return 0.;

	memcpy( temp, p, len );
	temp[len] = 0;
	return strtod( temp, NULL );
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   P A R S E R                           *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

static void nmea_reader_update_utc_diff( NmeaReader*  r )
{
	time_t         now = time(NULL);
	struct tm      tm_local;
	struct tm      tm_utc;
	long           time_local, time_utc;

	gmtime_r( &now, &tm_utc );
	localtime_r( &now, &tm_local );

	time_local = tm_local.tm_sec +
		60*(tm_local.tm_min +
		60*(tm_local.tm_hour +
		24*(tm_local.tm_yday +
		365*tm_local.tm_year)));

	time_utc = tm_utc.tm_sec +
		60*(tm_utc.tm_min +
		60*(tm_utc.tm_hour +
		24*(tm_utc.tm_yday +
		365*tm_utc.tm_year)));

	r->utc_diff = time_utc - time_local;
}


static void nmea_reader_init( NmeaReader*  r )
{
	memset( r, 0, sizeof(*r) );

	r->pos      = 0;
	r->overflow = 0;
	r->utc_year = -1;
	r->utc_mon  = -1;
	r->utc_day  = -1;

	nmea_reader_update_utc_diff( r );
}

static int nmea_reader_get_timestamp(NmeaReader*  r, Token  tok, time_t *timestamp)
{
	int        hour, minute;
	double     seconds;
	struct tm  tm;
	time_t     ttime;

	if (tok.p + 6 > tok.end)
		return -1;

	if (r->utc_year < 0) {
		return -1;
	}

	hour    = str2int(tok.p,   tok.p+2);
	minute  = str2int(tok.p+2, tok.p+4);
	seconds = str2float(tok.p+4, tok.end);

	tm.tm_hour = hour;
	tm.tm_min  = minute;
	tm.tm_sec   = (int) seconds;
	tm.tm_year = r->utc_year - 1900;
	tm.tm_mon  = r->utc_mon - 1;
	tm.tm_mday = r->utc_day;
	tm.tm_isdst = -1;

	// D("h: %d, m: %d, s: %d", tm.tm_hour, tm.tm_min, tm.tm_sec);
	// D("Y: %d, M: %d, D: %d", tm.tm_year, tm.tm_mon, tm.tm_mday);

	nmea_reader_update_utc_diff(r);

	ttime = mktime( &tm );
	*timestamp = ttime - r->utc_diff;
	//D("*timestamp : %ld ", *timestamp);

	return 0;
}

static int nmea_reader_update_time( NmeaReader*  r, Token  tok )
{
	time_t timestamp = 0;
	int ret = nmea_reader_get_timestamp( r, tok, &timestamp);
	if (0 == ret)
		r->fix.timestamp = (long long)timestamp * 1000;
	return ret;
}

static int nmea_reader_update_cdate( NmeaReader*  r, Token  tok_d, Token tok_m, Token tok_y )
{

	if ( (tok_d.p + 2 > tok_d.end) ||
			(tok_m.p + 2 > tok_m.end) ||
			(tok_y.p + 4 > tok_y.end) )
		return -1;

	r->utc_day = str2int(tok_d.p,   tok_d.p+2);
	r->utc_mon = str2int(tok_m.p, tok_m.p+2);
	r->utc_year = str2int(tok_y.p, tok_y.end+4);

	return 0;
}

static int nmea_reader_update_date( NmeaReader*  r, Token  date, Token  mtime )
{
	Token  tok = date;
	int    day, mon, year;

	if (tok.p + 6 != tok.end) {

		/* no date info, will use host time in _update_time function
		*/
	}
	/* normal case */
	day  = str2int(tok.p, tok.p+2);
	mon  = str2int(tok.p+2, tok.p+4);
	year = str2int(tok.p+4, tok.p+6) + 2000;

	if ((day|mon|year) < 0) {
		return -1;
	}

	r->utc_year  = year;
	r->utc_mon   = mon;
	r->utc_day   = day;

	return nmea_reader_update_time( r, mtime );
}


static double convert_from_hhmm( Token  tok )
{
	double  val     = str2float(tok.p, tok.end);
	int     degrees = (int)(floor(val) / 100);
	double  minutes = val - degrees*100.;
	double  dcoord  = degrees + minutes / 60.0;
	return dcoord;
}


static int nmea_reader_update_latlong( NmeaReader*  r,
		Token        latitude,
		char         latitudeHemi,
		Token        longitude,
		char         longitudeHemi )
{
	double   lat, lon;
	Token    tok;

	tok = latitude;
	if (tok.p + 6 > tok.end) {
		return -1;
	}
	lat = convert_from_hhmm(tok);
	if (latitudeHemi == 'S')
		lat = -lat;

	tok = longitude;
	if (tok.p + 6 > tok.end) {
		return -1;
	}
	lon = convert_from_hhmm(tok);
	if (longitudeHemi == 'W')
		lon = -lon;

	r->fix.latitude  = lat;
	r->fix.longitude = lon;
	r->fix.flags    |= GPS_LOCATION_HAS_LAT_LONG;
	return 0;
}


static int nmea_reader_update_altitude( NmeaReader*  r,
		Token        altitude,
		Token        units )
{
	Token   tok = altitude;

	(void)units;

	if (tok.p >= tok.end)
		return -1;

	r->fix.altitude = str2float(tok.p, tok.end);
	r->fix.flags   |= GPS_LOCATION_HAS_ALTITUDE;
	return 0;
}

static int nmea_reader_update_accuracy( NmeaReader*  r,
		Token        accuracy )
{
	Token   tok = accuracy;

	if (tok.p >= tok.end)
		return -1;

	//tok is cep*cc, we only want cep
	r->fix.accuracy = str2float(tok.p, tok.end);

	if (r->fix.accuracy == 99.99){
		return 0;
	}

	r->fix.flags   |= GPS_LOCATION_HAS_ACCURACY;
	return 0;
}

static int nmea_reader_update_bearing( NmeaReader*  r,
		Token        bearing )
{
	Token   tok = bearing;

	if (tok.p >= tok.end)
		return -1;

	r->fix.bearing  = str2float(tok.p, tok.end);
	r->fix.flags   |= GPS_LOCATION_HAS_BEARING;
	return 0;
}


static int nmea_reader_update_speed( NmeaReader*  r,
		Token        speed )
{
	Token   tok = speed;

	if (tok.p >= tok.end)
		return -1;

	r->fix.speed    = str2float(tok.p, tok.end);
	r->fix.speed   *= 0.514444;    // fix for Speed Unit form Knots to Meters per Second
	r->fix.flags   |= GPS_LOCATION_HAS_SPEED;
	return 0;
}

/*
static int nmea_reader_update_timemap( NmeaReader* r,
		Token       systime_tok,
		Token       timestamp_tok)
{
	int ret;
	time_t timestamp;

	if ( systime_tok.p >= systime_tok.end ||
			timestamp_tok.p >= timestamp_tok.end)
	{
		r->timemap.valid = 0;
		return -1;
	}

	ret = nmea_reader_get_timestamp(r, timestamp_tok, &timestamp);
	if (ret)
	{
		r->timemap.valid = 0;
		return ret;
	}

	r->timemap.valid = 1;
	r->timemap.systime = str2float(systime_tok.p, systime_tok.end);
	r->timemap.timestamp = (GpsUtcTime)((long long)timestamp * 1000);
	return 0;
}
*/

/* commands sent to the gps thread */
enum {
	CMD_QUIT  = 0,
	CMD_START = 1,
	CMD_STOP  = 2,
	CMD_SET_FREQ = 3,
	CMD_SET_BAUD = 4,
	CMD_MODE_GPS = 5,
	CMD_MODE_BD = 6,
	CMD_MODE_GN = 7,
};

int valid_modes[] = {
	CMD_MODE_GN,
	CMD_MODE_GPS,
	CMD_MODE_BD,
	CMD_MODE_GN,
};

#ifdef	CONCAT_NMEA
int report_nmea_concat(unsigned long long ms, char *buf, int len)
{
	static char lastbuf[2048];
	static char *lastp;
	unsigned long long lastms;
	char *p = buf + len - 1;

	// init
	if(!lastp) lastp = lastbuf;

	// $GNRMC, $GPRMC, $BDRMC, $GBRMC
	if(strncmp(buf+3, "RMC", 3) == 0) {
		lastms = ms;

		if(lastp != lastbuf) {
			// append <CR><LF> if has data
			*lastp++ = 0x0d; *lastp++ = 0x0a;
			gps_state->callbacks.nmea_cb(lastms, lastbuf, lastp - lastbuf);
			D("CONCAT_NMEA: %.*s", lastp-lastbuf, lastbuf);
			lastp = lastbuf;
		}
	}

	// remove <CR><LF> and save NMEA
	if(*p == 0x0d || *p == 0x0a) p--;
	if(*p == 0x0d || *p == 0x0a) p--;
	len = p - buf;

	lastp += sprintf(lastp, "%.*s|", len, buf);

	return 0;
}
#endif

char *trim(char *buf, int *len)
{
	char *bufend = buf + *len;

	while(bufend > buf && (isspace(bufend[-1]) || iscntrl(bufend[-1])))
		bufend--;

	while(buf < bufend && (isspace(*buf) || iscntrl(*buf)))
		buf++;

	*bufend = 0;

	*len = bufend - buf;

	return buf;
}

/*
 * $GPGGA,133456,0000.348,S,00000.348,E,2,07,1.2,10.7,M,48.8,M,7,0001*5A
 *  |                                                               |
 *  `---------------------------------------------------------------`-- xor value
 */
#define NMEA_MINLEN	9	// $PPSSS*XX
int nmea_verify_packet(char *buf, int len)
{
	char sum = 0, crc;
	char *bufend = buf + len;

	if(len < NMEA_MINLEN) return -1;

	if(buf[0] != '!' && buf[0] != '$' && bufend[-3] != '*')
		return -1;

	buf++;
	len -= 4;   /* 3 + 1 */

	while(len--)
		sum ^= buf[len];

	crc = (char)strtoul(&bufend[-2], NULL, 16);

	return sum != crc;
}

#define min(x,y)	((x) < (y) ? (x) : (y))
#define DUMP_SIZE	1024
char debugbuf[2][DUMP_SIZE];
int debuglen[2];
char dumpbuf[DUMP_SIZE*3+8];
void dump(char *s, int len)
{
	char *buf = dumpbuf, *p, *end, *bufend;

	p = buf;
	bufend = buf + DUMP_SIZE*3;
	end = s+len;
	while(s < end && p < bufend) {
		if(isprint(*s))
			*p++ = *s;
		else
			p += sprintf(p, "\\%02X", *(unsigned char *)s);
		s++;
	}

	*p++ = 0;
	
	D("DUMP(len=%d): [%s]", len, buf);
}

char *pre_nmea(char *pbuf, int *olen)
{
	NmeaTokenizer  tzer[1];
	char ibuf[NMEA_MAX_SIZE+1];
	static char obuf[NMEA_MAX_SIZE+1];
	int len = *olen;

	// hook with GGA for geoid
	if(pbuf[0] != '$' || strncmp(pbuf+3,"GGA",3) != 0)
		return pbuf;

	memcpy(ibuf, pbuf, len);
	nmea_tokenizer_init(tzer, ibuf, ibuf + len);

/*  0     1          2           3 4            5 6 7  8     9       A B CDE
 * $BDGGA,093256.122,3037.511084,N,10402.236998,E,1,05,8.697,483.933,M,0,M,,*6C
 *                                                           |         |
 *                                                           \         `-- abnormal height
 *                                                            `-- MSL height
 */
	Token  tok_fixstaus      = nmea_tokenizer_get(tzer,6);
	if (tok_fixstaus.p[0] >= '0') {

		Token  tok_latitude      = nmea_tokenizer_get(tzer,2);
		Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,3);
		Token  tok_longitude     = nmea_tokenizer_get(tzer,4);
		Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,5);
		Token  tok_altitude      = nmea_tokenizer_get(tzer,9);
//		Token  tok_altitudeUnits = nmea_tokenizer_get(tzer,10);

		{
			double lat, lon, alt;
			double h, N;
			char *p;
			int n, ndig = 0;
			unsigned char sum;


			// calculate significant digits
			p = (char *)tok_altitude.p;
			while(*p != '.' && p < tok_altitude.end) p++;
			if(p < tok_altitude.end) {
				p++;
				ndig = tok_altitude.end - p;
			}

			lat = convert_from_hhmm(tok_latitude);
			if (tok_latitudeHemi.p[0] == 'S') lat = -lat;

			lon = convert_from_hhmm(tok_longitude);
			if (tok_longitudeHemi.p[0] == 'W') lon = -lon;

			alt = str2float(tok_altitude.p, tok_altitude.end);

			// h(elevation above elipsoid) = H(msl) + N(geoid)
			N = get_geoid(lat, lon);
			h = alt;
			p = obuf;
//			D("[tzer->count = %d] N=%.3f h=%.3f lat=%.6f lon=%.6f", tzer->count, N, h, lat, lon);
			for (n = 0; n < tzer->count; n++) {
				Token  tok = nmea_tokenizer_get(tzer,n);
//				D("%2d: '%.*s'", n, tok.end-tok.p, tok.p);
				if(n == 9)
					p += sprintf(p, ",%.*f", ndig, h-N);
				else if(n == 11)
					p += sprintf(p, ",%.*f", ndig, N);
				else
					p += sprintf(p, ",%.*s", (int)(tok.end-tok.p), tok.p);
			}
			// skip leading ","
			p = obuf; *p = '$';
			n = strlen(p);
			sum = bd_check_eor(p, n-1);
			sprintf(p + n, "*%02X",sum);
			pbuf = p;
			*olen = strlen(pbuf);
		}
	}

	return pbuf;
}

// $BDGSV,1,1,2,1705,,8,173,5,,27*64
static void nmea_reader_parse( NmeaReader*  r )
{
	/* we received a complete sentence, now parse it to generate
	 * a new GPS fix...
	 */
	NmeaTokenizer  tzer[1];
	Token          tok;
	int rtype;
	int len = r->pos;
	char *pbuf = trim(r->in, &len);

	/**********************************************
	N/A G1B1 COM1
	PN N/A
	SN N/A
	HWVer V0.0
	FWVer R3.1.0Build1979
	Copyright (c), Unicore Communications Inc.
	All rights reserved.
	**********************************************/
	if(strncasecmp(pbuf, "copyright", 8) == 0 && strcasestr(pbuf, "Unicore")) {
		init_cmds(gps_state);
	}

	if (len < NMEA_MINLEN) {
		D("short nmea");
		if(log_level >= LOG_VERBOSE) {
			dump(pbuf, len);
		}
		return;
	}

	// legal check
	if(nmea_verify_packet(pbuf, len) != 0) {
		D("bad nmea");
		if(log_level >= LOG_VERBOSE) {
#ifdef VERBOSE
			if(debuglen[0]) {
				D("DUMP debugbuf[0]");
				dump(debugbuf[0], debuglen[0]);
			}
			if(debuglen[1]) {
				D("DUMP debugbuf[1]");
				dump(debugbuf[1], debuglen[1]);
			}
#endif
			D("DUMP nmeabuf");
			dump(pbuf, len);
		}
		return;
	}

	// allow changes to nmea before continue.
	// returns with new buffer if modified, otherwise old.
#ifdef	ENABLE_UNDULATION
	pbuf = pre_nmea(pbuf, &len);
#endif

	if (gps_state->callbacks.nmea_cb) {
		struct timeval tv;
		unsigned long long mytimems;
		gettimeofday(&tv,NULL);
		mytimems = tv.tv_sec * 1000 + tv.tv_usec / 1000;
#ifdef	CONCAT_NMEA
		report_nmea_concat(mytimems, pbuf, len);
#else
		D("um220_reader_parse(pos=%d): %.*s ", len, len, pbuf);
		// some buggy parser needs <CR><LF> at end
		//	*p++ = 0x0d; *p++ = 0x0a; r->pos += 2;
		gps_state->callbacks.nmea_cb(mytimems, pbuf, len);
#endif
	}

	nmea_tokenizer_init(tzer, pbuf, pbuf + len);
#ifdef GPS_DEBUG_TOKEN
	{
		int  n;
		D("Found %d tokens", tzer->count);
		for (n = 0; n < tzer->count; n++) {
			Token  tok = nmea_tokenizer_get(tzer,n);
			D("%2d: '%.*s'", n, tok.end-tok.p, tok.p);
		}
	}
#endif

	tok = nmea_tokenizer_get(tzer, 0);

	if (tok.p + 5 > tok.end) {
		/* short sentences */
		return;
	}

	clock_gettime(CLOCK_MONOTONIC, &gps_state->ts_last_active);

	//--GSA,--GGA,--GSV
	if ( !memcmp(tok.p, "GPG", 3) || !memcmp(tok.p, "GNG", 3) || !memcmp(tok.p, "BDG", 3) || !memcmp(tok.p, "GBG", 3))
		bGetFormalNMEA = 1;

	// check for RNSS type
	if(!memcmp(tok.p, "BD", 2) || !memcmp(tok.p, "GB", 2)) {
		rtype = RNSS_BDS;
		glo_or_bds = rtype;
	} else if(!memcmp(tok.p, "GL", 2)) {
		rtype = RNSS_GLO;
		glo_or_bds = rtype;
	} else
		rtype = RNSS_GPS;
	// ignore first two characters.
	tok.p += 2;

	if ( !memcmp(tok.p, "GGA", 3) ) {
		// GPS fix
		Token  tok_fixstaus      = nmea_tokenizer_get(tzer,6);

		if (tok_fixstaus.p[0] > '0') {

			Token  tok_time          = nmea_tokenizer_get(tzer,1);
			Token  tok_latitude      = nmea_tokenizer_get(tzer,2);
			Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,3);
			Token  tok_longitude     = nmea_tokenizer_get(tzer,4);
			Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,5);
			Token  tok_altitude      = nmea_tokenizer_get(tzer,9);
			Token  tok_altitudeUnits = nmea_tokenizer_get(tzer,10);

			nmea_reader_update_time(r, tok_time);
			nmea_reader_update_latlong(r, tok_latitude,
					tok_latitudeHemi.p[0],
					tok_longitude,
					tok_longitudeHemi.p[0]);
			nmea_reader_update_altitude(r, tok_altitude, tok_altitudeUnits);
		}

	} else if ( !memcmp(tok.p, "GLL", 3) ) {

		Token  tok_fixstaus      = nmea_tokenizer_get(tzer,6);

		if (tok_fixstaus.p[0] == 'A') {

			Token  tok_latitude      = nmea_tokenizer_get(tzer,1);
			Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,2);
			Token  tok_longitude     = nmea_tokenizer_get(tzer,3);
			Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,4);
			Token  tok_time          = nmea_tokenizer_get(tzer,5);

			nmea_reader_update_time(r, tok_time);
			nmea_reader_update_latlong(r, tok_latitude,
					tok_latitudeHemi.p[0],
					tok_longitude,
					tok_longitudeHemi.p[0]);
		}
	} else if ( !memcmp(tok.p, "GSA", 3) ) {

		Token  tok_fixStatus   = nmea_tokenizer_get(tzer, 2);
		int i;

		if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != '1') {

			Token tok_accuracy      = nmea_tokenizer_get(tzer, 15);
			nmea_reader_update_accuracy(r, tok_accuracy);
			
			//
			// h30/h40/h41 GPS or BD only:
			//      $GPGSA,A,3,2,5,6,7,9,13,20,30,25,29,15,,1.445,0.924,1.111*0D
			// h40 hybrid:
			//      $GNGSA,A,3,166,168,169,171,173,174,161,162,163,164,165,167,0.688,0.837,0.554*24
			// h41 hybrid:
			//      $GNGSA,A,3,56,58,59,61,63,64,51,52,53,54,55,57,1.084,0.689,0.837*2E
			// h51 Hybrid:
			//      $GNGSA,A,3,27,01,07,08,09,11,16,10,30,28,,,1.12,0.72,0.86,1*05  ==> sysid=1, GPS
			//      $GNGSA,A,3,01,02,03,04,05,07,08,10,13,,,,1.12,0.72,0.86,4*00    ==> sysid=4, BDS
			//      $GNGSA,A,3,68,74,66,73,80,83,81,67,,,,,1.18,0.65,0.98,2*0A      ==> sysid=2, GLO
			// h51 GPS or BD only:
			//      $GPGSA,A,3,02,03,06,09,12,17,19,23,28,25,,,1.34,0.85,1.04,1*1E
			//      $GBGSA,A,3,02,03,06,09,12,17,19,23,28,25,,,1.34,0.85,1.04,1*1E

			Token tok_systemID = nmea_tokenizer_get(tzer, tzer->count-1);
			if(tzer->count == 19 && tok_systemID.p != tok_systemID.end && tok_systemID.p[1] == '*') {
				switch(tok_systemID.p[0]) {
					case '2':
						rtype = RNSS_GLO;
						break;
					case '4':
						rtype = RNSS_BDS;
						break;
					case '1':
					default:
						rtype = RNSS_GPS;
				}
			}

			for (i = 3; i <= 14; ++i) {

				Token  tok_prn  = nmea_tokenizer_get(tzer, i);
				int prn = str2int(tok_prn.p, tok_prn.end);

				/* available for PRN 1-255 */
				// TODO
				// check PRN for BD2 satellites not starting from 160, or greater than 192
				// check PRN for GLONASS or GALILEO satellites PRN numbering
				if(prn > 0 && prn < 256) {
					// TODO
					// Protocol violation. H41 not supported.(BD2 PRNs will overlap with GLO)
					// h41 hybrid will use GN and has prn between 51~87
					// BD2: 1~37 or 51~87 or 161~197
					// GLO: 1~24 or 65~96
					if(glo_or_bds == RNSS_BDS && 50 < prn && prn < 88) {
						rtype = RNSS_BDS;
						prn -= 50;
						prn += 160;
					}

					// if not identified, default to glonass
					if(rtype != RNSS_GPS && 64 < prn && prn < 96) {
						rtype = RNSS_GLO;
						prn -= 64;
					}

					if(rtype == RNSS_GLO && prn < 32) {
						prn += 64;
					}
//					D("glo_or_bds %d, rtype %d, prn %d", glo_or_bds, rtype, prn);

					r->sv_list_tmp[prn].flags |= GNSS_SV_FLAGS_USED_IN_FIX;
					r->sv_list_tmp[prn].svid = prn;

					/* mark this parameter to identify the GSA is in fixed state */
					r->gsa_fixed = 1;
				}
			} // end for(;;)

		} else {
			if (r->gsa_fixed == 1) {
				for(i = 0; i < MAX_SVS; i++) {
					r->sv_list[i].flags &= ~GNSS_SV_FLAGS_USED_IN_FIX;
					r->sv_list_tmp[i].flags &= ~GNSS_SV_FLAGS_USED_IN_FIX;
				}

				r->gsa_fixed = 0;
				r->sv_status_commit = 1;
			}
		}
	} else if ( !memcmp(tok.p, "GSV", 3) ) {

		Token  tok_noSatellites  = nmea_tokenizer_get(tzer, 3);
		int    noSatellites = str2int(tok_noSatellites.p, tok_noSatellites.end);

		Token  tok_noSentences   = nmea_tokenizer_get(tzer, 1);
		Token  tok_sentence      = nmea_tokenizer_get(tzer, 2);

		if (noSatellites > 0) {

			int sentence = str2int(tok_sentence.p, tok_sentence.end);
			int totalSentences = str2int(tok_noSentences.p, tok_noSentences.end);
			static int num_svs;
			int i;
			int prn;

			static int ff = 0;	// first found
			if (sentence == 1) {
				for(i = 0; i < MAX_SVS; i++) {
					if(r->sv_list_tmp[i].constellation == rtype) r->sv_list_tmp[i].svid = 0;
				}
				num_svs = 0;
				ff = 1;
			} else {
				if(!ff) {
					D("partial GSV, skip.");
					return;
				}
			}

			i = 0;

			while (i < 4 && num_svs < noSatellites) {


				Token  tok_prn = nmea_tokenizer_get(tzer, i * 4 + 4);
				Token  tok_elevation = nmea_tokenizer_get(tzer, i * 4 + 5);
				Token  tok_azimuth = nmea_tokenizer_get(tzer, i * 4 + 6);
				Token  tok_snr = nmea_tokenizer_get(tzer, i * 4 + 7);

				prn = str2int(tok_prn.p, tok_prn.end);
				if(rtype == RNSS_BDS) {
					// BD2: 1~37 or 51~87 or 161~197
					if(50 < prn && prn < 88) prn -= 50;

					if(prn < 160) prn += 160;
				}

				if(rtype == RNSS_GLO && prn < 32)
					prn += 64;

				r->sv_list_tmp[prn].svid = prn;
				r->sv_list_tmp[prn].elevation = str2float(tok_elevation.p, tok_elevation.end);
				r->sv_list_tmp[prn].azimuth = str2float(tok_azimuth.p, tok_azimuth.end);
				r->sv_list_tmp[prn].c_n0_dbhz = str2float(tok_snr.p, tok_snr.end);
				r->sv_list_tmp[prn].constellation = rtype;

				i++;
				num_svs++;
			}

			if (sentence == totalSentences) {
				//memcpy(r->sv_status[rtype].sv_list, r->sv_status_tmp[rtype].sv_list, sizeof(GpsSvInfo)*GPS_MAX_SVS);
				//r->sv_status[rtype].num_svs = r->sv_status_tmp[rtype].num_svs;

				// $GPGSV,2,1,...
				// $GPGSV,2,2,... commit=1
				// $BDGSV,2,1,...
				// $BDGSV,2,2,... commit=0,changed=1
				// if no GSV, then trigger changed in RMC
				r->sv_status_commit = 1;

				if ( (r->nmea_mode != CMD_MODE_GN)
						|| ((r->nmea_mode == CMD_MODE_GN) && (rtype == RNSS_BDS || rtype == RNSS_GLO))) {

					memcpy(r->sv_list, r->sv_list_tmp, sizeof(r->sv_list));
					r->sv_status_commit = 0;
					r->sv_status_changed = 1;
				}

				num_svs = 0;
				ff = 0;
			}
		}

	} else if ( !memcmp(tok.p, "RMC", 3) ) { //NMEA first line
		// SEE ALSO "GSV" COMMENTS
		// EPOCH begin, if last epoch not sent out, mark it as changed
		if(r->sv_status_commit) {
			memcpy(r->sv_list, r->sv_list_tmp, sizeof(r->sv_list));
			r->sv_status_commit = 0;
			r->sv_status_changed = 1;
		}
		if(!memcmp(tok.p - 2, "GNR", 3)) {
			r->nmea_mode = CMD_MODE_GN;
		} else if(!memcmp(tok.p - 2, "BDR", 3) || !memcmp(tok.p - 2, "GBR", 3)) {
			r->nmea_mode = CMD_MODE_BD;
		} else {
			r->nmea_mode = CMD_MODE_GPS;
		}

		memset(gps_state->reader.sv_list_tmp, 0, sizeof(gps_state->reader.sv_list_tmp));


		Token  tok_fixStatus     = nmea_tokenizer_get(tzer,2);

		if (tok_fixStatus.p[0] == 'A')
		{
			Token  tok_time          = nmea_tokenizer_get(tzer,1);
			Token  tok_latitude      = nmea_tokenizer_get(tzer,3);
			Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,4);
			Token  tok_longitude     = nmea_tokenizer_get(tzer,5);
			Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,6);
			Token  tok_speed         = nmea_tokenizer_get(tzer,7);
			Token  tok_bearing       = nmea_tokenizer_get(tzer,8);
			Token  tok_date          = nmea_tokenizer_get(tzer,9);

			nmea_reader_update_date( r, tok_date, tok_time );

			nmea_reader_update_latlong( r, tok_latitude,
					tok_latitudeHemi.p[0],
					tok_longitude,
					tok_longitudeHemi.p[0] );

			nmea_reader_update_bearing( r, tok_bearing );
			nmea_reader_update_speed  ( r, tok_speed );
		}

		check_rnss_mode(r);
		clock_gettime(CLOCK_MONOTONIC, &gps_state->ts_last_active);

	} else if ( !memcmp(tok.p, "VTG", 3) ) {

		Token  tok_fixStatus     = nmea_tokenizer_get(tzer,9);

		if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != 'N')
		{
			Token  tok_bearing       = nmea_tokenizer_get(tzer,1);
			Token  tok_speed         = nmea_tokenizer_get(tzer,5);

			nmea_reader_update_bearing( r, tok_bearing );
			nmea_reader_update_speed  ( r, tok_speed );
		}

	} else if ( !memcmp(tok.p, "ZDA", 3) ) {

		Token  tok_time;
		Token  tok_year  = nmea_tokenizer_get(tzer,4);

		if (tok_year.p[0] != '\0') {

			Token  tok_day   = nmea_tokenizer_get(tzer,2);
			Token  tok_mon   = nmea_tokenizer_get(tzer,3);

			nmea_reader_update_cdate( r, tok_day, tok_mon, tok_year );

		}

		tok_time  = nmea_tokenizer_get(tzer,1);

		if (tok_time.p[0] != '\0') {

			nmea_reader_update_time(r, tok_time);

		}


	} else {
		tok.p -= 2;
	}

	if (!gps_state->first_fix &&
			gps_state->init == STATE_INIT &&
			r->fix.flags & GPS_LOCATION_HAS_LAT_LONG) {

		um_send_ni_notification(r);

		if (gps_state->callbacks.location_cb) {
			r->fix.size = sizeof(GpsLocation);
			gps_state->callbacks.location_cb( &r->fix );
			r->fix.flags = 0;
		}

		gps_state->first_fix = 1;
	}
}

static void nmea_reader_addc( NmeaReader*  r, int  c )
{
	if (r->overflow) {
		r->overflow = (c != '\n');
		return;
	}

	if (r->pos >= (int) sizeof(r->in)-1 ) {
		r->overflow = 1;
		r->pos      = 0;
		return;
	}

	r->in[r->pos] = (char)c;
	r->pos       += 1;

	if (c == '\n') {
		gps_state_lock_fix(gps_state);
		nmea_reader_parse( r );
		gps_state_unlock_fix(gps_state);
		r->pos = 0;
	}
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       C O N N E C T I O N   S T A T E                 *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/


/*
static void gps_state_update_fix_freq(GpsState *s, int fix_freq)
{

	s->fix_freq = fix_freq;
	return;

}
*/


static void gps_state_done( GpsState*  s )
{
	// tell the thread to quit, and wait for it
	char   cmd = CMD_QUIT;
	int ret;

	do { ret=write( s->control[0], &cmd, 1 ); }
	while (ret < 0 && errno == EINTR);

	DFR("gps waiting for command thread to stop");
	gps_sleep(s);

	//pthread_join(s->thread, &dummy);
	while(isStateThreadExited ==0) usleep(1000);

	/* Timer thread depends on this state check */
	s->init = STATE_QUIT;
	s->fix_freq = -1;

	// close the control socket pair
	close( s->control[0] ); s->control[0] = -1;
	close( s->control[1] ); s->control[1] = -1;

	sem_destroy(&s->fix_sem);
	g_gpscallback = 0;

	memset(s, 0, sizeof(*s));

}

static int um220_run_hook(char* name)
{
	char   prop[PROPERTY_VALUE_MAX];
	char   buf[PROPERTY_VALUE_MAX + 20];
	int	ret;
	if(property_get("um220.gps.hookspath",prop,"/system/etc/gps") == 0) {
		ALOGE("%s: um220.gps.hookspath property is not set", __FUNCTION__);

		return 0;
	}
	sprintf(buf,"%s/%s" , prop, name);
	ret = system(buf);
	ALOGI("%s: execute hook  \"%s\" = %d", __FUNCTION__, buf, ret);
	if(ret != 0) {
		const char *pwrdev = "/sys/gps_ctrl/beidou_on";
		int ret = -1;
		char *pcmd = "0\n";

		if(strcmp(name, "start") == 0)
			pcmd = "1\n";

		if(access(pwrdev, W_OK) == 0) {
			int fd = open(pwrdev, O_WRONLY);
			if(fd >= 0) {
				ret = write(fd, pcmd, 2);

				if(ret > 0) {
					D("trying cmd '%s' on '%s'", name, pwrdev);
				}
			}
		} else {
			D("failed access on %s", pwrdev);
		}
	}

	return 0;
}

static int um220_run_hook_start()
{
	return um220_run_hook("start");
}

static int um220_run_hook_stop()
{
	return um220_run_hook("stop");
}


void gps_wakeup(GpsState *state)
{
	if( sleep_lock == 0) // it reset by um220_gps_start
	{
		gps_state_lock_fix(state);

		gps_opentty(state);

		if (um220_run_hook_start())
		{
			D("%s: Hook says: quit now", __FUNCTION__);
		}
		else
		{
			D("Send WAKEUP command to GPS");
			sleep_lock = time((time_t*)NULL);
			bOrionShutdown = 0;
		}
		bGetFormalNMEA = 0;
		gps_state_unlock_fix(state);
	}
}

void gps_sleep(GpsState *state)
{
	if (um220_run_hook_stop()) {
		D("%s: Hook says: quit now", __FUNCTION__);
		gps_state_lock_fix(state);
		started = 0;
		sleep_lock = 0; // allow next wakeup command
		gps_state_unlock_fix(state);
	}
	else
	{
		gps_state_lock_fix(state);

		if(state->fd == -1)
			gps_opentty(state);

		bOrionShutdown = 0;

		started = 0;
		sleep_lock = 0; // allow next wakeup command

		gps_state_unlock_fix(state);
	}
	gps_state_lock_fix(state);
	gps_closetty(state);
	gps_state_unlock_fix(state);
}

static void gps_state_start( GpsState*  s )
{
	char  cmd = CMD_START;
	int   ret;
	DFR("%s", __FUNCTION__);

	gps_state_lock_fix(s);
	lastcmd = CMD_START;
	gps_state_unlock_fix(s);


	do { ret=write( s->control[0], &cmd, 1 ); }
	while (ret < 0 && errno == EINTR);

	if (ret != 1)
		D("%s: could not send CMD_START command: ret=%d: %s", __FUNCTION__, ret, strerror(errno));
}


static void gps_state_stop( GpsState*  s )
{
	char  cmd = CMD_STOP;
	int   ret;

	DFR("%s", __FUNCTION__);

	gps_state_lock_fix(s);
	lastcmd = CMD_STOP;
	gps_state_unlock_fix(s);

	do
	{
		DFR("try %s", __FUNCTION__);
		ret=write( s->control[0], &cmd, 1 );
		if(ret < 0)
		{
			ALOGE("write control socket error %s", strerror(errno));
			sleep(1);
		}
	}
	while (ret < 0 && errno == EINTR);

	if (ret != 1)
		D("%s: could not send CMD_STOP command: ret=%d: %s",
				__FUNCTION__, ret, strerror(errno));
}


static int epoll_register( int  epoll_fd, int  fd )
{
	struct epoll_event  ev;
	int                 ret, flags;

	/* important: make the fd non-blocking */
	flags = fcntl(fd, F_GETFL);
	fcntl(fd, F_SETFL, flags | O_NONBLOCK);

	ev.events  = EPOLLIN;
	ev.data.fd = fd;
	do {
		ret = epoll_ctl( epoll_fd, EPOLL_CTL_ADD, fd, &ev );
	} while (ret < 0 && errno == EINTR);
	return ret;
}


static int epoll_deregister( int  epoll_fd, int  fd )
{
	int  ret;
	do {
		ret = epoll_ctl( epoll_fd, EPOLL_CTL_DEL, fd, NULL );
	} while (ret < 0 && errno == EINTR);
	return ret;
}

/* this is the main thread, it waits for commands from gps_state_start/stop and,
 * when started, messages from the QEMU GPS daemon. these are simple NMEA sentences
 * that must be parsed to be converted into GPS fixes sent to the framework
 */

static int         epoll_ctrlfd ;
static int         epoll_nmeafd ;


static void gps_state_thread( void*  arg )
{
	GpsState*   state = (GpsState*) arg;
	int         gps_fd     = state->fd;
	int         control_fd = state->control[1];
	epoll_ctrlfd   = epoll_create(1);
	epoll_nmeafd   = epoll_create(1);

	// register control file descriptors for polling
	epoll_register( epoll_ctrlfd, control_fd );

	D("gps thread running");

	state->tmr_thread = state->callbacks.create_thread_cb("um220_gps_tmr", gps_timer_thread, state);
	if (!state->tmr_thread)
	{
		ALOGE("could not create gps timer thread: %s", strerror(errno));
		started = 0;
		state->init = STATE_INIT;
		goto Exit;
	}

	state->nmea_thread = state->callbacks.create_thread_cb("um220_nmea_thread", gps_nmea_thread, state);
	if (!state->nmea_thread)
	{
		ALOGE("could not create gps nmea thread: %s", strerror(errno));
		started = 0;
		state->init = STATE_INIT;
		goto Exit;
	}

	started = 0;
	state->init = STATE_INIT;

	// now loop
	for (;;) {
		struct epoll_event   events[1];
		int                  ne, nevents;

		nevents = epoll_wait( epoll_ctrlfd, events, 1, -1 );
		if (nevents < 0) {
			if (errno != EINTR)
				ALOGE("epoll_wait() unexpected error: %s", strerror(errno));
			continue;
		}
		// D("gps thread received %d events", nevents);
		for (ne = 0; ne < nevents; ne++) {
			if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) {
				ALOGE("EPOLLERR or EPOLLHUP after epoll_wait() !?");
				goto Exit;
			}
			if ((events[ne].events & EPOLLIN) != 0) {
				int  fd = events[ne].data.fd;

				if (fd == control_fd)
				{
					char  cmd = 255;
					int   ret;
					D("gps control fd event");
					do {
						ret = read( fd, &cmd, 1 );
					} while (ret < 0 && errno == EINTR);
					D("got event cmd=%d", cmd);

					if (cmd == CMD_QUIT) {
						D("gps thread quitting on demand");
						goto Exit;
					}
					else if (cmd == CMD_START)
					{
						if (!started)
						{
							NmeaReader  *reader;
							reader = &state->reader;
							nmea_reader_init( reader );
							D("gps thread starting  location_cb=%p", state->callbacks.location_cb);
							started = 1;
							state->init = STATE_START;
							/* handle wakeup routine*/
							gps_wakeup(state);
							initdone = 0;
						}
						else
							D("LM already start");

					}
					else if (cmd == CMD_STOP) {
						if (started) {
							state->init = STATE_INIT;
						}
					}
					else if ((cmd == CMD_MODE_GPS) || (cmd == CMD_MODE_BD) || (cmd == CMD_MODE_GN)) {
						D("%s %02x %d %d",__FUNCTION__, cmd, gps_fd, gps_state->fd);
						if(gps_state->fd > 0)
							set_rnss_mode(gps_state->fd, cmd);
					}
					else if (cmd == CMD_SET_FREQ) {
						D("__%s:%d__ state->fd=%d", __FUNCTION__, __LINE__, gps_state->fd);
						if(gps_state->fd > 0) {
							set_fix_freq(gps_state->fd, gps_state->fix_freq);
						}
					} else if(cmd == CMD_SET_BAUD) {
						D("__%s:%d__ state->fd=%d", __FUNCTION__, __LINE__, gps_state->fd);
						if(gps_state->fd > 0) {
							set_baud_rate(gps_state->fd, gps_state->bps);
						}
					}
				}
				else
				{
					ALOGE("epoll_wait() returned unkown fd %d ?", fd);
					gps_fd = _gps_state->fd; //resign fd to gps_fd
				}
			}
		}
	}
Exit:
	{
		continue_thread = 0;
		close(epoll_ctrlfd);
		close(epoll_nmeafd);
		//pthread_join(state->tmr_thread, &dummy);
		//pthread_join(state->nmea_thread, &dummy);
		while(isTimerThreadExited == 0) usleep(1000);
		while(isNmeaThreadExited == 0) usleep(1000);
		isStateThreadExited = 1;
		DFR("gps control thread destroyed");
	}
	return;
}

static void gps_nmea_thread( void*  arg )
{
	GpsState *state = (GpsState *)arg;
	NmeaReader  *reader;
	reader = &state->reader;

	DFR("gps entered nmea thread: tid=%d", gettid());

	DFR("%s %d fd=%d", __FUNCTION__, __LINE__, state->fd);
	// now loop
	while (continue_thread)
	{
		char buf[512];
		int  nn, ret, fd;
		struct timeval tv;
		fd_set readfds;

		while(sleep_lock == 0 && continue_thread) //don't read while sleep
			sleep(1);

		if(state->fd == -1)
		{
			GPS_STATUS_CB(state->callbacks, GPS_STATUS_SESSION_END);
			gps_opentty(state);
			sleep(1);
			continue;
		}

		if(bOrionShutdown && started) // Orion be shutdown but LM is started, try to wake it up.
		{
			ALOGI("Try to wake orion up after 5 secs");
			sleep_lock = 0;
			sleep(5);
			GPS_STATUS_CB(state->callbacks, GPS_STATUS_SESSION_BEGIN);
			gps_wakeup(state);
			nmea_reader_init( reader );
			bOrionShutdown = 0;
		}

		if(!continue_thread)
			break;

		if(state->fd < 0) continue;
		fd = state->fd;

		FD_ZERO(&readfds);
		FD_SET(fd, &readfds);
		tv.tv_sec = 0;
		tv.tv_usec = 500*1000;	// 50ms

		ret = select(fd + 1, &readfds, NULL, NULL, &tv);
		if(ret < 0) {
			D("select errno=%d", errno);
		}

		if(ret > 0 && FD_ISSET(fd, &readfds) && is_valid_fd(fd)) {
			memset(buf,0,sizeof(buf));
			ret = read(fd, buf, sizeof(buf));
			if (ret > 0)
			{
#ifdef	VERBOSE
				static int bufid;
				memcpy(debugbuf[bufid&1], buf, min(ret, DUMP_SIZE));
				debuglen[bufid&1] = min(ret, DUMP_SIZE);
				bufid++;
				if(log_level >= LOG_VERBOSE) {
					DFR("==>%.*s", ret, buf);
					dump(buf, ret);
				}
#endif
				for (nn = 0; nn < ret; nn++)
					nmea_reader_addc(reader, buf[nn]);
			}
			else
			{
				DFR("Error on NMEA read :%s",strerror(errno));
				gps_closetty(state);
				GPS_STATUS_CB(state->callbacks, GPS_STATUS_SESSION_END);
				sleep(3); //wait Orion shutdown.
				bOrionShutdown = 1;
				continue;
			}
		}

		{
			struct timespec ts;
			// whether to switch baud rate
			clock_gettime(CLOCK_MONOTONIC, &ts);
			if(time_diff(&ts, &gps_state->ts_last_active) > 5000) {
				D("gps idle for %u ms, current[%lu,%lu], last[%lu,%lu]",
						time_diff(&ts, &gps_state->ts_last_active),
						ts.tv_sec, ts.tv_nsec,
						gps_state->ts_last_active.tv_sec,
						gps_state->ts_last_active.tv_nsec);

				if(current_bps == 9600)
					current_bps = 115200;
				else
					current_bps = 9600;
				init_serial(state->fd, current_bps);
			}
		}


		if(!continue_thread)
			break;
	}

	isNmeaThreadExited = 1;
	gps_closetty(state);
	DFR("gps nmea thread destroyed");
	return;
}

static unsigned int time_diff(struct timespec *tv1, struct timespec *tv2)
{
	return (unsigned int)((tv1->tv_sec - tv2->tv_sec)*1000 + (tv1->tv_nsec - tv2->tv_nsec)/1000000);
}

static void gps_timer_thread( void*  arg )
{
	int need_sleep = 0;
	int sleep_val = 0;
	GnssSvStatus sv_status;
	int n;

	memset(&sv_status, 0, sizeof(sv_status));

	GpsState *state = (GpsState *)arg;


	DFR("gps entered timer thread");

	do {

		while(started != 1 && continue_thread) //
		{
			usleep(500*1000);
		}

		gps_state_lock_fix(state);

		// Prepare GnssSvStatus struct with sv_list
		if(state->reader.sv_status_changed) {
			memset(&sv_status, 0, sizeof(sv_status));
			n = 0;
			for(int i = 0; i < MAX_SVS; i++) {
				if(state->reader.sv_list[i].svid) {
					// Tweak Beidou PRN
					if(state->reader.sv_list[i].constellation == GNSS_CONSTELLATION_BEIDOU &&
						state->reader.sv_list[i].svid > 160)
						state->reader.sv_list[i].svid -= 160;
					// Tweak Glonass PRN
					if(state->reader.sv_list[i].constellation == GNSS_CONSTELLATION_GLONASS &&
						state->reader.sv_list[i].svid > 64)
						state->reader.sv_list[i].svid -= 64;

					memcpy(&sv_status.gnss_sv_list[n], &state->reader.sv_list[i], sizeof(GnssSvInfo));
					sv_status.gnss_sv_list[n].size = sizeof(GnssSvInfo);
					n++;
				}
			}
			sv_status.num_svs = n;
			sv_status.size = sizeof(sv_status);
		}

		D("__%s:%d__ flags=0x%08X changed=%d num_svs=%d", __FUNCTION__, __LINE__,
				state->reader.fix.flags,
				state->reader.sv_status_changed,
				sv_status.num_svs);
		if ((state->reader.fix.flags & GPS_LOCATION_HAS_LAT_LONG) != 0) {
			int suppress = 0;

			// CTS: suppress position report for passive provider
			if(state->fix_freq > 1000) {
				struct timespec ts;
				unsigned int ms;

				clock_gettime(CLOCK_MONOTONIC, &ts);
				ms = time_diff(&ts, &state->ts_last_report);

				suppress = ms < state->fix_freq/2U;
			}

			D("gps fix cb: flags=0x%x, lat=%.6lf, lon=%.6lf, alt=%.6lf, spd=%.2f %s",
					state->reader.fix.flags,
					state->reader.fix.latitude,
					state->reader.fix.longitude,
					state->reader.fix.altitude,
					state->reader.fix.speed,
					suppress ? " suppressed" : "");

			if (state->callbacks.location_cb && !suppress) {
				state->reader.fix.size = sizeof(GpsLocation);
				state->callbacks.location_cb( &state->reader.fix );
				state->reader.fix.flags = 0;
				state->first_fix = 1;
				clock_gettime(CLOCK_MONOTONIC, &state->ts_last_report);
			}

			if (state->fix_freq == 0) {
				state->fix_freq = -1;
			}
		}

		if (state->reader.sv_status_changed) {

			if (state->callbacks.gnss_sv_status_cb) {
				int i;
				char buffer[512], *p;

#ifdef GPS_DEBUG
				p = buffer;
				for(i = 0; i < sv_status.num_svs; i++) {
					p += sprintf(p, " (%c%02d,%.0f,%c)",
							sv_status.gnss_sv_list[i].constellation == GNSS_CONSTELLATION_GPS ? 'G' :
							sv_status.gnss_sv_list[i].constellation == GNSS_CONSTELLATION_BEIDOU ? 'B' :
							sv_status.gnss_sv_list[i].constellation == GNSS_CONSTELLATION_GLONASS ? 'L' : '?',
							sv_status.gnss_sv_list[i].svid,
							sv_status.gnss_sv_list[i].c_n0_dbhz,
							sv_status.gnss_sv_list[i].flags & GNSS_SV_FLAGS_USED_IN_FIX ? 'U' : '-');
					if(i % 8 == 7) p += sprintf(p, "\n\t");
				}
				if(p - buffer) DFR("sv_list[] = %s", buffer);
#endif

				state->callbacks.gnss_sv_status_cb(&sv_status);
				// report done, purge data
				memset(gps_state->reader.sv_list, 0, sizeof(gps_state->reader.sv_list));
				state->reader.sv_status_changed = 0;
			}

		}

		need_sleep = (state->fix_freq != -1 && (state->init != STATE_QUIT) ? 1 : 0);
		sleep_val = state->fix_freq;

		gps_state_unlock_fix(state);

		if (need_sleep) {
			int i;
			// wait 100ms at most if sv_status_changed
			for(i = 0; i < sleep_val; i += 100) {
				if(state->reader.sv_status_changed) break;
				usleep(100*1000);
			}
            D("sleep because fix_freq=%d state->init=%d",state->fix_freq, state->init);
		} else {
			//D("won't sleep because fix_freq=%d state->init=%d",state->fix_freq, state->init);
		}
		um_send_ni_notification(&state->reader);
		if( state->init == STATE_INIT && lastcmd == CMD_STOP && started == 1)
		{
			int gap = 0;
			D("Wait for NMEA coming,%d,%d,%d", state->init , lastcmd, started);

			while (state->init != STATE_START && bGetFormalNMEA == 0 && continue_thread && !bOrionShutdown)
			{
				usleep(300*1000);
				if (++gap > 100)
					break;
			} ;

			D("Get NMEA %d and state %d",bGetFormalNMEA,state->init);
			// even we don't get nmea after 30 second, still force close it
			bGetFormalNMEA |= (gap >= 100);

			if( state->init == STATE_INIT && lastcmd == CMD_STOP && started == 1)
			{
				gps_sleep(state);
			}
			else
			{
				D("User enter LM before sending sleep, so drop it");
			}
		}

	} while(continue_thread);

	isTimerThreadExited = 1;
	DFR("gps timer thread destroyed");

	return;

}

static char   prop[PROPERTY_VALUE_MAX];

int gps_opentty(GpsState *state)
{

	DFR("%s", __FUNCTION__);

	if(strlen(prop) <= 0)
	{
		state->fd  = -1;
		return state->fd;
	}

	if(state->fd != -1)
		gps_closetty(state);

	do {
		state->fd = open( prop, O_RDWR | O_NOCTTY | O_NONBLOCK);
	} while (state->fd < 0 && errno == EINTR);

	if (state->fd < 0) {
		char device[256];

		// let's try add /dev/ to the property name as a last resort
		if ( snprintf(device, sizeof(device), "/dev/%s", prop) >= (int)sizeof(device) ) {
			DFR("gps serial device name too long: '%s'", prop);
			return -1;
		}

		do {
			state->fd = open( device, O_RDWR | O_NOCTTY | O_NONBLOCK);
		} while (state->fd < 0 && errno == EINTR);

		if (state->fd < 0)
		{
			ALOGE("could not open gps serial device %s: %s", prop, strerror(errno) );
			return -1;
		}
	}

	DFR("gps will read from %s(fd=%d)", prop, state->fd);

	// disable echo on serial lines
	if ( isatty( state->fd ) ) {
		init_serial(state->fd, current_bps);
	}

	epoll_register( epoll_nmeafd, state->fd );

	return 0;
}

void gps_closetty(GpsState *s)
{
	if(s->fd != -1)
	{
		DFR("%s", __FUNCTION__);
		// close connection to the QEMU GPS daemon
		epoll_deregister( epoll_nmeafd, s->fd );
		close( s->fd );
		s->fd = -1;
	}
}

static int um220_gps_start()
{
	GpsState*  s = _gps_state;
	D("%s: called", __FUNCTION__ );
	current_bps = property_get_int32("persist.sys.config.gps_baud", 9600);

	if(gps_checkstate(s) == -1)
	{
		DFR("%s: called with uninitialized state !!", __FUNCTION__);
		return -1;
	}
	gps_state_start(s);

	GPS_STATUS_CB(s->callbacks, GPS_STATUS_SESSION_BEGIN);

	return 0;
}

static void gps_state_init( GpsState*  state )
{
	state->init       = STATE_INIT;
	state->control[0] = -1;
	state->control[1] = -1;
	state->fd         = -1;
	state->fix_freq   = -1;
	state->first_fix  = 0;
	state->rnss_mode = CMD_MODE_GN;
	continue_thread = 1;
	isNmeaThreadExited = 0;
	isTimerThreadExited = 0;
	isStateThreadExited = 0;

	DFR("%s %d : tid %d, pid %d, uid %d, euid %d", __FUNCTION__, __LINE__,
			gettid(),getpid(),
			getuid(), geteuid());
	if (sem_init(&state->fix_sem, 0, 1) != 0) {
		D("gps semaphore initialization failed! errno = %d", errno);
		return;
	}

	//[ADD-BEGIN] by dk
	char productName[PROPERTY_VALUE_MAX] = {'\0'};
	char device_4g[PROPERTY_VALUE_MAX] = {"/dev/ttyUSB1"};
	char device[PROPERTY_VALUE_MAX] = {'\0'};
	
	property_get("ro.build.product", productName, "default");
	if ((strstr(productName, "t7_p_3a23")) != NULL) 
	{
		strncpy(device, device_4g, sizeof(device_4g));
	} 
	else 
	{
		strncpy(device, GPS_DEVICE, sizeof(GPS_DEVICE));
	}
	DFR("project name: %s, gps path: %s", productName, device);
	//[ADD-END]
	
	// look for a kernel-provided device name
	if (property_get("ro.kernel.android.gps", prop, device) == 0) {
		DFR("no kernel-provided gps device name");
		DFR("please set ro.kernel.android.gps property");
		return;
	}

	if(access(prop, R_OK)) {
		DFR("no permissions on %s", prop);
		//return;//del
	}

	if ( socketpair( AF_LOCAL, SOCK_STREAM, 0, state->control ) < 0 ) {
		ALOGE("could not create thread control socket pair: %s", strerror(errno));
		goto Fail;
	}

	state->thread = state->callbacks.create_thread_cb("um220_gps", gps_state_thread, state);
	if (!state->thread)
	{
		ALOGE("could not create gps thread: %s", strerror(errno));
		goto Fail;
	}
	state->callbacks.set_capabilities_cb(GPS_CAPABILITY_SCHEDULING);
	D("gps state initialized");
    
    //um220_gps_start();//[ADD]

	return;

Fail:
	gps_state_done( state );
}

#define ARRAY_SIZE(x)	(sizeof(x)/sizeof((x)[0]))
static void init_serial(int fd, int bps)
{
	struct termios opt;
//	int cm = 0;
//	int rc;
	int baud = 0;
	unsigned int i;
	struct {
		int bps;
		int baud;
	} bpstbl[] = {
		{ 115200, B115200 },
		{   9600,   B9600 },
	};

	for(i = 0; i < ARRAY_SIZE(bpstbl); i++) {
		if(bpstbl[i].bps == bps) {
			baud = bpstbl[i].baud;
			break;
		}
	}

	if(!baud) baud = B9600;

	memset(&opt, 0, sizeof(opt));
	tcgetattr(fd, &opt);

	cfmakeraw(&opt);
//	opt.c_cflag &= ~CRTSCTS;
	cfsetispeed(&opt, baud);
	cfsetospeed(&opt, baud);
	tcflush(fd, TCIOFLUSH);
	tcsetattr(fd, TCSANOW, &opt);
	tcflush(fd, TCIOFLUSH);

	/* restore control lines to make sure we're satisfied */
/*
	rc = ioctl(fd, TIOCMGET, &cm);
	cm &= ~TIOCM_LOOP;
	cm &= ~TIOCM_RTS;
	rc |= ioctl(fd, TIOCMSET, &cm);

	if(rc) DFR("error init_serial()");
*/

	D("init_serial fd=%d bps=%d", fd, bps);
	clock_gettime(CLOCK_MONOTONIC, &gps_state->ts_last_active);
	return;
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       I N T E R F A C E                               *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/


static int um220_gps_init(GpsCallbacks* callbacks)
{
	GpsState*  s = _gps_state;

	DFR("gps state initializing %d",s->init);

	s->callbacks = *callbacks;
	if (!s->init)
		gps_state_init(s);

	if(!g_gpscallback)
		g_gpscallback = callbacks;

	return 0;
}

static void um220_gps_cleanup(void)
{
	GpsState*  s = _gps_state;

	D("%s: called", __FUNCTION__);

	if (s->init)
		gps_state_done(s);
}

/*static int um220_gps_start()
{
	GpsState*  s = _gps_state;
	D("%s: called", __FUNCTION__ );
	current_bps = property_get_int32("persist.sys.config.gps_baud", 9600);

	if(gps_checkstate(s) == -1)
	{
		DFR("%s: called with uninitialized state !!", __FUNCTION__);
		return -1;
	}
	gps_state_start(s);

	GPS_STATUS_CB(s->callbacks, GPS_STATUS_SESSION_BEGIN);

	return 0;
}*/


static int um220_gps_stop()
{
	GpsState*  s = _gps_state;

	D("%s: called", __FUNCTION__ );

	if(gps_checkstate(s) == -1)
	{
		DFR("%s: called with uninitialized state !!", __FUNCTION__);
		return -1;
	}

	//close LM first
	gps_state_stop(s);
	D("Try to change state to init");
	//change state to INIT
	GPS_STATUS_CB(s->callbacks, GPS_STATUS_SESSION_END);

	// save baud rate
	{
		char buf[32];
		sprintf(buf, "%d", current_bps);
        DFR("%s: persist.sys.config.gps_baud: %s", __FUNCTION__, buf);
		property_set("persist.sys.config.gps_baud", buf);
	}

	return 0;
}

static void um220_gps_set_fix_frequency(int freq)
{
	GpsState*  s = _gps_state;

	if(gps_checkstate(s) == -1)
	{
		DFR("%s: called with uninitialized state !!", __FUNCTION__);
		return;
	}

	s->fix_freq = (freq <= 0) ? 1000 : freq;

	D("gps fix frequency set to %d secs", freq);
	if(gps_state->control[0] > 0) {
		int ret;
		char cmd = CMD_SET_FREQ;
		D("%s cmd=%02x freq=%d",__FUNCTION__, cmd, s->fix_freq);
		do
		{
			ret = write(gps_state->control[0], &cmd, 1);
		}
		while (ret < 0 && errno == EINTR);
	}
}

static int um220_gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
	D("__%s:%d__", __FUNCTION__, __LINE__);
	(void)time;
	(void)timeReference;
	(void)uncertainty;
	return 0;
}

static void um220_gps_delete_aiding_data(GpsAidingData flags)
{
	GpsState*  s = _gps_state;
	int v;
	char cmd[32];
	unsigned char sum, bits = 0;

	if(flags == GPS_DELETE_ALL) {
		bits |= 0x95;
	} else if(flags & GPS_DELETE_EPHEMERIS) {
		bits |= 0x01;
	} else if(flags & (GPS_DELETE_POSITION|GPS_DELETE_TIME)) {
		bits |= 0x04;
	} else if(flags & GPS_DELETE_IONO) {
		bits |= 0x10;
	} else if(flags & GPS_DELETE_ALMANAC) {
		bits |= 0x80;
	}

	sprintf(cmd, "$RESET,0,h%02X", bits);
	v = strlen(cmd);
	sum = bd_check_eor(cmd + 1, v - 1);
	sprintf(cmd + v, "*%02X\r\n",sum);
	if(s->fd > 0)
		gps_dev_send(s->fd, cmd);
	else {
		D("__%s: invalid fd = %d", __FUNCTION__, s->fd);
	}
}

static int um220_gps_inject_location(double latitude, double longitude, float accuracy)
{
	D("__%s:%d__", __FUNCTION__, __LINE__);
	(void)latitude;
	(void)longitude;
	(void)accuracy;
	return 0;
}

static int um220_gps_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
		uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time)
{
	GpsState*  s = _gps_state;
	int v = (mode >> 4)&0x3;
//	int ret;

	(void)recurrence;
	(void)preferred_accuracy;
	(void)preferred_time;

	// mask private fields
	mode &= 0xf;

	// only standalone is supported for now.
	if (mode != GPS_POSITION_MODE_STANDALONE)
	{
		D("%s: set GPS POSITION mode error! (mode:%d) ", __FUNCTION__, mode);
		D("Set as standalone mode currently! ");
		//        return -1;
	}

	if (!s->init) {
		D("%s: called with uninitialized state !!", __FUNCTION__);
		return -1;
	}

	s->fix_freq = min_interval;
	if (s->fix_freq ==0)
	{
		s->fix_freq = 1000;
	}
	D("gps fix frequency set to %d ms", s->fix_freq);
	s->fix_freq_pending = 1;

	// if not set, do not change default
	if(v) {
		s->rnss_mode = valid_modes[v];
	}
	D("RNSS mode %s %02x",__FUNCTION__, s->rnss_mode);
	/*
	   do
	   {
	   ret = write(s->control[0], &cmd, 1);
	   D("SEND CMD %d",cmd);
	   }
	   while (ret < 0 && errno == EINTR);
	   */
	return 0;
}

int gps_checkstate(GpsState *s)
{
	if (!s->init) {

		if(g_gpscallback)
			um220_gps_init(g_gpscallback);

		if(!s->init)
		{
			ALOGE("%s: still called with uninitialized state !!", __FUNCTION__);
			return -1;
		}
	}

	return 0;
}

#ifdef UM220_GPSXtra
/*
 * no special parameter to be initialized here
 */
static int um220_gps_xtra_init(GpsXtraCallbacks* callbacks)
{
	D("%s: entered", __FUNCTION__);

	return 0;
}

/*
 * real nmea command injection function
 */
static int um220_gps_xtra_inject_data(char *data, int length)
{
	GpsState*  s = _gps_state;
	int cnt;

	D("%s: entered, data: %.2x (len: %d)", __FUNCTION__, *data, length);

	if (write(s->fd, data, length) < 0)
	{
		D("Send command ERROR!");
		return -1;
	}

#ifdef GPS_DEBUG
	/* debug */
	D("inject cmds: ");
	for (cnt=0; cnt<length; cnt++)
		D("%.2x (%d) ", data[cnt], data[cnt]);
	/* end debug */
	D("%s: closed", __FUNCTION__);
#endif
	return 0;
}

static const GpsXtraInterface um220GpsXtraInterface = {
	sizeof(GpsXtraInterface),
	um220_gps_xtra_init,
	um220_gps_xtra_inject_data,
};
#endif /* UM220_GPSXtra */

#ifdef UM220_GPSNi
/*
 * Registers the callbacks for HAL to use
 */
static void um220_gps_ni_init(GpsNiCallbacks* callbacks)
{
	GpsState*  s = _gps_state;

	D("%s: entered", __FUNCTION__);

	if (callbacks)
	{
		s->ni_init = 1;
		s->ni_callbacks = *callbacks;
	}
}

/*
 * Sends a response to HAL
 */
static void um220_gps_ni_respond(int notif_id, GpsUserResponseType user_response)
{
	// D("%s: entered", __FUNCTION__);
}

static const GpsNiInterface um220GpsNiInterface = {
	sizeof(GpsNiInterface),
	um220_gps_ni_init,
	um220_gps_ni_respond,
};
#endif // UM220_GPSNi

static const void* um220_gps_get_extension(const char* name)
{
	if (strcmp(name, GPS_XTRA_INTERFACE) == 0)
	{
#ifdef UM220_GPSXtra
		D("%s: found xtra extension", __FUNCTION__);
		return (&um220GpsXtraInterface);
#endif // UM220_GPSXtra
	}
	else if (strcmp(name, GPS_NI_INTERFACE) == 0)
	{
#ifdef UM220_GPSNi
		D("%s: found ni extension", __FUNCTION__);
		return (&um220GpsNiInterface);
#endif // UM220_GPSNi
	}
	D("%s: no GPS extension for %s is found", __FUNCTION__, name);
	return NULL;
}

static const GpsInterface  um220GpsInterface = {
	.size =sizeof(GpsInterface),
	.init = um220_gps_init,
	.start = um220_gps_start,
	.stop = um220_gps_stop,
	.cleanup = um220_gps_cleanup,
	.inject_time = um220_gps_inject_time,
	.inject_location = um220_gps_inject_location,
	.delete_aiding_data = um220_gps_delete_aiding_data,
	.set_position_mode = um220_gps_set_position_mode,
	.get_extension = um220_gps_get_extension,
};

const GpsInterface* gps_get_hardware_interface()
{
	return &um220GpsInterface;
}

/* for GPSXtraIterface */

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       D E V I C E                                     *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

static void gps_dev_send(int fd, char *msg)
{
	int i, n, ret;

	i = strlen(msg);

	n = 0;

	DFR("function gps_dev_send: %s", msg);
	do {

		ret = write(fd, msg + n, i - n);

		if (ret < 0 && errno == EINTR) {
			continue;
		}

		n += ret;

	} while (n < i);

	// drain cmd
	tcdrain(fd);

	return;
}

static unsigned char bd_check_eor(char *buf, int len)
{
	unsigned char csum = 0;

	while(len--)
		csum ^= buf[len];

	return csum;
}

static void set_rnss_mode(int fd, char mode)
{
	int v;
	unsigned char sum;
	char cmd[32];

	D("%s %02x %d",__FUNCTION__, mode, fd);
	switch(mode) {
		case CMD_MODE_GPS:
			mode = 1;
			break;
		case CMD_MODE_BD:
			mode = 10;
			break;
		case CMD_MODE_GN:
			mode = 11;
			break;
		default: 
			mode=0;
			break;
	}

	// BD: $CFGSYS,h10*5E
	// GN: $CFGSYS,h11*5F
	if(mode) {
		sprintf(cmd, "$CFGSYS,h%02d",mode);
		v = strlen(cmd);
		sum = bd_check_eor(cmd + 1, v - 1);
		sprintf(cmd + v, "*%02X\r\n",sum);
	}
	else {
		sprintf(cmd, "$CFGSYS\r\n");
	}


	gps_dev_send(fd, cmd);
	D("gps sent to device: %s", cmd);
	memset(&gps_state->reader.sv_list, 0, sizeof(gps_state->reader.sv_list));
	memset(&gps_state->reader.sv_list_tmp, 0, sizeof(gps_state->reader.sv_list_tmp));
	gps_state->reader.sv_status_changed = 1;
	return;
}

static void init_cmds(GpsState *state)
{
	int v;
	unsigned char sum;
	char cmd[32];
	int fd = state->fd;

	D("%s %d",__FUNCTION__, fd);

	// e.g.
	//    $CFGPRT,,0,115200,3,3*0F
	//    $CFGPRT,,0,115200,129,3*06
	//    $CFGPRT,,0,9600,3,3*07
	//    $CFGPRT,,0,9600,129,3*0E
	D("Get chip config");
	gps_dev_send(fd, "$CFGPRT\r\n"); usleep(1000);
	gps_dev_send(fd, "$CFGSYS\r\n"); usleep(1000);

	sprintf(cmd,"$CFGPRT,,0,,%d,%d", cfg_inproto, cfg_outproto);
	v = strlen(cmd);
	sum = bd_check_eor(cmd + 1, v - 1);
	sprintf(cmd + v, "*%02X\r\n",sum);

	gps_dev_send(fd, cmd);
	if(cfg_outproto & 0x8) {
		sprintf(cmd,"$CFGMSG,255,0,1");
		v = strlen(cmd);
		sum = bd_check_eor(cmd + 1, v - 1);
		sprintf(cmd + v, "*%02X\r\n",sum);

		gps_dev_send(fd, cmd);
	}

	gps_dev_send(fd, "$CFGSAVE\r\n");
}

static void set_baud_rate(int fd, int bps)
{
	int v;
	unsigned char sum;
	char cmd[32];

	D("%s %d",__FUNCTION__, fd);

	sprintf(cmd,"$CFGPRT,,0,%d,%d,%d", bps, cfg_inproto, cfg_outproto);
	v = strlen(cmd);
	sum = bd_check_eor(cmd + 1, v - 1);
	sprintf(cmd + v, "*%02X\r\n",sum);

	gps_dev_send(fd, cmd);
	D("gps sent to device: %s", cmd);

	current_bps = bps;
	init_serial(fd, bps);

	usleep(50*1000);
	gps_dev_send(fd, "$CFGSAVE\r\n");

	return;
}

static void check_rnss_mode(NmeaReader* r)
{
	char prop[PROPERTY_VALUE_MAX];
	int v;

	D("__%s:%d", __FUNCTION__, __LINE__);
	if(!initdone) {
		init_cmds(gps_state);
		initdone = 1;
	}

	if(property_get("persist.sys.config.gps_debug", prop, 0) > 0) {
		v = strtoul(prop, NULL, 0);
		if(v != log_level) {
			ALOGI("log_level changed to %d\n", log_level);
			log_level = v;
		}
	}

	if(property_get("persist.sys.config.gps", prop, 0) > 0) {
		v = valid_modes[prop[0]&3];
		if(v != gps_state->rnss_mode) {
			ALOGI("hook into persist.sys.config.gps %s -> %d", prop, v);
			gps_state->rnss_mode = v;
		}
	}

	if(gps_state->rnss_mode != r->nmea_mode) {
		if(gps_state->control[0] > 0) {
			//set_rnss_mode(gps_state->fd, r->nmea_mode);
			int ret;
			char cmd = (char) gps_state->rnss_mode;
			D("%s mode=%02x cmd=%02x nmea=%02x",__FUNCTION__, gps_state->rnss_mode,cmd, r->nmea_mode);
			do
			{
				ret = write(gps_state->control[0], &cmd, 1);
			}
			while (ret < 0 && errno == EINTR);
		}

	}

	// check output freq
	if(gps_state->fix_freq_pending)
		um220_gps_set_fix_frequency(gps_state->fix_freq);

	// check baud rate
	v = property_get_int32("persist.sys.config.gps_baud", current_bps);
	D("prop: baud=%d current=%d", v, current_bps);
	if(v > 0 && v != current_bps) {
		gps_state->bps = v;

		if(gps_state->control[0] > 0) {
			//set_rnss_mode(gps_state->fd, r->nmea_mode);
			int ret;
			char cmd = CMD_SET_BAUD;
			D("%s cmd=%02x",__FUNCTION__, cmd);
			do
			{
				ret = write(gps_state->control[0], &cmd, 1);
			}
			while (ret < 0 && errno == EINTR);
		}
	}
}

static void set_fix_freq(int fd, int freq)
{
	int v;
	unsigned char sum;
	char cmd[32];

	// make sure we have reasonable interval(200ms~30s)
	if(freq < 200) freq = 200;
	if(freq > 30*1000) freq = 30*1000;

	D("%s fd=%d, freq=%d",__FUNCTION__, fd, freq);
	if(freq > 0) {
		sprintf(cmd, "$CFGNAV,%d,%d,3", freq, freq);
		v = strlen(cmd);
		sum = bd_check_eor(cmd + 1, v - 1);
		sprintf(cmd + v, "*%02X\r\n",sum);
	}
	else {
		sprintf(cmd, "$CFGNAV\r\n");
	}


	gps_dev_send(fd, cmd);
	D("gps sent to device: %s", cmd);
	gps_state->fix_freq_pending = 0;

	return;
}

