#include "tbox_gps.h"
#include "tbox_buffer.h"

#undef	 GPS_DEBUG_TOKEN	/* print out NMEA tokens */

#ifdef GPS_DEBUG
#  define  D(...)   ALOGD(__VA_ARGS__)
#else
#  define  D(...)   ((void)0)
#endif

extern int tbox_gps_init(GpsCallbacks* callbacks);

static GpsState  _gps_state[1];
static GpsState *gps_state = _gps_state;
GpsCallbacks* g_gpscallback = 0;
static int bGetFormalNMEA = 1;

static void tbox_gps_state_done(GpsState*  s)
{
    /* Timer thread depends on this state check */
    s->init = STATE_QUIT;
    s->fix_freq = -1;

	g_gpscallback = 0;

    memset(s, 0, sizeof(*s));
}

static void tbox_gps_cleanup(void) {
    GpsState*  s = _gps_state;

	ALOGE("tbox_gps_cleanup");

    if (s->init)
        tbox_gps_state_done(s);
}

static int tbox_gps_start() {
	ALOGE("tbox_gps_start");
    return 0;
}

static int tbox_gps_stop() {
	ALOGE("tbox_gps_stop");
    return 0;
}

static void tbox_gps_set_fix_frequency(int freq) {
	ALOGE("tbox_gps_set_fix_frequency: %d", freq);
}

static int tbox_gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty) {
	ALOGE("tbox_gps_inject_time");
    return 0;
}

static void tbox_gps_delete_aiding_data(GpsAidingData flags) {
	ALOGE("tbox_gps_delete_aiding_data");
}

static int tbox_gps_inject_location(double latitude, double longitude, float accuracy) {
	ALOGE("tbox_gps_inject_location");
    return 0;
}

static int tbox_gps_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
									  uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time) {
	ALOGE("tbox_gps_set_position_mode");
    return 0;
}

static const void* tbox_gps_get_extension(const char* name)
{
	ALOGE("tbox_gps_get_extension: %s", name);
	return NULL;
}


static const GpsInterface  TBoxGpsInterface = {
  .size =sizeof(GpsInterface),
  .init = tbox_gps_init,
  .start = tbox_gps_start,
  .stop = tbox_gps_stop,
  .cleanup = tbox_gps_cleanup,
  .inject_time = tbox_gps_inject_time,
  .inject_location = tbox_gps_inject_location,
  .delete_aiding_data = tbox_gps_delete_aiding_data,
  .set_position_mode = tbox_gps_set_position_mode,
  .get_extension = tbox_gps_get_extension,
};

const GpsInterface* gps_get_hardware_interface()
{
    return &TBoxGpsInterface;
}

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

static int
nmea_tokenizer_init( NmeaTokenizer*  t, const char*  p, const char*  end )
{
    int    count = 0;
    char*  q;

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

    t->count = count;
    return count;
}

static Token
nmea_tokenizer_get( NmeaTokenizer*  t, int  index )
{
    Token  tok;
    static const char*  dummy = "";

    if (index < 0 || index >= t->count) {
        tok.p = tok.end = dummy;
    } else
        tok = t->tokens[index];

    return tok;
}


static int
str2int( const char*  p, const char*  end )
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

static double
str2float( const char*  p, const char*  end )
{
    int   result = 0;
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

#if 1	
	ttime = mktime( &tm );
	*timestamp = ttime;		// Hi gaode map, let us fuck you!
#else
	nmea_reader_update_utc_diff(r);
	ttime = mktime( &tm );
	*timestamp = ttime - r->utc_diff;
#endif

    return 0;
}

static int
nmea_reader_update_time( NmeaReader*  r, Token  tok )
{
    time_t timestamp = 0;
    int ret = nmea_reader_get_timestamp( r, tok, &timestamp);
    if (0 == ret)
        r->fix.timestamp = (long long)timestamp * 1000;
    return ret;
}

static int
nmea_reader_update_cdate( NmeaReader*  r, Token  tok_d, Token tok_m, Token tok_y )
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

static int
nmea_reader_update_date( NmeaReader*  r, Token  date, Token  mtime )
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


static double
convert_from_hhmm( Token  tok )
{
    double  val     = str2float(tok.p, tok.end);
    int     degrees = (int)(floor(val) / 100);
    double  minutes = val - degrees*100.;
    double  dcoord  = degrees + minutes / 60.0;
    return dcoord;
}


static int
nmea_reader_update_latlong( NmeaReader*  r,
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

    r->fix.flags    |= GPS_LOCATION_HAS_LAT_LONG;
    r->fix.latitude  = lat;
    r->fix.longitude = lon;
    return 0;
}


static int
nmea_reader_update_altitude( NmeaReader*  r,
                             Token        altitude,
                             Token        units )
{
    double  alt;
    Token   tok = altitude;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_ALTITUDE;
    r->fix.altitude = str2float(tok.p, tok.end);
    return 0;
}

static int
nmea_reader_update_accuracy( NmeaReader*  r,
                             Token        accuracy )
{
    double  acc;
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

static int
nmea_reader_update_bearing( NmeaReader*  r,
                            Token        bearing )
{
    double  alt;
    Token   tok = bearing;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_BEARING;
    r->fix.bearing  = str2float(tok.p, tok.end);
    return 0;
}


static int
nmea_reader_update_speed( NmeaReader*  r,
                          Token        speed )
{
    double  alt;
    Token   tok = speed;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_SPEED;
    r->fix.speed    = str2float(tok.p, tok.end);
    r->fix.speed   *= 0.514444;    // fix for Speed Unit form Knots to Meters per Second
    return 0;
}

static int
nmea_reader_update_timemap( NmeaReader* r,
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


static void
nmea_reader_parse( NmeaReader*  r )
{
   /* we received a complete sentence, now parse it to generate
    * a new GPS fix...
    */
    NmeaTokenizer  tzer[1];
    Token          tok;

    if (r->pos < 9) {
         return;
    }

    if (gps_state->callbacks.nmea_cb) {
        struct timeval tv;
        unsigned long long mytimems;
        gettimeofday(&tv,NULL);
        mytimems = tv.tv_sec * 1000 + tv.tv_usec / 1000;
        gps_state->callbacks.nmea_cb(mytimems, r->in, r->pos);
    }

    nmea_tokenizer_init(tzer, r->in, r->in + r->pos);
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

	r->fix.accuracy = 3;	// zhangzhenbang temp solution is 3 meters
	r->fix.flags   |= GPS_LOCATION_HAS_ACCURACY;

#if 0
    if (tok.p + 5 > tok.end) {
        /* for $PUNV sentences */
        if ( !memcmp(tok.p, "PUNV", 4) ) {
            Token tok_cfg = nmea_tokenizer_get(tzer,1);

            if (!memcmp(tok_cfg.p, "CFG_R", 5)) {
            } else if ( !memcmp(tok_cfg.p, "QUAL", 4) ) {
                Token  tok_sigma_x   = nmea_tokenizer_get(tzer, 3);

                if (tok_sigma_x.p[0] != ',') {
                    Token tok_accuracy      = nmea_tokenizer_get(tzer, 10);
                    nmea_reader_update_accuracy(r, tok_accuracy);
                }
            } else if (!memcmp(tok_cfg.p, "TIMEMAP", 7))
            {
                Token systime = nmea_tokenizer_get(tzer, 8); // system time token
                Token timestamp = nmea_tokenizer_get(tzer, 2); // UTC time token
                nmea_reader_update_timemap(r, systime, timestamp);
            }
        }else{
        }
        return;
    }
#endif

	if ( !memcmp(tok.p, "GPG", 3) ) //GPGSA,GPGGA,GPGSV
		bGetFormalNMEA = 1;
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

          r->sv_status.used_in_fix_mask = 0ul;

          for (i = 3; i <= 14; ++i){

            Token  tok_prn  = nmea_tokenizer_get(tzer, i);
            int prn = str2int(tok_prn.p, tok_prn.end);

            /* only available for PRN 1-32 */
            if ((prn > 0) && (prn < 33)){
              r->sv_status.used_in_fix_mask |= (1ul << (prn-1));
              r->sv_status_changed = 1;
              /* mark this parameter to identify the GSA is in fixed state */
              r->gsa_fixed = 1;
            }
          }

        }else {
          if (r->gsa_fixed == 1) {
            r->sv_status.used_in_fix_mask = 0ul;
            r->sv_status_changed = 1;
            r->gsa_fixed = 0;
          }
        }
    } else if ( !memcmp(tok.p, "GSV", 3) ) {

        Token  tok_noSatellites  = nmea_tokenizer_get(tzer, 3);
        int    noSatellites = str2int(tok_noSatellites.p, tok_noSatellites.end);

        if (noSatellites > 0) {

          Token  tok_noSentences   = nmea_tokenizer_get(tzer, 1);
          Token  tok_sentence      = nmea_tokenizer_get(tzer, 2);

          int sentence = str2int(tok_sentence.p, tok_sentence.end);
          int totalSentences = str2int(tok_noSentences.p, tok_noSentences.end);
          int curr;
          int i;

          if (sentence == 1) {
              r->sv_status_changed = 0;
              r->sv_status.num_svs = 0;
          }

          curr = r->sv_status.num_svs;

          i = 0;

          while (i < 4 && r->sv_status.num_svs < noSatellites){

                 Token  tok_prn = nmea_tokenizer_get(tzer, i * 4 + 4);
                 Token  tok_elevation = nmea_tokenizer_get(tzer, i * 4 + 5);
                 Token  tok_azimuth = nmea_tokenizer_get(tzer, i * 4 + 6);
                 Token  tok_snr = nmea_tokenizer_get(tzer, i * 4 + 7);

                 r->sv_status.sv_list[curr].prn = str2int(tok_prn.p, tok_prn.end);
                 r->sv_status.sv_list[curr].elevation = str2float(tok_elevation.p, tok_elevation.end);
                 r->sv_status.sv_list[curr].azimuth = str2float(tok_azimuth.p, tok_azimuth.end);
                 r->sv_status.sv_list[curr].snr = str2float(tok_snr.p, tok_snr.end);

                 r->sv_status.num_svs += 1;

                 curr += 1;

                 i += 1;
          }

          if (sentence == totalSentences) {
              r->sv_status_changed = 1;
          }
        }

    } else if ( !memcmp(tok.p, "RMC", 3) ) {

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
        r->fix.flags & GPS_LOCATION_HAS_LAT_LONG) {

        if (gps_state->callbacks.location_cb) {
            gps_state->callbacks.location_cb( &r->fix );
            r->fix.flags = 0;
        }

        gps_state->first_fix = 1;
    }
}

static void
nmea_reader_addc( NmeaReader*  r, int  c )
{
    int cnt;

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
        nmea_reader_parse( r );
        r->pos = 0;
    }
}

int tbox_gps_udp_port_open(GpsState* state)
{
    struct sockaddr_in si_me; // Structures for me with socket addr info
    
    if (state->fd != -1)
		close(state->fd);
	
	/* open socket */
	if ((state->fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        ALOGD("unable to create socket: %s\n", strerror(errno));
        return -1;
    }
	
    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons((unsigned short) TBOX_UDP_SERVER_PORT);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    const int opt = 1;  
    int nb = 0;  
    nb = setsockopt(state->fd, SOL_SOCKET, SO_BROADCAST, (char *)&opt, sizeof(opt));  
    if(nb == -1)  
    {  
		ALOGD("set socket error.... \n");
        return -1;  
    }  

    if (bind(state->fd, (struct sockaddr *) (void *) &si_me, sizeof(si_me))==-1)
    {
        ALOGD("unable to bind socket: %s\n", strerror(errno));
		close(state->fd);
		state->fd = -1;
    }
	else
		ALOGD("Udp port opened, fd = %d", state->fd);

    return state->fd;
}

void tbox_gps_udp_port_close(GpsState* state) {
	if (state->fd != -1) {
	   close(state->fd);
	}
}

int tbox_gps_udp_data_recv(GpsState* state, char * pBuf, int buflen)
{
    struct sockaddr_in si_other; // Structures for me and others with socket addr info

    memset(&si_other, 0, sizeof(si_other));

    int slen = sizeof(si_other);
    int conn;

#if 1
	int len = recvfrom(state->fd,
				   pBuf,
				   (unsigned int) buflen,
				   0,
				   (struct sockaddr *) (void *) &si_other,
				   (socklen_t *)&slen);

	ALOGE("gps udp[%d]:\n%s", len, pBuf);

#else

	int len = recvfrom(state->fd,
				   pBuf,
				   (unsigned int) buflen,
				   0,
				   (struct sockaddr *) (void *) &si_other,
				   (socklen_t *)&slen);

	char buf[1024];
	char path[128];

	memset(path, 0, sizeof(path));
	sprintf(path, "/data/local/tmp/%d.bin", ++state->test_count);

	ALOGE("gps path:%s", path);

	FILE *p = fopen(path, "r");
	memset(buf, 0, 1024);
	len = fread(buf, sizeof(char), sizeof(buf), p);
	memcpy(pBuf, buf, len);
	fclose(p);
	
	if (state->test_count >= 4) {
		state->test_count = 1;
	}

	char tmpBuf[1024];
	memset(tmpBuf, 0, sizeof(tmpBuf));
	memcpy(tmpBuf, pBuf, len);
	//ALOGE("gps recvPort[%d]:\n%s", len, tmpBuf);
#endif

    return len;
}

static void tbox_gps_data_publish(GpsState* state)
{
//	ALOGE("tbox_gps_data_publish");
	
	if ((state->reader.fix.flags & GPS_LOCATION_HAS_LAT_LONG) != 0) {

	  //D("gps fix cb: 0x%x", state->reader.fix.flags);

	  if (state->callbacks.location_cb) {
	      state->callbacks.location_cb( &state->reader.fix );
	      state->reader.fix.flags = 0;
	      state->first_fix = 1;
	  }
	}

	if (state->reader.sv_status_changed != 0) {

	  // D("gps sv status callback");

	  if (state->callbacks.sv_status_cb) {
	      state->callbacks.sv_status_cb( &state->reader.sv_status );
	      state->reader.sv_status_changed = 0;
	  }

	}
}

static void tbox_gps_nmea_thread(void* arg) {
	int frameCount = 0;

	GpsState *state = (GpsState *)arg;
	NmeaReader	*reader;
	reader = &state->reader;

	//开启udp端口
	tbox_gps_udp_port_open(state);

	//初始化缓冲区
	tbox_gps_buffer_init(state);

	while (1)
	{
		int  nn;

		int iSize = tbox_gps_udp_data_recv(state, state->tboxbuf, 1024);
		
		if (iSize)
        {
			tbox_gps_buffer_append(state, state->tboxbuf, iSize);

			int isExistFrame = tbox_gps_buffer_frame_checkout(state);

			if (isExistFrame != 1)
			{
				//ALOGE("gps GPS frame data is incomplete!");
			}
			else
			{
				int i, nmeaLength;
				frameCount = tbox_gps_buffer_get_frame_count(state);

				//ALOGE("gpx -----------------------xxx--------------------------");
				
	            for (i = 0; i < frameCount; i++)
	            {
	            	char *pFrame = tbox_gps_buffer_get_frame(state, i);
					if (pFrame != NULL)
					{
						memset(state->tboxbuf, 0, sizeof(state->tboxbuf));
						strcpy(state->tboxbuf, pFrame);
						
			            nmeaLength = strlen(state->tboxbuf);
		            	//ALOGE("gps nmeaBuf[%d]:%s", nmeaLength, state->tboxbuf);
						for (nn = 0; nn < nmeaLength; nn++) 
						{
							nmea_reader_addc( reader, state->tboxbuf[nn] );
						}
					}
	        	}

				// 提交数据
				tbox_gps_data_publish(state);
			}
	 	}
		else
		{
			ALOGE("Error on NMEA read :%s", state->tboxbuf);
		}
	}
Exit:
	tbox_gps_buffer_deinit();	
	tbox_gps_udp_port_close(state);
}

static void tbox_gps_state_init(GpsState* state) {
	void *dummy;

    state->init       = STATE_INIT;
    state->fd         = -1;
    state->fix_freq   = 1;
    state->first_fix  = 0;

	state->nmea_thread = state->callbacks.create_thread_cb(
		"tbox_nmea_thread", 
		tbox_gps_nmea_thread, 
		state);
	if (!state->nmea_thread)
	{
		ALOGE("could not create gps nmea thread: %s", strerror(errno));
		goto Exit;
	}

Exit:
	ALOGE("tbox_gps_state_init -------exit");
	pthread_join(state->nmea_thread, &dummy);
}

int tbox_gps_init(GpsCallbacks* callbacks) {
    GpsState*  s = _gps_state;

	ALOGE("gps tbox_gps_init: %d, callbacks: %d", s->init, callbacks);

    s->callbacks = *callbacks;
    if (!s->init)
        tbox_gps_state_init(s);

	if(!g_gpscallback)
		g_gpscallback = callbacks;

    return 0;
}
