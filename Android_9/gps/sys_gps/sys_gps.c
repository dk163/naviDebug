/******************************************************************************

  Copyright (C), 2019-2029, DCN Co., Ltd.

 ******************************************************************************
  File Name     : sys_gps.c
  Version       : Initial Draft
  Author        : Kang
  Created       : 2019/6/1
  Last Modified :
  Description   : my gps
  Function List :
              close_gps
              convert_from_hhmm
              epoll_deregister
              epoll_register
              gps_checkstate
              gps_closetty
              gps_dev_calc_nmea_csum
              gps_dev_send
              gps_done
              gps_get_gps_interface
              gps_init
              gps_lock_fix
              gps_opentty
              gps_sleep
              gps_start
              gps_stop
              gps_thread
              gps_unlock_fix
              gps_update_fix_freq
              gps_wakeup
              isValid_nmea_cmd_checksum
              parse_nmea_reader_addc
              parse_nmea_reader_get_timestamp
              parse_nmea_reader_init
              parse_nmea_reader
              parse_nmea_reader_update_accuracy
              parse_nmea_reader_update_altitude
              parse_nmea_reader_update_bearing
              parse_nmea_reader_update_cdate
              parse_nmea_reader_update_date
              parse_nmea_reader_update_latlong
              parse_nmea_reader_update_speed
              parse_nmea_reader_update_time
              parse_nmea_reader_update_timemap
              parse_nmea_reader_update_utc_diff
              parse_nmea_tokenizer_get
              parse_nmea_tokenizer_init
              open_gps
              str2float
              str2int
              sys_gps_cleanup
              sys_gps_delete_aiding_data
              sys_gps_get_extension
              sys_gps_init
              sys_gps_inject_location
              sys_gps_inject_time
              sys_gps_set_fix_frequency
              sys_gps_set_position_mode
              sys_gps_start
              sys_gps_stop
  History       :
  1.Date        : 2019/6/13
    Author      : Kang
    Modification: Created file

******************************************************************************/


#include "sys_gps.h"

static void gps_lock_fix(GpsState *state) {
    int ret;
    do {
        ret=sem_wait(&state->fix_sem);
    } while (ret < 0 && errno == EINTR);
    if (ret < 0) {
        AIP_LOGI("Error in GPS state lock:%s\n", strerror(errno));
    }
}

static void gps_unlock_fix(GpsState *state) {
    if (sem_post(&state->fix_sem) == -1)
	{
		if(errno == EAGAIN)
			if(sem_post(&state->fix_sem)== -1)
				AIP_LOGI("Error in GPS state unlock:%s\n", strerror(errno));
	}
}

static int parse_nmea_tokenizer_init( NmeaTokenizer*  t, const char*  p, const char*  end )
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

static Token parse_nmea_tokenizer_get( NmeaTokenizer*  t, int  index )
{
    Token  tok;
    static const char*  dummy = "";

    if (index < 0 || index >= t->count) {
        tok.p = tok.end = dummy;
    }
	else {
	    tok = t->tokens[index];
	}

    return tok;
}


static int str2int( const char*  p, const char*  end )
{
    int result = 0;
    int len = end-p;

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
    int result = 0;
    int len = end-p;
    char temp[32];

    if (len >= (int)sizeof(temp))
        return 0.;

    memcpy(temp, p, len);
    temp[len] = 0;
    return strtod(temp, NULL);	
}

static void parse_nmea_reader_update_utc_diff( NmeaReader*  r )
{
    time_t now = time(NULL);
    struct tm tm_local;
    struct tm tm_utc;
    long time_local, time_utc;

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


static void parse_nmea_reader_init( NmeaReader*  r )
{
    memset( r, 0, sizeof(*r) );

    r->pos = 0;
    r->overflow = 0;
    r->utc_year = -1;
    r->utc_mon  = -1;
    r->utc_day  = -1;

    //parse_nmea_reader_update_utc_diff( r );//del
}

static int parse_nmea_reader_get_timestamp(NmeaReader*  r, Token  tok, time_t *timestamp)
{
    int hour;
	int minute;
    double seconds;
    struct tm  tm;
    time_t ttime;

    if (tok.p + 6 > tok.end)
        return -1;

    if (r->utc_year < 0) {
        return -1;
    }

    hour = str2int(tok.p,   tok.p+2);
    minute  = str2int(tok.p+2, tok.p+4);
    seconds = str2float(tok.p+4, tok.end);

    tm.tm_hour = hour;
    tm.tm_min  = minute;
    tm.tm_sec   = (int) seconds;
    tm.tm_year = r->utc_year - 1900;
    tm.tm_mon  = r->utc_mon - 1;
    tm.tm_mday = r->utc_day;
    tm.tm_isdst = -1;

    //AIP_LOGI("h: %d, m: %d, s: %d", tm.tm_hour, tm.tm_min, tm.tm_sec);
    //AIP_LOGI("Y: %d, M: %d, D: %d", tm.tm_year, tm.tm_mon, tm.tm_mday);

#if 0
	parse_nmea_reader_update_utc_diff(r);

	ttime = mktime( &tm );
	*timestamp = ttime - r->utc_diff;
#endif

	ttime = mktime( &tm );
	*timestamp = ttime;
	//AIP_LOGI("timestamp:%s ", ctime(timestamp));

    return 0;
}

static int parse_nmea_reader_update_time( NmeaReader*  r, Token  tok )
{
    time_t timestamp = 0;
    int ret = parse_nmea_reader_get_timestamp( r, tok, &timestamp);
    if (0 == ret)
        r->fix.timestamp = (long long)timestamp * 1000;
    return ret;
}

static int parse_nmea_reader_update_cdate( NmeaReader*  r, Token  tok_d, Token tok_m, Token tok_y )
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

static int parse_nmea_reader_update_date( NmeaReader*  r, Token  date, Token  mtime )
{
    Token  tok = date;
    int day;
	int mon;
	int year;

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

    return parse_nmea_reader_update_time( r, mtime );
}


static double convert_from_hhmm( Token  tok )
{
    double  val     = str2float(tok.p, tok.end);
    int     degrees = (int)(floor(val) / 100);
    double  minutes = val - degrees*100.;
    double  dcoord  = degrees + minutes / 60.0;
	
    return dcoord;
}


static int parse_nmea_reader_update_latlong( NmeaReader*  r,
                            Token        latitude,
                            char         latitudeHemi,
                            Token        longitude,
                            char         longitudeHemi )
{
    double lat;
	double lon;
    Token tok;

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


static int parse_nmea_reader_update_altitude( NmeaReader*  r,
                             Token        altitude,
                             Token        units )
{
    double  alt;
    Token tok = altitude;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags |= GPS_LOCATION_HAS_ALTITUDE;
    r->fix.altitude = str2float(tok.p, tok.end);
	
    return 0;
}

static int parse_nmea_reader_update_accuracy( NmeaReader* r, Token accuracy )
{
    double  acc;
    Token tok = accuracy;

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

static int parse_nmea_reader_update_bearing( NmeaReader* r, Token bearing )
{
    double alt;
    Token tok = bearing;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags |= GPS_LOCATION_HAS_BEARING;
    r->fix.bearing  = str2float(tok.p, tok.end);
	
    return 0;
}


static int parse_nmea_reader_update_speed( NmeaReader* r, Token speed )
{
    double  alt;
    Token tok = speed;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags |= GPS_LOCATION_HAS_SPEED;
    r->fix.speed  = str2float(tok.p, tok.end);
    r->fix.speed  *= 0.514444;    // Speed Unit : knot to m/sec
    return 0;
}

static int parse_nmea_reader_update_timemap( NmeaReader* r, Token systime_tok, Token timestamp_tok)
{
    int ret;
    time_t timestamp;

    if ( systime_tok.p >= systime_tok.end ||
         timestamp_tok.p >= timestamp_tok.end)
    {
        r->timemap.valid = 0;
        return -1;
    }

    ret = parse_nmea_reader_get_timestamp(r, timestamp_tok, &timestamp);
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


static void parse_nmea_reader( NmeaReader* r )
{
   /* we received a complete sentence, now parse it to generate
    * a new GPS fix...
    */
    NmeaTokenizer  tzer[1];
    Token tok;

	char *ptrSentence;		
	
    if (r->pos < 9) {
         return;
    }

    if (gps_state->callbacks.nmea_cb) {
        struct timeval tv;
        unsigned long long mytimems;
        gettimeofday(&tv,NULL);
        mytimems = tv.tv_sec * 1000 + tv.tv_usec / 1000;
        //gps_state->callbacks.nmea_cb(mytimems, r->in, r->pos); 	// Modify_Add
		//gps_state->callbacks.nmea_cb(mytimems, r->in, r->pos +1);	// Modify_Add 
		gps_state->callbacks.nmea_cb(mytimems, r->in, r->pos); 		
    }

	parse_nmea_tokenizer_init(tzer, r->in, r->in + r->pos);

    tok = parse_nmea_tokenizer_get(tzer, 0);
	
	ptrSentence = tok.p;
	
	if( !strchr(ptrSentence,'*') ) {		
		return;		
	} 	
	
	if( isValid_nmea_cmd_checksum(ptrSentence)  <  0 ) {		
		return;
	}		
	
    tok.p += 2;

    if ( !memcmp(tok.p, "GGA", 3) ) {			
		
		gnss_GPS_satellite_used_cnt = 0;
		gnss_Other_satellite_used_cnt = 0;	
		
        Token  tok_fixstaus		= parse_nmea_tokenizer_get(tzer,6);

        if (tok_fixstaus.p[0] > '0') {
			
			Token  tok_time          = parse_nmea_tokenizer_get(tzer,1);
			Token  tok_latitude      = parse_nmea_tokenizer_get(tzer,2);
			Token  tok_latitudeHemi  = parse_nmea_tokenizer_get(tzer,3);
			Token  tok_longitude     = parse_nmea_tokenizer_get(tzer,4);
			Token  tok_longitudeHemi = parse_nmea_tokenizer_get(tzer,5);	
			Token  tok_altitude		= parse_nmea_tokenizer_get(tzer,9);
			Token  tok_altitudeUnits = parse_nmea_tokenizer_get(tzer,10);
						
			parse_nmea_reader_update_latlong(r, tok_latitude,
                                        tok_latitudeHemi.p[0],
                                        tok_longitude,
                                        tok_longitudeHemi.p[0]);	

			parse_nmea_reader_update_time(r, tok_time);
			parse_nmea_reader_update_altitude(r, tok_altitude, tok_altitudeUnits);
        } 
    } 	
	else if ( !memcmp(tok.p, "GSA", 3) ) {		
	
		Token  tok_prn;
		int i, prn, mod32_value; 
						
        Token  tok_fixStatus   = parse_nmea_tokenizer_get(tzer, 2);  		
		Token  tok_accuracy      = parse_nmea_tokenizer_get(tzer, 15);	//15 ==> PDOP	
		
		parse_nmea_reader_update_accuracy(r, tok_accuracy);

		if (tok_fixStatus.p[0] != '\0' ) {	// && tok_fixStatus.p[0] != '1') {  
				
			if( !memcmp(ptrSentence, "GP", 2) ) { // GPGSA
		
				for (i = 3; i <= 14; ++i) {					
					tok_prn  = parse_nmea_tokenizer_get(tzer, i);
					prn = str2int(tok_prn.p, tok_prn.end);	
					if( prn >= 1 ) {			
						gnss_GPS_used_prn_buf[gnss_GPS_satellite_used_cnt++] = prn;				
					}
				}
			}
			else {	// Other : BDGSA
			
				for (i = 3; i <= 14; ++i) {					
					tok_prn  = parse_nmea_tokenizer_get(tzer, i);
					prn = str2int(tok_prn.p, tok_prn.end);	
					if( prn >= 1 ) {			
						gnss_Other_used_prn_buf[gnss_Other_satellite_used_cnt++] = prn;				
					}
				}
			}
			
		} else {
			
			if (r->gsa_fixed == 1) {				
				r->sv_status_changed = 1;
				r->gsa_fixed = 0;
			}
        }	
    } else if ( !memcmp(tok.p, "GSV", 3) ) {        	
		
		int prn = 0;
		int i;
		int j;
		int num;
		int iTotal_Sentence_num;
		int iCurrent_Sentence_num;
		int SV_InView_num_GNSS;

		Token  tok_Total_Message_num   = parse_nmea_tokenizer_get(tzer, 1);		       
    	Token  tok_Current_Message_num	= parse_nmea_tokenizer_get(tzer, 2);    	   	
    	Token  tok_SV_InView_num  = parse_nmea_tokenizer_get(tzer, 3); 
		
		iTotal_Sentence_num = str2int(tok_Total_Message_num.p, tok_Total_Message_num.end); 
		iCurrent_Sentence_num = str2int(tok_Current_Message_num.p, tok_Current_Message_num.end); 	
		
		if( !memcmp(ptrSentence, "GP", 2) ) { 
			
			SV_InView_num_GNSS = str2int(tok_SV_InView_num.p, tok_SV_InView_num.end);
			num = (iCurrent_Sentence_num -1) *4;			
			
			if( (SV_InView_num_GNSS <= 0)  ||  (num < 0)  ) {
				return; 
			}
			if(SV_InView_num_GNSS > GPS_MAX_SVS) {				
				SV_InView_num_GNSS = GPS_MAX_SVS;			
			}						
			
			if (iCurrent_Sentence_num == 1) {				
				r->sv_status_changed = 0;				
				r->gnss_sv_info.num_svs = 0;
			}	
			
			Token  tok_prn[4];
			Token  tok_elevation[4];
			Token  tok_azimuth[4];
			Token  tok_snr[4];	
			
			i=0;	
						
			while( (i < 4)  &&  ((i+num) < SV_InView_num_GNSS)  &&  ((i+num) < GNSS_MAX_SVS) ) {	
				
				tok_prn[i]			= parse_nmea_tokenizer_get(tzer, i*4 + 4);
				tok_elevation[i]	= parse_nmea_tokenizer_get(tzer, i*4 + 5);
				tok_azimuth[i]		= parse_nmea_tokenizer_get(tzer, i*4 + 6);
				tok_snr[i]			= parse_nmea_tokenizer_get(tzer, i*4 + 7);          		
						
				prn = str2int(tok_prn[i].p, tok_prn[i].end);
				r->gnss_sv_info.gnss_sv_list[i+num].svid 			= (int16_t)prn;
				r->gnss_sv_info.gnss_sv_list[i+num].c_n0_dbhz 		= str2float(tok_snr[i].p, tok_snr[i].end); 
				r->gnss_sv_info.gnss_sv_list[i+num].elevation		= str2float(tok_elevation[i].p, tok_elevation[i].end);
				r->gnss_sv_info.gnss_sv_list[i+num].azimuth			= str2float(tok_azimuth[i].p, tok_azimuth[i].end);								
				r->gnss_sv_info.gnss_sv_list[i+num].flags			= GNSS_SV_FLAGS_NONE;
				
				for(j=0; j<gnss_GPS_satellite_used_cnt; j++) {
					if(prn == gnss_GPS_used_prn_buf[j]){
						r->gnss_sv_info.gnss_sv_list[i+num].flags	= GNSS_SV_FLAGS_USED_IN_FIX;
						break;
					}
				}	
										
				if(prn <= 32 
						||
					(prn >= 33  &&  prn <= 55) 
						||
					(prn >= 120  &&  prn <= 140) 
				) {					
					if(prn <= 32)
						r->gnss_sv_info.gnss_sv_list[i+num].constellation = GNSS_CONSTELLATION_GPS;						 	
					else
						r->gnss_sv_info.gnss_sv_list[i+num].constellation = GNSS_CONSTELLATION_SBAS;					
				} else if(prn >= 193  &&  prn <= 200) { 					
					r->gnss_sv_info.gnss_sv_list[i+num].constellation = GNSS_CONSTELLATION_QZSS;					
				} else {												
					r->gnss_sv_info.gnss_sv_list[i+num].constellation 	= GNSS_CONSTELLATION_UNKNOWN;				
				}				
				i++;
			}	
			
			if( iCurrent_Sentence_num == iTotal_Sentence_num ) {
				
				r->gnss_sv_info.num_svs = SV_InView_num_GNSS;				
				r->gnss_sv_info.size = sizeof(GnssSvStatus);				
				r->sv_status_changed = 1;

				D("SV_InView_num_GNSS: %d ", SV_InView_num_GNSS);
			}	
					
		}
		else {	
		
			SV_InView_num_GNSS = str2int(tok_SV_InView_num.p, tok_SV_InView_num.end);
			num = (iCurrent_Sentence_num -1) *4;			
			
			if( (SV_InView_num_GNSS <= 0)  ||  (num < 0)  ) {
				return; 
			}
			if(SV_InView_num_GNSS > GPS_MAX_SVS) {				
				SV_InView_num_GNSS = GPS_MAX_SVS;			
			}						
			
			Token  tok_prn[4];
			Token  tok_elevation[4];
			Token  tok_azimuth[4];
			Token  tok_snr[4];	
			
			i=0;	
						
			while( (i < 4)  &&  ((i+num) < SV_InView_num_GNSS)  &&  ((i+num) < GNSS_MAX_SVS) ) {	
				
				tok_prn[i]			= parse_nmea_tokenizer_get(tzer, i*4 + 4);
				tok_elevation[i]	= parse_nmea_tokenizer_get(tzer, i*4 + 5);
				tok_azimuth[i]		= parse_nmea_tokenizer_get(tzer, i*4 + 6);
				tok_snr[i]			= parse_nmea_tokenizer_get(tzer, i*4 + 7);          		
						
				prn = str2int(tok_prn[i].p, tok_prn[i].end);
				r->gnss_sv_info.gnss_sv_list[i+num +r->gnss_sv_info.num_svs].svid 			= (int16_t)prn;
				r->gnss_sv_info.gnss_sv_list[i+num +r->gnss_sv_info.num_svs].c_n0_dbhz 		= str2float(tok_snr[i].p, tok_snr[i].end); 
				r->gnss_sv_info.gnss_sv_list[i+num +r->gnss_sv_info.num_svs].elevation		= str2float(tok_elevation[i].p, tok_elevation[i].end);
				r->gnss_sv_info.gnss_sv_list[i+num +r->gnss_sv_info.num_svs].azimuth		= str2float(tok_azimuth[i].p, tok_azimuth[i].end);								
				r->gnss_sv_info.gnss_sv_list[i+num +r->gnss_sv_info.num_svs].flags			= GNSS_SV_FLAGS_NONE;
				
				for(j=0; j<gnss_Other_satellite_used_cnt; j++) {
					if(prn == gnss_Other_used_prn_buf[j]){
						r->gnss_sv_info.gnss_sv_list[i+num +r->gnss_sv_info.num_svs].flags	= GNSS_SV_FLAGS_USED_IN_FIX;
						break;
					}
				}	
										
				r->gnss_sv_info.gnss_sv_list[i+num +r->gnss_sv_info.num_svs].constellation = GNSS_CONSTELLATION_BEIDOU;	
				
				i++;
			}	
			
			if( iCurrent_Sentence_num == iTotal_Sentence_num ) {				
			
				r->gnss_sv_info.num_svs += SV_InView_num_GNSS;				
				r->gnss_sv_info.size = sizeof(GnssSvStatus);				
				r->sv_status_changed = 1;
			}
		}	
	
    } else if ( !memcmp(tok.p, "VTG", 3) ) {	
        Token tok_fixStatus = parse_nmea_tokenizer_get(tzer,9);
        if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != 'N')
        {
            Token  tok_bearing = parse_nmea_tokenizer_get(tzer,1);
            Token  tok_speed = parse_nmea_tokenizer_get(tzer,5);
            parse_nmea_reader_update_bearing( r, tok_bearing );
            parse_nmea_reader_update_speed  ( r, tok_speed );
        }
	} else if ( !memcmp(tok.p, "RMC", 3) ) {
		
        Token  tok_fixStatus = parse_nmea_tokenizer_get(tzer,2);	
			
		if (tok_fixStatus.p[0] == 'A') {

			Token  tok_time = parse_nmea_tokenizer_get(tzer,1);
			Token  tok_latitude = parse_nmea_tokenizer_get(tzer,3);
			Token  tok_latitudeHemi = parse_nmea_tokenizer_get(tzer,4);
			Token  tok_longitude = parse_nmea_tokenizer_get(tzer,5);
			Token  tok_longitudeHemi = parse_nmea_tokenizer_get(tzer,6);
			Token  tok_speed = parse_nmea_tokenizer_get(tzer,7);
			Token  tok_bearing = parse_nmea_tokenizer_get(tzer,8);
			Token  tok_date = parse_nmea_tokenizer_get(tzer,9);
	
            parse_nmea_reader_update_latlong( r, tok_latitude,
                                           tok_latitudeHemi.p[0],
                                           tok_longitude,
                                           tok_longitudeHemi.p[0] );
										   
			parse_nmea_reader_update_speed( r, tok_speed );
			parse_nmea_reader_update_bearing( r, tok_bearing );
			parse_nmea_reader_update_date( r, tok_date, tok_time );
        }		
		if (gps_state->callbacks.location_cb) {
			gps_state->callbacks.location_cb( &gps_state->reader.fix );
			gps_state->reader.fix.flags = 0;					
		}	
		
		if (gps_state->callbacks.gnss_sv_status_cb) {
//#ifdef GPS_DEBUG
		int i;
		char buffer[512];
		char *p;
		int SV_VAlID = 0;

		p = buffer;
		for(i = 0; i < r->gnss_sv_info.num_svs; i++) {
			int in_fix = r->gnss_sv_info.gnss_sv_list[i].flags & GNSS_SV_FLAGS_USED_IN_FIX;
			if(in_fix) {
				SV_VAlID += 1;
			}
			
			p += sprintf(p, " (%c%02d,%.0f,%c)",
			r->gnss_sv_info.gnss_sv_list[i].constellation == GNSS_CONSTELLATION_GPS ? 'G' :
			r->gnss_sv_info.gnss_sv_list[i].constellation == GNSS_CONSTELLATION_BEIDOU ? 'B' :
			r->gnss_sv_info.gnss_sv_list[i].constellation == GNSS_CONSTELLATION_GLONASS ? 'L' : '?',
			r->gnss_sv_info.gnss_sv_list[i].svid,
			r->gnss_sv_info.gnss_sv_list[i].c_n0_dbhz,
			in_fix ? 'U' : '-');
		
			if(i % 8 == 7) {
				p += sprintf(p, "\n\t");
			}
		}
		if(p - buffer) {
			AIP_LOGI("sv_list[] = %s", buffer);
			D("SV_VAlID = %d", SV_VAlID);
		}
//#endif
			gps_state->callbacks.gnss_sv_status_cb( &gps_state->reader.gnss_sv_info );
			gps_state->reader.sv_status_changed = 0;				
		}
		
		gnss_GPS_satellite_used_cnt = 0;
		gnss_Other_satellite_used_cnt = 0;
    } 
	

	else {
        tok.p -= 2;
    }
	
    if (!gps_state->first_fix &&
        gps_state->init == STATE_INIT &&
        r->fix.flags & GPS_LOCATION_HAS_LAT_LONG) {

        gps_state->first_fix = 1;
    }
}

static void parse_nmea_reader_addc( NmeaReader* r, int  c )
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
        gps_lock_fix(gps_state);
		//r->in[r->pos + 1] = 0; 	// Modify_Add
		r->in[r->pos - 1] = '\n';
		r->in[r->pos] = 0;
        parse_nmea_reader( r );
        gps_unlock_fix(gps_state);
        r->pos = 0;
    }
}

enum {
    CMD_QUIT   = 0,
    CMD_START = 1,
    CMD_STOP   = 2,
    CMD_REOPEN = 3
};


static void gps_update_fix_freq(GpsState *s, int fix_freq)
{

  s->fix_freq = fix_freq;
  
  return;
}


static void gps_done( GpsState*  s )
{
    // tell the thread to quit, and wait for it
    char   cmd = CMD_QUIT;
    void*  dummy;
    int ret;

    do {
		ret=write( s->control[0], &cmd, 1 );
	}while (ret < 0 && errno == EINTR);

    AIP_LOGI("gps waiting for command thread to stop");
	gps_sleep(s);

    pthread_join(s->thread, &dummy);

    /* Timer thread depends on this state check */
    s->init = STATE_QUIT;
    s->fix_freq = -1;

    // close the control socket pair
    close( s->control[0] );
	s->control[0] = -1;
    close( s->control[1] );
	s->control[1] = -1;

    sem_destroy(&s->fix_sem);
	g_gpscallback = 0;

    memset(s, 0, sizeof(*s));

}


static void gps_wakeup(GpsState *state)
{
	gps_lock_fix(state);

	gps_opentty(state);
	
	gps_unlock_fix(state);
	
}

static void gps_sleep(GpsState *state)
{	
	gps_lock_fix(state);
	
	started = 0;
		
	gps_closetty(state);
	gps_unlock_fix(state);
}

/*****************************************************************************
 Prototype    : gps_start
 Description  : start Gps
 Input        : GpsState*  s  
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/6/01
    Author       : Kang
    Modification : Created function

*****************************************************************************/
static void gps_start( GpsState*  s )
{
    char  cmd = CMD_START;
    int   ret;
	AIP_LOGI("%s: called", __FUNCTION__ );

	gps_lock_fix(s);
	lastcmd = CMD_START;
	gps_unlock_fix(s);


    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1)
        AIP_LOGI("%s: could not send CMD_START command: ret=%d: %s", __FUNCTION__, ret, strerror(errno));
}

/*****************************************************************************
 Prototype    : gps_stop
 Description  : stop Gps
 Input        : GpsState*  s  
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/6/01
    Author       : Kang
    Modification : Created function

*****************************************************************************/
static void gps_stop( GpsState*  s )
{
    char  cmd = CMD_STOP;
    int   ret;

	//AIP_LOGI("%s", __FUNCTION__);
	AIP_LOGI("(%s)  tid %d, pid %d", __FUNCTION__, gettid(),getpid());

	gps_lock_fix(s);
	lastcmd = CMD_STOP;
	gps_unlock_fix(s);

    do
	{
		AIP_LOGI("try %s", __FUNCTION__);
		ret=write( s->control[0], &cmd, 1 );
		if(ret < 0)
		{
			AIP_LOGE("write control socket error %s", strerror(errno));
			sleep(1);
		}
	}
    while (ret < 0 && errno == EINTR);

    if (ret != 1)
        AIP_LOGI("%s: could not send CMD_STOP command: ret=%d: %s",
          __FUNCTION__, ret, strerror(errno));
}

static void gps_reopen( GpsState*  s )
{
    char  cmd = CMD_REOPEN;
    int   ret;
	
	AIP_LOGI("(%s)  tid %d, pid %d", __FUNCTION__, gettid(),getpid());

	gps_lock_fix(s);
	lastcmd = CMD_REOPEN;
	gps_unlock_fix(s);


    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1)
        AIP_LOGI("%s: could not send CMD_REOPEN command: ret=%d: %s", __FUNCTION__, ret, strerror(errno));
}


/*****************************************************************************
 Prototype    : epoll_register
 Description  : epoll register
 Input        : int  epoll_fd  
                int  fd        
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/6/01
    Author       : Kang
    Modification : Created function

*****************************************************************************/
static int epoll_register( int  epoll_fd, int  fd )
{
    struct epoll_event  ev;
    int ret;
	int flags;

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

/*****************************************************************************
 Prototype    : epoll_deregister
 Description  : epoll unregister
 Input        : int  epoll_fd  
                int  fd        
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/6/01
    Author       : Kang
    Modification : Created function

*****************************************************************************/
static int epoll_deregister( int  epoll_fd, int  fd )
{
    int ret;
	
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_DEL, fd, NULL );
    } while (ret < 0 && errno == EINTR);
	
    return ret;
}

/*****************************************************************************
 Prototype    : gps_thread
 Description  : Gps main thread
 Input        : void*  arg
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/6/01
    Author       : Kang
    Modification : Created function

*****************************************************************************/
static void gps_thread( void*  arg )
{
    GpsState* state = (GpsState*) arg;
    gps_fd  = state->fd;
    control_fd = state->control[1];
	epoll_ctrlfd   = epoll_create(2);


	reader = &state->reader;
	parse_nmea_reader_init( reader );
	

    // register control file descriptors for polling
    epoll_register( epoll_ctrlfd, control_fd );	

    AIP_LOGI("gps thread running");
	AIP_LOGI("(%s %d ): tid %d, pid %d, uid %d, euid %d", __FUNCTION__, __LINE__,
	gettid(),getpid(),
	getuid(), geteuid());
	
	started = 0;
	state->init = STATE_INIT;

	AIP_LOGI("gps thread for loop");
	
    // now loop
    for (;;) {
        struct epoll_event   events[2];
        int ne;
		int nevents;

        
		nevents = epoll_wait( epoll_ctrlfd, events, 2, -1 );
		
        if (nevents < 0) {
            AIP_LOGE("epoll_wait() unexpected error: %s", strerror(errno));
            continue;
        }

        for (ne = 0; ne < nevents; ne++) {
			
            if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) {
                AIP_LOGE("EPOLLERR or EPOLLHUP after epoll_wait(). ");
                //goto Exit;//del

				if(state->fd != -1)
				{
					AIP_LOGI("%d  state->fd = %d", __LINE__, state->fd);
					AIP_LOGI("%d  gps_fd = %d", __LINE__, gps_fd);
					AIP_LOGI("stop read , gps device error restart");
					
					gps_stop(state);
					started = 0;
					state->init = STATE_INIT;
					gps_reopen(state);
					
				}
            }
            if ((events[ne].events & EPOLLIN) != 0) {
				
                int  fd = events[ne].data.fd;

                if (fd == control_fd)
                {
                    char  cmd = 255;
                    int   ret;
                    AIP_LOGI("gps control fd event");
                    do {
                        ret = read( fd, &cmd, 1 );
                    } while (ret < 0 && errno == EINTR);

                    if (cmd == CMD_QUIT) {						
						
                        AIP_LOGI("gps thread quitting on demand");
                        goto Exit;
                    }
                    else if (cmd == CMD_START)
					{						
                        if (!started)
						{
                            AIP_LOGI("gps thread starting  location_cb=%p", state->callbacks.location_cb);
                            started = 1;
                            state->init = STATE_START;
							
							/* handle wakeup routine*/
							gps_wakeup(state);	
							AIP_LOGI("%d  _gps_state->fd = %d", __LINE__, _gps_state->fd);
                        }
						else
						{
							AIP_LOGI("LM already start");
						}

                    }
                    else if (cmd == CMD_STOP) {
						
                        if (started)
						{
							AIP_LOGI("gps thread stoping  location_cb=%p", state->callbacks.location_cb);
                            state->init = STATE_INIT;	
						}
						else
						{
							AIP_LOGI("LM no starting, so no stoping");
							continue;
						}

						if( state->init == STATE_INIT && lastcmd == CMD_STOP && started == 1)
						{
							gps_sleep(state);
							AIP_LOGI("LM go to sleep");
						}
						
                    }	
					else if (cmd == CMD_REOPEN) {
						AIP_LOGI("%s  CMD_REOPEN ", __FUNCTION__);
						
                        if (!started)
						{
                            started = 1;
                            state->init = STATE_START;
							
							/* handle wakeup routine*/
							gps_wakeup(state);	
							AIP_LOGI("%d  _gps_state->fd = %d", __LINE__, _gps_state->fd);
                        }
						else
						{
							AIP_LOGI("%d LM already start", __LINE__);
						}
					}
                }				
				else if (fd == gps_fd)//receive data
				{					
					memset(buff, 0, sizeof(buff));

					ret = read( fd, buff, sizeof(buff) );
												
					if (ret == 0) {
						continue;
					}
					else if (ret < 0) {
						if (errno == EINTR)
							continue;
						if (errno != EWOULDBLOCK)
							AIP_LOGE("error while reading from gps daemon socket: %s:", strerror(errno));
						
						AIP_LOGE("error while reading gps device %d(%s):", errno, strerror(errno));
						gps_reopen(state);
						break;
					}

					AIP_LOGI("received %d bytes: \n%.*s", ret, ret, buff);//debug gps data
						
					for (nn = 0; nn < ret; nn++) {
						parse_nmea_reader_addc( reader, buff[nn] );	
					}						
				}
                else {
                    AIP_LOGE("epoll_wait() returned unkown fd %d , resign fd to gps_fd", fd);
					gps_fd = _gps_state->fd; //resign fd to gps_fd
                }
            }
        }
    }

Exit:
	{
		close(epoll_ctrlfd);	

		close(gps_fd);
		gps_fd = -1;
		AIP_LOGI("gps control thread destroyed");
	}

    return;
}

#ifdef CHINATSP_F202_P_8Q
static void gps_opentty_thread(void*  arg)
{
	int try = 0;	
	char tmp[CMD_MAX_LEN] = {'0'};
	FILE *fp = NULL;
	char data[ARRAY_NORMAL_SIZE] = {'0'};
	int i = 0;
	GpsState* state = (GpsState*) arg;

	AIP_LOGI("(%s %d ): tid %d, pid %d, uid %d, euid %d", __FUNCTION__, __LINE__,
	gettid(),getpid(),
	getuid(), geteuid());
	AIP_LOGI("project name: %s", CHINATSP_F202_P_8Q);
	AIP_LOGI("%s %d  state->fd = %d", __FUNCTION__, __LINE__,  state->fd); 
	do {
		i = 0;
		//try open ttyUSBn
		for(i = 0;i < 6;i++) {
			sprintf(tmp, "%s%d", GPS_READLINK_CMD, i);
			AIP_LOGI("readlink cmd str : %s ", tmp);
		
			fp = popen(tmp, "r");
			if (fp == NULL)
			{
				AIP_LOGE("popen error!");
				continue;
			}

			if (fgets(data, sizeof(data), fp) != NULL)
			{
				AIP_LOGI("readlink return: %s", data);
				sprintf(prop, "%s%d", GPS_DEVICE_TMP, i);
				AIP_LOGI("check tty name : %s ", prop);

				if((strlen(data) > 0) && (strstr(data, GPS_USB_KEYWORD)   != NULL))
				{	
					AIP_LOGI("readlink found gps tty name : %s ", prop);

					if(access(prop, R_OK)) {
						AIP_LOGI("no permissions on errno = %d, (%s) ", errno, strerror(errno));
					}
					else
					{
						state->fd = open( prop, O_RDWR | O_NOCTTY | O_NONBLOCK);
					}
					
					pclose(fp);
					if(NULL != fp) fp = NULL;
					break;
				}
				else
				{
					AIP_LOGE("could not open gps serial device %s, errno = %d, reopen  gps device , try times = %d", prop, errno, try);
				}
			}

			pclose(fp);
			if(NULL != fp) fp = NULL;
		}

		if((state->fd > 0) && (try < TRY_TIMES)) break;

		try++;
		AIP_LOGI("gps device not exits, sleep 10s ,try open %d time", try);
		sleep(10);
	} while ((state->fd < 0) && (try < TRY_TIMES)); 	

	if(state->fd < 0) {
		AIP_LOGE("gps open device failed,errno = %d, (%s) , try times = %d , return", errno, strerror(errno), TRY_TIMES);
	
		return;
	}
	else
	{
		gps_fd = state->fd;
		AIP_LOGI("%s  gps_fd = %d", __FUNCTION__, gps_fd); 
	    AIP_LOGI("gps will read from %s", prop);

	    struct termios  ios;
	    tcgetattr( state->fd, &ios );
		memset(&ios, 0 ,sizeof(ios));
		
		ios.c_cflag = GPS_DEVICE_BAUDRATE | CS8 | CLOCAL | CREAD;
		
	    ios.c_iflag = IGNPAR;
	    ios.c_oflag = 0;
	    ios.c_lflag = 0;  /* disable ECHO, ICANON, etc... */
	    tcsetattr( state->fd, TCSANOW, &ios );
		tcflush(state->fd,TCIOFLUSH);

		epoll_register( epoll_ctrlfd, state->fd );
		AIP_LOGI("%s  state->fd = %d", __FUNCTION__, state->fd); 
		AIP_LOGI("%s  _gps_state->fd = %d", __FUNCTION__, _gps_state->fd);
	}
}
#endif

#ifdef CHINATSP_S203_P_8Q
static void gps_opentty_thread(void*  arg)
{
	GpsState* state = (GpsState*) arg;

	AIP_LOGI("(%s %d ): tid %d, pid %d, uid %d, euid %d", __FUNCTION__, __LINE__,
	gettid(),getpid(),
	getuid(), geteuid());
	AIP_LOGI("project name: %s", CHINATSP_S203_P_8Q);
	AIP_LOGI("%s %d  state->fd = %d", __FUNCTION__, __LINE__,  state->fd); 

	char device[] = GPS_DEVICE_PATH;
	int try = 0;
	sprintf(prop, "%s", GPS_DEVICE_PATH);

	if(state->fd < 0 ) {
	do {
		sleep(10);
		AIP_LOGE("could not open gps serial device %s, errno = %d, (%s), reopen  gps device ", prop, errno, strerror(errno));
		state->fd = open( device, O_RDWR | O_NOCTTY | O_NONBLOCK);
		} while ((state->fd < 0) && (try < TRY_TIMES));//((state->fd < 0)&& ((errno == EINTR) || (errno == ENODEV) || (errno == ENOENT)))
	}

	if(state->fd < 0) {
		AIP_LOGE("gps open device failed,errno = %d, (%s) , try times = %d , return", errno, strerror(errno), TRY_TIMES);
		
		return;
	}
	gps_fd = state->fd;
	AIP_LOGI("%s  gps_fd = %d", __FUNCTION__, gps_fd); 
    AIP_LOGI("gps will read from %s", prop);

    struct termios  ios;
    tcgetattr( state->fd, &ios );
	memset(&ios, 0 ,sizeof(ios));
	
	ios.c_cflag = GPS_DEVICE_BAUDRATE | CS8 | CLOCAL | CREAD;
	
    ios.c_iflag = IGNPAR;
    ios.c_oflag = 0;
    ios.c_lflag = 0;  /* disable ECHO, ICANON, etc... */
    tcsetattr( state->fd, TCSANOW, &ios );
	tcflush(state->fd,TCIOFLUSH);

	epoll_register( epoll_ctrlfd, state->fd );
	AIP_LOGI("%s  state->fd = %d", __FUNCTION__, state->fd); 
	AIP_LOGI("%s  _gps_state->fd = %d", __FUNCTION__, _gps_state->fd);
}
#endif


/*****************************************************************************
 Prototype    : gps_opentty
 Description  : open gps device
 Input        : GpsState *state  
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/6/01
    Author       : Kang
    Modification : Created function

*****************************************************************************/
static int gps_opentty(GpsState *state)
{

	AIP_LOGI("called %s", __FUNCTION__);

	if((strlen(prop) <= 0) || (strlen(GPS_DEVICE_PATH) <=0))
	{
		state->fd  = -1;
		return state->fd;
	}

	if(state->fd != -1) {
		D("close gps %d", __LINE__);
		gps_closetty(state);
	}
	
#ifdef CHINATSP_S203_P_8Q
	pthread_t opentty_thread = state->callbacks.create_thread_cb("sys_opentty", gps_opentty_thread, state);
	if (!opentty_thread)
	{
		AIP_LOGE("could not create gps thread: %s", strerror(errno));
		return -1;
	}
#endif

#ifdef CHINATSP_F202_P_8Q
	pthread_t opentty_thread = state->callbacks.create_thread_cb("sys_opentty", gps_opentty_thread, state);
	if (!opentty_thread)
	{
		AIP_LOGE("could not create gps thread: %s", strerror(errno));
		return -1;
	}
#endif


	return 0;
}

/*****************************************************************************
 Prototype    : gps_closetty
 Description  : close gps device
 Input        : GpsState *s  
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/6/01
    Author       : Kang
    Modification : Created function

*****************************************************************************/
static void gps_closetty(GpsState *s)
{
	if(s->fd != -1)
	{
		AIP_LOGI("called %s", __FUNCTION__);
		// close connection to the QEMU GPS daemon
		epoll_deregister( epoll_ctrlfd, s->fd );

		close(gps_fd);
		gps_fd = -1;
		
		s->fd = -1;
	}
}

/*****************************************************************************
 Prototype    : gps_init
 Description  : gps state init
 Input        : GpsState*  state  
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/6/01
    Author       : Kang
    Modification : Created function

*****************************************************************************/
static void gps_init( GpsState*  state )
{
    int done = 0;

    struct sigevent tmr_event;

    state->init       = STATE_INIT;
    state->control[0] = -1;
    state->control[1] = -1;
    state->fd         = -1;
    state->fix_freq   = -1;
    state->first_fix  = 0;
	state->rnss_mode = GPS_POSITION_MODE_STANDALONE;


    if (sem_init(&state->fix_sem, 0, 1) != 0) {
      AIP_LOGE("gps semaphore initialization failed! errno = %d", errno);
      return;
    }

	//Create a pair of socket words for interprocess communication
    if ( socketpair( AF_LOCAL, SOCK_STREAM, 0, state->control ) < 0 ) {
        AIP_LOGE("could not create thread control socket pair: %s", strerror(errno));
        goto Fail;
    }
    	
	state->thread = state->callbacks.create_thread_cb("sys_gps", gps_thread, state);
    if (!state->thread)
	{
        AIP_LOGE("could not create gps thread: %s", strerror(errno));
        goto Fail;
    }
    state->callbacks.set_capabilities_cb(GPS_CAPABILITY_SCHEDULING);
	
	AIP_LOGI("gps state initialized");
		
    return;

Fail:
    gps_done( state );
}

static int sys_gps_init(GpsCallbacks* callbacks)
{
    GpsState*  s = _gps_state;

	//AIP_LOGI("%s %d : tid %d, pid %d, uid %d, euid %d", __FUNCTION__, __LINE__,
	//	gettid(),getpid(),
	//	getuid(), geteuid());
	
	AIP_LOGI("gps state initializing init = %d",s->init);

    s->callbacks = *callbacks;
    if (!s->init)
        gps_init(s);

	if(!g_gpscallback)
		g_gpscallback = callbacks;

    return 0;
}

static void sys_gps_cleanup(void)
{
    GpsState*  s = _gps_state;

	AIP_LOGI("%s: called", __FUNCTION__ );

    if (s->init)
        gps_done(s);
}


static int sys_gps_start()
{
    GpsState*  s = _gps_state;
	AIP_LOGI("%s: called", __FUNCTION__ );
	//D("%s: called, s->fd = %d", __FUNCTION__, s->fd);

	if(gps_checkstate(s) == -1)
	{
		AIP_LOGI("%s: called with uninitialized state !!", __FUNCTION__);
		return -1;
	}
	gps_start(s);
	AIP_LOGI("GPS_STATUS_SESSION_BEGIN");

	GPS_STATUS_CB(s->callbacks, GPS_STATUS_SESSION_BEGIN);

    return 0;
}

static int sys_gps_stop()
{
    GpsState*  s = _gps_state;

	AIP_LOGI("%s: called", __FUNCTION__ );
	//D("%s: called, s->fd = %d", __FUNCTION__, s->fd);

	if(gps_checkstate(s) == -1)
	{
		AIP_LOGI("%s: called with uninitialized state !!", __FUNCTION__);
		return -1;
	}

	//close LM first
	gps_stop(s);
	AIP_LOGI("Try to change state to init, GPS_STATUS_SESSION_END");
	//change state to INIT
	GPS_STATUS_CB(s->callbacks, GPS_STATUS_SESSION_END);

    return 0;
}


static void sys_gps_set_fix_frequency(int freq)
{
    GpsState*  s = _gps_state;

	if(gps_checkstate(s) == -1)
	{
		AIP_LOGI("%s: called with uninitialized state !!", __FUNCTION__);
		return;
	}

    s->fix_freq = (freq <= 0) ? 1 : freq;

    AIP_LOGI("(called %s)gps fix frquency set to %d secs", __FUNCTION__, freq);
}

static int sys_gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
    return 0;
}

static void sys_gps_delete_aiding_data(GpsAidingData flags)
{
	return;
}

static int sys_gps_inject_location(double latitude, double longitude, float accuracy)
{
    return 0;
}


static int sys_gps_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
									  uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time)
{
	GpsState*  s = _gps_state;

	// only standalone is supported for now.
	if (mode != GPS_POSITION_MODE_STANDALONE)
	{
		AIP_LOGI("%s: set GPS POSITION mode error! (mode:%d) ", __FUNCTION__, mode);
		AIP_LOGI("Set as standalone mode currently! ");
	}

	if (!s->init) {
		AIP_LOGI("%s: called with uninitialized state !!", __FUNCTION__);
		return -1;
	}
	s->rnss_mode = GPS_POSITION_MODE_STANDALONE;
	//s->fix_freq = min_interval/1000;
	//if (s->fix_freq ==0)
	//{
	//	s->fix_freq =1;
	//}
	
	int freq = min_interval/1000;
	if(freq == 0)
	{
		freq = 1;
	}
    //AIP_LOGI("(%s) gps fix frquency set to %d sec", __FUNCTION__, s->fix_freq);
	sys_gps_set_fix_frequency(freq);
    
    return 0;
}


static int gps_checkstate(GpsState *s)
{
    if (!s->init) {

        if(g_gpscallback)
			sys_gps_init(g_gpscallback);

		if(!s->init)
		{
			AIP_LOGE("%s: still called with uninitialized state !!", __FUNCTION__);
			return -1;
		}
    }

	return 0;
}

static const void* sys_gps_get_extension(const char* name)
{
	AIP_LOGI("%s: no GPS extension for %s is found", __FUNCTION__, name);
	
    return NULL;
}


//
// Check receive nmea CMD checksum
//
// for MC-Series , BRIAN-CMD
//
static unsigned char isValid_nmea_cmd_checksum( const char *recv_cmd )
{
	const char HexTab_Uppercase[] = "0123456789ABCDEF";
	const char HexTab_Lowercase[] = "0123456789abcdef";
	
	unsigned char sum;
	const char *p = recv_cmd;

	if (*p == '$') 
		++p;

	sum = 0;           

	while (*p)
	{
		if (*p != '*')
		{			
			sum = sum ^ (*p);
		}
		else
		{			
			if ((*(p + 1)) == HexTab_Uppercase[sum >> 4]  &&  (*(p + 2)) == HexTab_Uppercase[sum & 0x0f]
					|| 
					(*(p + 1)) == HexTab_Lowercase[sum >> 4]  &&  (*(p + 2)) == HexTab_Lowercase[sum & 0x0f] 
				)
				return 0;// success
			else
				return -1;// fail
		}
		++p;
	}
	
	return -1;// fail
}




static const GpsInterface  sysGpsInterface = {
    sizeof(GpsInterface),
    sys_gps_init,
    sys_gps_start,
    sys_gps_stop,
    sys_gps_cleanup,
    sys_gps_inject_time,
    sys_gps_inject_location,
    sys_gps_delete_aiding_data,
    sys_gps_set_position_mode,
    sys_gps_get_extension, 	
};


static void gps_dev_send(int fd, char *msg)
{
  int i;
  int n;
  int ret;

  i = strlen(msg);

  n = 0;

  AIP_LOGI("function gps_dev_send: %s", msg);
  do {

    ret = write(fd, msg + n, i - n);

    if (ret < 0 && errno == EINTR) {
      continue;
    }

    n += ret;

  } while (n < i);

  return;

}

static unsigned char gps_dev_calc_nmea_csum(char *msg)
{
  unsigned char csum = 0;
  int i;

  for (i = 1; msg[i] != '*'; ++i) {
    csum ^= msg[i];
  }

  return csum;
}


static const GpsInterface* gps_get_gps_interface(struct gps_device_t* dev)
{
    return &sysGpsInterface;
}

static int close_gps(struct hw_device_t *device)
{
	GpsState*  s = _gps_state;
	if(NULL != s)
	{
		AIP_LOGI("%s", __FUNCTION__);
		gps_closetty(s);
	}
	

	return 0;
}

static int open_gps(const struct hw_module_t* module, char const* name, struct hw_device_t** device)
{
	AIP_LOGI("%s: SYS GNSS Driver Version : %s", __FUNCTION__, SYS_GNSS_DRIVER_LIB_VERSION );
	
    struct gps_device_t *dev = malloc(sizeof(struct gps_device_t));
    memset(dev, 0, sizeof(*dev));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (struct hw_module_t*)module;
	dev->common.close = (int (*)(struct hw_device_t*))close_gps;
    dev->get_gps_interface = gps_get_gps_interface;

    *device = (struct hw_device_t*)dev;
    return 0;
}


static struct hw_module_methods_t gps_module_methods = {
    .open = open_gps
};

struct hw_module_t HAL_MODULE_INFO_SYM = {
    .tag = HARDWARE_MODULE_TAG,
    .version_major = 0,
    .version_minor = 0,
    .id = GPS_HARDWARE_MODULE_ID,
    .name = "SYS GNSS Module",
    .author = "Kang",
    .methods = &gps_module_methods,
};
