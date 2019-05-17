//
//  autonavi_parser.c
//  tst
//
//  Created by ylee on 11/12/2017.
//  Copyright © 2017 ylee. All rights reserved.
//
#include <hardware/gps.h>

#include "autonavi_parser.h"
#include "stb_hw_if.h"

GpsUtcTime datetime2UTC(int year, int month, int day, int hour, int minute, int second) {
    struct tm t;
    t.tm_year = year - 1900;
    t.tm_mon = month - 1;
    t.tm_mday = day;
    t.tm_hour = hour;
    t.tm_min = minute;
    t.tm_sec = second;
    t.tm_isdst = 0; // 为在移动平台同样运行正确，必须设置为0或-1

    time_t utc_time = mktime(&t);
    return (GpsUtcTime)utc_time;
}

void gd2android(LOCGPSINFO gps_gd, GpsLocation &gps_an) {
    gps_an.size = sizeof(GpsLocation);
    gps_an.flags = 0;
    gps_an.latitude = gps_gd.lat;
    gps_an.longitude = gps_gd.lon;
    gps_an.altitude = gps_gd.alt;
    gps_an.speed = gps_gd.speed;
    gps_an.bearing = gps_gd.angle;
    gps_an.accuracy = gps_gd.accuracy;
    gps_an.timestamp = datetime2UTC(gps_gd.year, gps_gd.month, gps_gd.day,
        gps_gd.hour, gps_gd.minute, gps_gd.second);

    if (gps_an.longitude >= 0 && gps_an.latitude >= 0) {
        gps_an.flags |= GPS_LOCATION_HAS_LAT_LONG;
    }
    if (gps_an.altitude >= 0) {
        gps_an.flags |= GPS_LOCATION_HAS_ALTITUDE;
    }
    if (gps_an.speed >= 0) {
        gps_an.flags |= GPS_LOCATION_HAS_SPEED;
    }
    if (gps_an.bearing >= 0) {
        gps_an.flags |= GPS_LOCATION_HAS_BEARING;
    }
    if (gps_an.accuracy >= 0) {
        gps_an.flags |= GPS_LOCATION_HAS_ACCURACY;
    }
}

void gd2android(LOCGSVINFO gsv_gd, GpsSvStatus &gsv_an) {
    gsv_an.size = sizeof(GpsSvStatus);
    
    for (int i=0; i<GPS_MAX_SVS; i++) {
        gsv_an.sv_list[i].size = 0;
        gsv_an.sv_list[i].prn = 0;
        gsv_an.sv_list[i].snr = 0.0f;
        gsv_an.sv_list[i].elevation = 0.0f;
        gsv_an.sv_list[i].azimuth = 0.0f;
    }
    
    gsv_an.num_svs = gsv_gd.num;
    gsv_an.ephemeris_mask = 0u;
    gsv_an.almanac_mask = 0u;
    gsv_an.used_in_fix_mask = 0u;
    for (int i=0; i<gsv_an.num_svs; i++) {
        gsv_an.sv_list[i].size = sizeof(GpsSvInfo);
        gsv_an.sv_list[i].prn = gsv_gd.prn[i];
        gsv_an.sv_list[i].snr = gsv_gd.snr[i];
        gsv_an.sv_list[i].elevation = gsv_gd.elevation[i];
        gsv_an.sv_list[i].azimuth = gsv_gd.azimuth[i];

        gsv_an.ephemeris_mask |= (1u << (gsv_an.sv_list[i].prn-1));
        gsv_an.almanac_mask |= (1u << (gsv_an.sv_list[i].prn-1));
        gsv_an.used_in_fix_mask |= (1u << (gsv_an.sv_list[i].prn-1));
    }
}


void gd2android(LOCGYROINFO gyr_gd, GpsGyrData &gyr_an) {
    gyr_an.ticktime = gyr_gd.ticktime;
    gyr_an.axis = gyr_gd.axis;
    gyr_an.interval = gyr_gd.interval;
    gyr_an.temp = gyr_gd.temp;
    gyr_an.x = gyr_gd.x;
    gyr_an.y = gyr_gd.y;
    gyr_an.z = gyr_gd.z;
}

void gd2android(LOCACCEINFO acc_gd, GpsAccData &acc_an) {
    acc_an.ticktime = acc_gd.ticktime;
    acc_an.axis = acc_gd.axis;
    acc_an.interval = acc_gd.interval;;
    acc_an.x = acc_gd.x;
    acc_an.y = acc_gd.y;
    acc_an.z = acc_gd.z;
}

void callback(
    /* [in] */ const LOCSIGNALINFO sig)
{
    switch (sig.type) {
        case enumGPS: {
            GpsLocation location;
            gd2android(sig.gps, location);
            //printf("size=%u ", location.size);
            StbHwIf::getInstance()->m_callbacks.location_cb(&location);
            break;
        }
        case enumGSV: {
            GpsSvStatus sv;
            gd2android(sig.gsv, sv);
            //printf("@gsv,size=%u,num_svs=%d,used_in_fix_mask=%d\n", sv.size, sv.num_svs, sv.used_in_fix_mask);
            StbHwIf::getInstance()->m_callbacks.sv_status_cb(&sv);
            break;
        }
        case enumACCE: {
            GpsAccData acc;
            gd2android(sig.acc, acc);
            //printf("@a3d,ticktime=%lld,axis=%d,interval=%d,z=%0.3f,y=%0.3f,x=%0.3f\n", acc.ticktime, acc.axis, acc.interval, acc.z, acc.y, acc.x);
            StbHwIf::getInstance()->m_callbacks.acc_cb(&acc);
            break;
        }
        case enumGYRO: {
            GpsGyrData gyr;
            //gd2android(sig.gyr, gyr);
            //printf("@gyr,ticktime=%lld,axis=%d,interval=%d,temp=%0.3f,z=%0.3f,y=%0.3f,x=%0.3f\n", gyr.ticktime, gyr.axis, gyr.interval, gyr.temp, gyr.z, gyr.y, gyr.x);
            StbHwIf::getInstance()->m_callbacks.gyr_cb(&gyr);
            break;
        }
        case enumPULSE: {
            
            break;
        }
        default: {
            
        }
    }
}

/**
 *
 * @return >= 0 剩余size
 */
int parseBuffer(
    /* [in] */ const size_t bufferSize,
    /* [in] */ const char *pbyBuffer,
    /* [in] */ PFNREPORTCALLBACK reportcb,
    /* [out] */ const char **ppRemain)
{
    int result = 0;
    const char *pbyHead = pbyBuffer;
    char line[MAXLINE];
    
    size_t bufSize = bufferSize;
    
    assert(pbyBuffer && ppRemain);
    /*
    printf("==========pbyBuffer==========\n");
    printf("bufSize=%d\n", bufSize);
    printf("%s", pbyBuffer);
    printf("========== pbyBuffer end ==========\n");
    */
    do {
        int lineSize = 0;
        int entireFlg = 0;
        memset((void *)line, 0, sizeof(line));
        const char *pbyRemain = splitEntireLine(bufSize, pbyBuffer, MAXLINE, line, &entireFlg, &lineSize);
        
        //printf("%s****\n", line);
        //printf("entireFlg=%d,lineSize=%d\n\n", entireFlg, lineSize);
        if (!entireFlg) {
            ppRemain[0] = pbyRemain;
            //printf("pbyRemain=%s\n", pbyRemain);
            result = (int)(bufferSize - (pbyRemain - pbyHead));
            break;
        }
        
        parse1Line(lineSize, line, reportcb);
        bufSize -= (pbyRemain - pbyBuffer);
        pbyBuffer = pbyRemain;
        /*
        printf("***pbyBuffer\n");
        printf("bufSize=%d\n", bufSize);
        printf("%s", pbyBuffer);
        printf("*** pbyBuffer end \n");
        */
        
        if (bufSize <= 0 || bufSize >= bufferSize) {
            result = 0;
            break;
        }
    } while (1);
    
    //printf("remainSize=%d\n", result);
    return result;
}

/**
 * @GPS......................................\r\n
 * @GSV..................................................\r\n
 * @GYR......................\r\n
 * @A3D..................\r\n
 **/
const char *splitEntireLine(
    /* [in] */ const size_t bufferSize,
    /* [in] */ const char *pbyBuffer,
    /* [in] */ const size_t lineMaxSize,
    /* [out, writable] */ char *lineBuffer,
    /* [out] */ int *pnEntireFlag,
    /* [out] */ int *pnRealSize)
{
    const char *result = 0;
    const char *start = 0;
    const char *end = 0;
    int i;
    
    assert(pbyBuffer && lineBuffer && pnEntireFlag && pnRealSize);
    start = strchr(pbyBuffer, '@');
    if (!start) {
        result = pbyBuffer + bufferSize; // 不存在 @ 则直接跳过剩下的数据
        goto Exit0;
    }
    end = strchr(start, '\n');
    if (!end) {
        // 非完整行，返回start；
        pnEntireFlag[0] = 0;
        result = start;
        goto Exit0;
    } else {
        // 完整行，返回end；
        pnEntireFlag[0] = 1;
    }
    
    // loop cp
    for (i = 0; (start <= end) && (i < lineMaxSize-1); ++start, ++i) {
        lineBuffer[i] = start[0];
    }
    lineBuffer[i] = '\0';
    if (pnRealSize) {
        pnRealSize[0] = i;
    }
    
    result = end + 1;
Exit0:
    return result;
}

int parseSignal(
    /* [in] */ SIGTYPE kind,
    /* [in] */ char *pbyline,
    /* [in] */ PFNREPORTCALLBACK cb)
{
    int result = 0;
    int ret;
    LOCSIGNALINFO sig;
    
    memset(&sig, 0, sizeof(sig));
    sig.type = kind;
    
    switch (kind)
    {
        case enumGYRO: {
            // 9638400 7 100 49.044 0.050 -0.455 -0.266 !44
            ret = sscanf(pbyline, "%lld %d %d %f %f %f %f",
                         &sig.gyr.ticktime, &sig.gyr.axis, &sig.gyr.interval, &sig.gyr.temp,
                         &sig.gyr.z, &sig.gyr.y, &sig.gyr.x);
            if (ret != 7) {
                goto Exit0;
            }
            break;
        }
        case enumACCE: {
            // 9638501 7 100 0.983 0.003 0.006 !09
            ret = sscanf(pbyline, "%lld %d %d %f %f %f",
                         &sig.acc.ticktime, &sig.acc.axis, &sig.acc.interval,
                         &sig.acc.z, &sig.acc.y, &sig.acc.x);
            if (ret != 6) {
                goto Exit0;
            }
            break;
        }
        case enumGPS: {
            // 9671459 0 E 0 N 0 0 0 0.000 0.000 0.000 V 99.900 99.900 99.900 0.000 N 0 2017 1 1 2 41 11 !62
            ret = sscanf(pbyline, "%lld %d %c %*s %c %*s %d %d %lf %lf %lf %c %lf %lf %lf %lf %c %d %d %d %d %d %d %d",
                         &sig.gps.ticktime, &sig.gps.sourcetype, &sig.gps.EW, &sig.gps.NS, &sig.gps.lon, &sig.gps.lat,
                         &sig.gps.angle, &sig.gps.speed, &sig.gps.alt, &sig.gps.status,
                         &sig.gps.hdop, &sig.gps.vdop, &sig.gps.pdop, &sig.gps.accuracy,
                         &sig.gps.mode,
                         &sig.gps.satnum,
                         &sig.gps.year, &sig.gps.month, &sig.gps.day, &sig.gps.hour, &sig.gps.minute, &sig.gps.second);
            if (ret != 22) {
                goto Exit0;
            }
            break;
        }
        case enumGSV: {
            // 9638573 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 !77
            char *next = 0;
            char *token = 0;
            char *end = 0;
            
            token = strtok_r(pbyline, " ", &next);
            if (!token) {
                goto Exit0;
            }
            sig.gsv.ticktime = strtoll(token, &end, 10);
            
            token = strtok_r(next, " ", &next);
            if (!token) {
                goto Exit0;
            }
            sig.gsv.type = atoi(token);
            
            token = strtok_r(next, " ", &next);
            if (!token) {
                goto Exit0;
            }
            sig.gsv.num = atoi(token);
            
            for (int i = 0; i < 16; ++i) {
                token = strtok_r(next, " ", &next);
                if (!token) goto Exit0;
                sig.gsv.prn[i] = atoi(token);
                
                token = strtok_r(next, " ", &next);
                if (!token) goto Exit0;
                sig.gsv.elevation[i] = atoi(token);
                
                token = strtok_r(next, " ", &next);
                if (!token) goto Exit0;
                sig.gsv.azimuth[i] = atoi(token);
                
                token = strtok_r(next, " ", &next);
                if (!token) goto Exit0;
                sig.gsv.snr[i] = atoi(token);
            }
            break;
        }
        default: {
            goto Exit0;
        }
    }
    
    result = 1;
    if (cb) {
        cb(sig);
        memset((void *)&sig, 0, sizeof(sig));
    }
Exit0:
    return result;
}

int parse1Line(
    /* [in] */ const size_t lineSize,
    /* [in] */ char *pbyLine,
    /* [in] */ PFNREPORTCALLBACK cb)
{
    int result = 0;
    
    assert(pbyLine);
    
    char *process = pbyLine + 4;
    if (!strncmp(pbyLine, "@GYR", 4)) {
        parseSignal(enumGYRO, process, cb);
    } else if (!strncmp(pbyLine, "@A3D", 4)) {
        parseSignal(enumACCE, process, cb);
    } else if (!strncmp(pbyLine, "@GPS", 4)) {
        parseSignal(enumGPS, process, cb);
    } else if (!strncmp(pbyLine, "@GSV", 4)) {
        parseSignal(enumGSV, process, cb);
    } else {
        // skip current line parsing
        goto Exit1;
    }
    
Exit1:
    result = 1;
Exit0:
    return result;
}

