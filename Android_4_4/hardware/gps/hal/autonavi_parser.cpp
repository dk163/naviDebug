//
//  autonavi_parser.c
//  tst
//
//  Created by ylee on 11/12/2017.
//  Copyright © 2017 ylee. All rights reserved.
//
#include <hardware/gps.h>
#include "ubx_log.h"

#include "autonavi_parser.h"
#include "stb_hw_if.h"


GpsUtcTime datetime2ms(int year, int month, int day, int hour, int minute, int second) {
    struct tm ti;
    ti.tm_year  = year - 1900;
    ti.tm_mon   = month - 1;
    ti.tm_mday  = day;
    ti.tm_hour  = hour;
    ti.tm_min   = minute;
    ti.tm_sec   = second;
    ti.tm_isdst = -1;
    
    time_t t = mktime(&ti);
    
    GpsUtcTime gpsUtcTime = (GpsUtcTime)t * 1000;
    
    // UBX_LOG(LCAT_DEBUG, "t=%ld gpsUtcTime=%lld", t, gpsUtcTime);

    return gpsUtcTime;
}

bool gd2android(LOCGPSINFO gps_gd, GpsLocation &gps_an) {
    if (gps_gd.status == 'A') {
        gps_an.size = sizeof(GpsLocation);
        gps_an.latitude = gps_gd.lat / 1000000.0;
        gps_an.longitude = gps_gd.lon / 1000000.0;
        gps_an.altitude = gps_gd.alt;
        gps_an.speed = gps_gd.speed / 3.6f;
        gps_an.bearing = gps_gd.angle;
        gps_an.accuracy = gps_gd.accuracy;
        gps_an.timestamp = datetime2ms(gps_gd.year, gps_gd.month, gps_gd.day, gps_gd.hour, gps_gd.minute, gps_gd.second);

        gps_an.flags = 0;
        gps_an.flags |= GPS_LOCATION_HAS_LAT_LONG;
        gps_an.flags |= GPS_LOCATION_HAS_ALTITUDE;
        gps_an.flags |= GPS_LOCATION_HAS_SPEED;
        gps_an.flags |= GPS_LOCATION_HAS_BEARING;
        gps_an.flags |= GPS_LOCATION_HAS_ACCURACY;

        return true;
    } else {
        return false;
    }
}

bool gd2android(LOCGSVINFO gsv_gd, GpsSvStatus &gsv_an) {
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

        if (gsv_an.sv_list[i].snr > 0
            && gsv_an.sv_list[i].elevation > 0
            && gsv_an.sv_list[i].azimuth > 0) {
            gsv_an.ephemeris_mask |= (1u << (gsv_an.sv_list[i].prn-1));
            gsv_an.almanac_mask |= (1u << (gsv_an.sv_list[i].prn-1));
            gsv_an.used_in_fix_mask |= (1u << (gsv_an.sv_list[i].prn-1));
        }
    }

    return true;
}


bool gd2android(LOCGYROINFO gyr_gd, GpsGyrData &gyr_an) {
    gyr_an.ticktime = gyr_gd.ticktime;
    gyr_an.axis = gyr_gd.axis;
    gyr_an.interval = gyr_gd.interval;
    gyr_an.temp = gyr_gd.temp;
    gyr_an.x = gyr_gd.x;
    gyr_an.y = gyr_gd.y;
    gyr_an.z = gyr_gd.z;

    return true;
}

bool gd2android(LOCACCEINFO acc_gd, GpsAccData &acc_an) {
    acc_an.ticktime = acc_gd.ticktime;
    acc_an.axis = acc_gd.axis;
    acc_an.interval = acc_gd.interval;;
    acc_an.x = acc_gd.x;
    acc_an.y = acc_gd.y;
    acc_an.z = acc_gd.z;

    return true;
}

bool gd2android(LocPulseInfo pul_gd, GpsPulData &pul_an) {
    pul_an.ticktime = pul_gd.ticktime;
    pul_an.interval = pul_gd.interval;;
    pul_an.speed = pul_gd.value * 5;

    return true;
}

void callback(
    /* [in] */ const LOCSIGNALINFO sig)
{
    switch (sig.type) {
        case enumGPS: {
            GpsLocation location;
            if (gd2android(sig.gps, location)) {
                StbHwIf::getInstance()->m_callbacks.location_cb(&location);
            }
            break;
        }
        case enumGSV: {
            if (sig.gsv.type == 0) {
                GpsSvStatus sv;
                if (gd2android(sig.gsv, sv)) {
                    StbHwIf::getInstance()->m_callbacks.sv_status_cb(&sv);
                }
            }
            break;
        }
        case enumACCE: {
            GpsAccData acc;
            if (gd2android(sig.acc, acc)) {
                StbHwIf::getInstance()->m_callbacks.acc_cb(&acc);
            }
            break;
        }
        case enumGYRO: {
            GpsGyrData gyr;
            if (gd2android(sig.gyr, gyr)) {
                UBX_LOG(LCAT_DEBUG, "%ld before gyr callback", getTimestamp_ms());
                StbHwIf::getInstance()->m_callbacks.gyr_cb(&gyr);
                UBX_LOG(LCAT_DEBUG, "%ld after gyr callback", getTimestamp_ms());
            }
            break;
        }
        case enumPULSE: {
            GpsPulData pul;
            if (gd2android(sig.mov, pul)) {
                // UBX_LOG(LCAT_DEBUG, "duty_cycle=%f speed=%f", sig.mov.value, pul.speed);
                StbHwIf::getInstance()->m_callbacks.pul_cb(&pul);
            }
            break;
        }
        default: {
            UBX_LOG(LCAT_INFO, "switch default ");
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
    assert(pbyBuffer && ppRemain);
    
    int result = 0;
    const char *pbyHead = pbyBuffer;
    char line[MAXLINE];
    
    size_t bufSize = bufferSize;
    // UBX_LOG(LCAT_DEBUG, "pbyBuffer=%s", pbyBuffer);
    do {
        int lineSize = 0;
        int entireFlg = 0;
        memset((void *)line, 0, sizeof(line));
        const char *pbyRemain = splitEntireLine(bufSize, pbyBuffer, MAXLINE, line, &entireFlg, &lineSize);
        
        // UBX_LOG(LCAT_DEBUG, "line=**%s**", line);
        // UBX_LOG(LCAT_DEBUG, "entireFlg=%d,lineSize=%d", entireFlg, lineSize);
        if (!entireFlg) {
            UBX_LOG(LCAT_WARNING, "incomplete line found, pbyRemain=**%s**", pbyRemain);
            ppRemain[0] = pbyRemain;
            result = (int)(bufferSize - (pbyRemain - pbyHead));
            break;
        }
        
        parse1Line(lineSize, line, reportcb);
        bufSize -= (pbyRemain - pbyBuffer);
        pbyBuffer = pbyRemain;
        // UBX_LOG(LCAT_DEBUG, "bufSize=%d pbyBuffer=**%s**", bufSize, pbyBuffer);

        if (bufSize == 0) {
            result = 0;
            break;
        }
        
        if (bufSize >= bufferSize) {
            UBX_LOG(LCAT_WARNING, "unexpected bufSize=%d", bufSize);
            result = 0;
            break;
        }
    } while (1);
    
    // UBX_LOG(LCAT_DEBUG, "remainSize=%d", result);
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
    assert(pbyBuffer && lineBuffer && pnEntireFlag && pnRealSize);

    const char *result = NULL;
    char tempLineBuffer[lineMaxSize];

    pnEntireFlag[0] = 0;
    pnRealSize[0] = 0;

    // 先找到换行符，确定一行数据。再判断这行数据是否正常（以@或者#开头）
    const char* line_end = strchr(pbyBuffer, '\n');
    if (line_end == NULL) {
        UBX_LOG(LCAT_ERROR, "**%s** no line feed", pbyBuffer);
        result = pbyBuffer + bufferSize;
        return result;
    }

    result = line_end + 1;

    // 截取缓冲区开头到换行为一行（可能不完整）
    size_t line_size = line_end - pbyBuffer + 1;
    if (line_size >= lineMaxSize) {
        UBX_LOG(LCAT_ERROR, "**%s** size out of range", pbyBuffer);
        return result;
    }
    // 拷贝一行到临时缓存
    strncpy(tempLineBuffer, pbyBuffer, line_size);
    tempLineBuffer[line_size] = '\0';

    // 查找起始符 @ 或者 # ，都不存在则不是完整行
    const char* line_start_normal = strchr(tempLineBuffer, '@');
    const char* line_start_additional = strchr(tempLineBuffer, '#');
    if (line_start_normal == NULL && line_start_additional == NULL) {
        UBX_LOG(LCAT_ERROR, "**%s** no line starter", tempLineBuffer);
        return result;
    }

    // 正常情况下起始符应该在行首，不在行首时，忽略起始符之前的字符
    const char* line_start = (line_start_normal != NULL ? line_start_normal : line_start_additional);
    int line_start_index = tempLineBuffer - line_start;
    if (line_start_index == 0) {
        strncpy(lineBuffer, tempLineBuffer, sizeof(tempLineBuffer));
    } else {
        int real_line_size = line_end - line_start + 1;
        strncpy(lineBuffer, line_start, sizeof(tempLineBuffer));
        UBX_LOG(LCAT_WARNING, "**%s** line starter index is %d", lineBuffer, line_start_index);
    }
    
    pnEntireFlag[0] = 1;
    pnRealSize[0] = strlen(lineBuffer);
    
//    if (strncmp(lineBuffer, "@GYR", 4) == 0) {
//        UBX_LOG(LCAT_DEBUG, "@GYR ts=%ld line=**%s**", getTimestamp_ms(), lineBuffer);
//    }
    
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
        
        case enumPULSE: {
            // 1187983 101 52.875000 !7A
            ret = sscanf(pbyline, "%lld %d %f",
                         &sig.mov.ticktime, &sig.mov.interval, &sig.mov.value);
            
            // UBX_LOG(LCAT_DEBUG, "pbyline=**%s**", pbyline);
            
            if (ret != 3) {
                UBX_LOG(LCAT_WARNING, "MOV parse error");
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
                UBX_LOG(LCAT_WARNING, "A3D parse error");
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
            // UBX_LOG(LCAT_DEBUG, "%d %d %d %d %d %d", sig.gps.year, sig.gps.month, sig.gps.day, sig.gps.hour, sig.gps.minute, sig.gps.second);
            if (ret != 22) {
                UBX_LOG(LCAT_WARNING, "GPS parse error");
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
        // UBX_LOG(LCAT_DEBUG, "if_cb callback type=%d ", sig.type);
        cb(sig);
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
        //UBX_LOG(LCAT_DEBUG, "%ld pbyLine=**%s**", getTimestamp_ms(), pbyLine);
        parseSignal(enumGYRO, process, cb);
    } else if (!strncmp(pbyLine, "@MOV", 4)) {
        parseSignal(enumPULSE, process, cb);
    } else if (!strncmp(pbyLine, "@A3D", 4)) {
        parseSignal(enumACCE, process, cb);
    } else if (!strncmp(pbyLine, "@GPS", 4)) {
        parseSignal(enumGPS, process, cb);
    } else if (!strncmp(pbyLine, "@GSV", 4)) {
        parseSignal(enumGSV, process, cb);
    } else if (!strncmp(pbyLine, "@W4M", 4)) {
        goto Exit1;
    } else if (!strncmp(pbyLine, "@TMP", 4)) {
        goto Exit1;
    } else if (!strncmp(pbyLine, "@Pul", 4)) {
        goto Exit1;
    } else if (!strncmp(pbyLine, "#$GNRMC", 7)) {
        goto Exit1;
    } else if (!strncmp(pbyLine, "#ID", 3)) {
        goto Exit1;
    } else if (!strncmp(pbyLine, "#CFG", 4)) {
        goto Exit1;
    } else if (!strncmp(pbyLine, "#@EXTRA", 7)) {
        goto Exit1;
    } else if (!strncmp(pbyLine, "##TEST", 6)) {
        goto Exit1;
    } else {
        UBX_LOG(LCAT_WARNING, "unknown line received ts=%ld pbyLine=**%s**", getTimestamp_ms(), pbyLine);
        goto Exit1;
    }
    
Exit1:
    result = 1;
Exit0:
    return result;
}

