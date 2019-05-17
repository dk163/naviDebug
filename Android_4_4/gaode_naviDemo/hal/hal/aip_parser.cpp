//
//  autonavi_parser.c
//  tst
//
//  Created by ylee on 11/12/2017.
//  Copyright © 2017 ylee. All rights reserved.
//
#include <stdlib.h>
#include <hardware/gps.h>
#include "aip_log.h"

#include "aip_parser.h"
#include "aip_hw_if.h"


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

    // 模块输出的时间为 UTC+8 ，这里转换为 UTC，由使用方去做时区转换处理。
    // TODO.如果模组的输出改为 UTC 时间，这行代码需要删除。
    t -= (8 * 60 * 60);
    
    GpsUtcTime gpsUtcTime = (GpsUtcTime)t * 1000;
    
    // AIP_LOGD("t=%ld gpsUtcTime=%lld", t, gpsUtcTime);

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
        if (gps_an.speed >= 0) {
            gps_an.flags |= GPS_LOCATION_HAS_SPEED;
        }
        if (gps_an.bearing >= 0) {
            gps_an.flags |= GPS_LOCATION_HAS_BEARING;
        }
        if (gps_an.accuracy >= 0) {
            gps_an.flags |= GPS_LOCATION_HAS_ACCURACY;
        }

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
    
    gsv_an.num_svs = 0;
    gsv_an.ephemeris_mask = 0u;
    gsv_an.almanac_mask = 0u;
    gsv_an.used_in_fix_mask = 0u;
    for (int i=0; i<gsv_gd.num; i++) {
        if (gsv_gd.elevation[i] > 0 && gsv_gd.azimuth[i] > 0 && gsv_gd.snr[i] > 0) {
            gsv_an.sv_list[gsv_an.num_svs].size = sizeof(GpsSvInfo);
            gsv_an.sv_list[gsv_an.num_svs].prn = gsv_gd.prn[i];
            gsv_an.sv_list[gsv_an.num_svs].snr = gsv_gd.snr[i];
            gsv_an.sv_list[gsv_an.num_svs].elevation = gsv_gd.elevation[i];
            gsv_an.sv_list[gsv_an.num_svs].azimuth = gsv_gd.azimuth[i];
            
            gsv_an.ephemeris_mask |= (1u << (gsv_an.sv_list[gsv_an.num_svs].prn-1));
            gsv_an.almanac_mask |= (1u << (gsv_an.sv_list[gsv_an.num_svs].prn-1));
            gsv_an.used_in_fix_mask |= (1u << (gsv_an.sv_list[gsv_an.num_svs].prn-1));

            gsv_an.num_svs++;
        }
    }
    
    if (gsv_an.num_svs > 0) {
        return true;
    } else {
       return false;
    }
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
    acc_an.interval = acc_gd.interval;
    acc_an.x = acc_gd.x;
    acc_an.y = acc_gd.y;
    acc_an.z = acc_gd.z;

    return true;
}

bool gd2android(LocPulseInfo pul_gd, GpsPulData &pul_an) {
    pul_an.ticktime = pul_gd.ticktime;
    pul_an.interval = pul_gd.interval;

#if defined(SPEED_PWM)
    // 2019.01.30 KFan
    // 1 为了避免占空比为 0% 时容易产生信号毛刺的问题，对占空比做了5%的偏移，最终转换关系为 占空比=(车速*40)/100 + 5 
    // 2 PWM 调占空比方案，在调制端和测量端都有误差。将 PWM 频率从10KHz改为1KHz后，误差已经很小，可以接受。
    // 3 当 MCU 检测到车速异常时，会发送2%。这边统一把绝对值小于偏移值的占空比都当作速度为 0 来处理。
    // 4 倒车时模组会对占空比加负号。
    float duty_offset = 5.0f;
    float duty_speed_factor = 100.0f/40.0f;
    // AIP_LOGD("duty_speed_factor %f", duty_speed_factor);
    
    if (pul_gd.value >= duty_offset) {
        pul_an.speed = (pul_gd.value - duty_offset) * duty_speed_factor;
    } else if (pul_gd.value <= -duty_offset) {
        pul_an.speed = (pul_gd.value + duty_offset) * duty_speed_factor;
    } else {
        AIP_LOGI("unexpected duty cycle %f", pul_gd.value);
        pul_an.speed = 0;
    }
//#elif defined(SPEED_FM)

#else
    pul_an.speed = pul_gd.value;
#endif

    return true;
}

void callback(
    /* [in] */ const LOCSIGNALINFO sig)
{
    switch (sig.type) {
        case enumGPS: {
            GpsLocation location;
            if (gd2android(sig.gps, location)) {
                // AIP_LOGD("location.flags=%ld,longitude=%f,latitude=%f", location.flags, location.longitude, location.latitude);
                AipHwIf::getInstance()->m_callbacks.location_cb(&location);
            }
            break;
        }
        case enumGSV: {
            if (sig.gsv.type == 0) {
                GpsSvStatus sv;
                if (gd2android(sig.gsv, sv)) {
                    // AIP_LOGD("sv.num_svs=%d", sv.num_svs);
                    AipHwIf::getInstance()->m_callbacks.sv_status_cb(&sv);
                }
            }
            break;
        }
        case enumACCE: {
            GpsAccData acc;
            if (gd2android(sig.acc, acc)) {
                AipHwIf::getInstance()->m_callbacks.acc_cb(&acc);
            }
            break;
        }
        case enumGYRO: {
            GpsGyrData gyr;
            if (gd2android(sig.gyr, gyr)) {
                // AIP_LOGD("%ld before gyr callback", getTimestamp_ms());
                AipHwIf::getInstance()->m_callbacks.gyr_cb(&gyr);
                // AIP_LOGD("%ld after gyr callback", getTimestamp_ms());
            }
            break;
        }
        case enumPULSE: {
            GpsPulData pul;
            if (gd2android(sig.mov, pul)) {
                // AIP_LOGD("duty_cycle=%f speed=%f", sig.mov.value, pul.speed);
                AipHwIf::getInstance()->m_callbacks.pul_cb(&pul);
            }
            break;
        }
        default: {
            AIP_LOGI("switch default ");
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
    
    do {
        int lineSize = 0;
        int entireFlg = 0;
        memset((void *)line, 0, sizeof(line));
        const char *pbyRemain = splitEntireLine(bufSize, pbyBuffer, MAXLINE, line, &entireFlg, &lineSize);
        
        if (!entireFlg) {
            AIP_LOGW("incomplete line found, pbyRemain=**%s**", pbyRemain);
            ppRemain[0] = pbyRemain;
            result = (int)(bufferSize - (pbyRemain - pbyHead));
            break;
        }
        
        parse1Line(lineSize, line, reportcb);
        bufSize -= (pbyRemain - pbyBuffer);
        pbyBuffer = pbyRemain;

        if (bufSize == 0) {
            result = 0;
            break;
        }
        
        if (bufSize >= bufferSize) {
            AIP_LOGW("unexpected bufSize=%d", bufSize);
            result = 0;
            break;
        }
    } while (1);
    
    // AIP_LOGD("remainSize=%d", result);
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
        AIP_LOGE("**%s** no line feed", pbyBuffer);
        result = pbyBuffer + bufferSize;
        return result;
    }

    result = line_end + 1;

    // 截取缓冲区开头到换行为一行（可能不完整）
    size_t line_size = line_end - pbyBuffer + 1;
    if (line_size >= lineMaxSize) {
        AIP_LOGE("**%s** size out of range", pbyBuffer);
        return result;
    }
    // 拷贝一行到临时缓存
    strncpy(tempLineBuffer, pbyBuffer, line_size);
    tempLineBuffer[line_size] = '\0';

    // 查找起始符 @ 或者 # ，都不存在则不是完整行
    const char* line_start_normal = strchr(tempLineBuffer, '@');
    const char* line_start_additional = strchr(tempLineBuffer, '#');
    if (line_start_normal == NULL && line_start_additional == NULL) {
        AIP_LOGE("**%s** no line starter", tempLineBuffer);
        return result;
    }

    // 正常情况下起始符应该在行首，不在行首时，忽略起始符之前的字符
    const char* line_start = (line_start_normal != NULL ? line_start_normal : line_start_additional);
    int line_start_index = line_start - tempLineBuffer;
    if (line_start_index == 0) {
        strncpy(lineBuffer, tempLineBuffer, sizeof(tempLineBuffer));
    } else {
        AIP_LOGW("**%s** line starter index is %d", tempLineBuffer, line_start_index);
        int real_line_size = line_end - line_start + 1;
        strncpy(lineBuffer, line_start, real_line_size);
    }
    
    pnEntireFlag[0] = 1;
    pnRealSize[0] = strlen(lineBuffer);
    
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
                AIP_LOGW("GYR parse error: %s", pbyline);
                goto Exit0;
            }
            break;
        }
        
        case enumPULSE: {
            // 1187983 101 52.875000 !7A
            ret = sscanf(pbyline, "%lld %d %f",
                         &sig.mov.ticktime, &sig.mov.interval, &sig.mov.value);
            if (ret != 3) {
                AIP_LOGW("MOV parse error: %s", pbyline);
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
                AIP_LOGW("A3D parse error: %s", pbyline);
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
            // AIP_LOGD("%d %d %d %d %d %d", sig.gps.year, sig.gps.month, sig.gps.day, sig.gps.hour, sig.gps.minute, sig.gps.second);
            if (ret != 22) {
                AIP_LOGW("GPS parse error: %s", pbyline);
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
    }
Exit0:
    return result;
}

int parse1Line(
    /* [in] */ const size_t lineSize,
    /* [in] */ char *pbyLine,
    /* [in] */ PFNREPORTCALLBACK cb)
{
    int result = -1;
    
    assert(pbyLine);

    char *process = pbyLine + 4;
    if (!strncmp(pbyLine, "@GYR", 4)) {
        // AIP_LOGD("%ld pbyLine=**%s**", getTimestamp_ms(), pbyLine);
        result = parseSignal(enumGYRO, process, cb);
    } else if (!strncmp(pbyLine, "@MOV", 4)) {
        // AIP_LOGD("%ld pbyLine=**%s**", getTimestamp_ms(), pbyLine);
#ifdef OUPUT_SPEED
        result = parseSignal(enumPULSE, process, cb);
#endif
    } else if (!strncmp(pbyLine, "@A3D", 4)) {
        result = parseSignal(enumACCE, process, cb);
    } else if (!strncmp(pbyLine, "@GPS", 4)) {
        result = parseSignal(enumGPS, process, cb);
        // AIP_LOGD("%ld pbyLine=**%s**", getTimestamp_ms(), pbyLine);
    } else if (!strncmp(pbyLine, "@GSV", 4)) {
        // AIP_LOGD("%ld pbyLine=**%s**", getTimestamp_ms(), pbyLine);
        result = parseSignal(enumGSV, process, cb);
    } else if (!strncmp(pbyLine, "@W4M", 4)) {
        result = 1;
    } else if (!strncmp(pbyLine, "@TMP", 4)) {
        result = 1;
    } else if (!strncmp(pbyLine, "@Pul", 4)) {
        result = 1;
    } else if (!strncmp(pbyLine, "#$GNRMC", 7)) {
        result = 1;
    } else if (!strncmp(pbyLine, "#ID", 3)) {
        result = 1;
    } else if (!strncmp(pbyLine, "#CFG", 4)) {
        result = 1;
    } else if (!strncmp(pbyLine, "#@EXTRA", 7)) {
        result = 1;
    } else if (!strncmp(pbyLine, "##TEST", 6)) {
        result = 1;
    } else {
        result = 0;
        AIP_LOGW("unknown line received ts=%ld pbyLine=**%s**", getTimestamp_ms(), pbyLine);
    }
    
    return result;
}

