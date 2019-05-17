//
//  common.h
//  tst
//
//  Created by ylee on 16/12/2017.
//  Copyright © 2017 ylee. All rights reserved.
//

#ifndef common_h
#define common_h

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#define     GSVSIZE     (16)
#define     MAXLINE     (1024)
#define     RBUFSIZE    (4096)

#ifdef __cplusplus
extern "C" {
#endif

typedef enum SIGTYPE {
    enumUNKNOWN = 0,
    enumGYRO = enumUNKNOWN + 1,
    enumPULSE = enumGYRO + 1,
    enumACCE = enumPULSE + 1,
    enumGPS = enumACCE + 1,
    enumGSV = enumGPS + 1,
    // if more...
    
    enumEND,
    
} SIGTYPE;

typedef struct LocGyroInfo {
    int axis;           /* 轴数 */
    float z;            /* yaw */
    float x;            /* pitch */
    float y;            /* roll */
    float temp;         /* 温度 */
    int interval;       /* 间隔 */
    long long ticktime; /* 时间戳 */
    
} LOCGYROINFO;

typedef struct LocAcceInfo {
    int axis;           /* 轴数 */
    float z;            /* yaw */
    float x;            /* pitch */
    float y;            /* roll */
    int interval;       /* 间隔 */
    long long ticktime; /* 时间戳 */
    
} LOCACCEINFO;

typedef struct LocGSVInfo {
    int type;
    int num;
    int prn[GSVSIZE];
    int elevation[GSVSIZE];
    int azimuth[GSVSIZE];
    int snr[GSVSIZE];
    long long ticktime;
    
} LOCGSVINFO;

typedef struct LocGpsInfo {
    int lon;            // * (10^6)
    int lat;            // * (10^6)
    double speed;
    double angle;
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    double accuracy;
    long long ticktime;
    char NS;
    char EW;
    double alt;
    int satnum;
    double hdop;
    double vdop;
    double pdop;
    char status;
    char mode;
    int sourcetype;
    
} LOCGPSINFO;

typedef struct LocPulseInfo {
    float value;      // unit km/h
    int interval;
    long long ticktime;
    
} LOCPULSEINFO;

typedef struct LocSignalInfo {
    SIGTYPE type;
    union {
        LOCGYROINFO gyr;
        LOCACCEINFO acc;
        LOCGSVINFO  gsv;
        LOCGPSINFO  gps;
        LOCPULSEINFO mov;
    };
} LOCSIGNALINFO;

/**
 * 解析结果回调，@mark 一次缓冲解析会有多个信号结果输出
 * 此处可以添加信号解析后 向java层转发的代码；
 */
typedef void (* PFNREPORTCALLBACK)(
    /* [in] */ const LOCSIGNALINFO signal);
    
#ifdef __cplusplus
};
#endif

#endif /* common_h */
