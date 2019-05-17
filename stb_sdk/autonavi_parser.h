//
//  autonavi_parser.h
//  tst
//
//  Created by ylee on 11/12/2017.
//  Copyright © 2017 ylee. All rights reserved.
//

#ifndef autonavi_parser_h
#define autonavi_parser_h

//@GPS 9671459 0 E 0 N 0 0 0 0.000 0.000 0.000 V 99.900 99.900 99.900 0.000 N 0 2017 1 1 2 41 11 !62
//@GPS 9638462 0 E 0 N 0 0 0.000 0.000 2 0 0 0 0 0.000 0.000 0.000 !12
//@GYR 9638500 7 100 49.023 0.033 -0.438 -0.246 !48
//@A3D 9638501 7 100 0.983 0.003 0.006 !09
//@MOV 9638501 100 0.000 !7B
//@TMP 9638504 500 0.000 0.000 0.000 0.000 0.000 0.000 0.000 0.000 !69
//@GSV 9638573 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 !7s
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
// report callback and data structure declaration
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "common.h"

#ifdef __cplusplus
    extern "C" {
#endif

void callback(
    /* [in] */ const LOCSIGNALINFO sig);

/**
 * 解析一块缓冲区， 不完整行在解析之后保留在ppRemain（需要copy到buffer头部）
 **/
int parseBuffer(
    /* [in] */ const size_t bufferSize,
    /* [in] */ const char *pbyBuffer,
    /* [in] */ PFNREPORTCALLBACK cb,
    /* [out] */const char **ppRemain);

/**
 * pnEntrieFlag : 1表示找到一个完整行; 0表示非完整行;
 **/
const char *splitEntireLine(
    /* [in] */ const size_t bufferSize,
    /* [in] */ const char *pbyBuffer,
    /* [in] */ const size_t lineMaxSize,
    /* [out, writable] */ char *lineBuffer,
    /* [out] */ int *pnEntireFlag,
    /* [out] */ int *pnRealSize);

int parse1Line(
    /* [in] */ const size_t lineSize,
    /* [in] */ char *pbyLine,
    /* [in] */ PFNREPORTCALLBACK cb);

int parseSignal(
    /* [in] */ SIGTYPE kind,
    /* [in] */ char *pbyline,
    /* [in] */ PFNREPORTCALLBACK cb);

#ifdef __cplusplus
    };
#endif


#endif /* autonavi_parser_h */
