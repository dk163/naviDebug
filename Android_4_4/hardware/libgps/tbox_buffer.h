#ifndef __TBOX_BUFFER_H__
#define __TBOX_BUFFER_H__

#include "tbox_gps.h"

///////////////////////////////////////////////////////////////////////////////
enum { WAIT = 0, NOT_FOUND = -1};

extern void tbox_gps_parser_init(GpsState* state);
extern void tbox_gps_parser_deinit(GpsState* state);
extern int tbox_gps_buffer_get_frame_count(GpsState* state);
extern char* tbox_gps_buffer_get_frame(GpsState* state, int index);
extern void tbox_gps_buffer_append(GpsState* state, char *buffer, int size);
extern void tbox_gps_buffer_remove(GpsState* state, int size);
extern int tbox_gps_buffer_frame_checkout(GpsState* state);

#endif // __TBOX_BUFFER_H__
