/******************************************************************************
 *
 * Copyright (C) u-blox AG
 * u-blox AG, Thalwil, Switzerland
 *
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose without fee is hereby granted, provided that this entire notice is
 * included in all copies of any software which is or includes a copy or
 * modification of this software and in all copies of the supporting
 * documentation for such software.
 *
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY OF
 * THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 ******************************************************************************
 *
 * Project: Android GNSS Driver
 *
 ******************************************************************************
 * $Id: supl_connection.h 108055 2015-12-11 14:22:01Z marcus.picasso $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/supl/supl_connection.h $
 *****************************************************************************/

/*!
  \file
  \brief  SUPL connection API

  Handles socket and SSL connections to the SUPL server. Used by
  suplSMmanager.cpp and uplsend.cpp.
*/

#ifndef __SUPL_CONNECTION_H__
#define __SUPL_CONNECTION_H__

struct SUPL_CONNECTION_STRUCT;
typedef struct SUPL_CONNECTION_STRUCT SUPL_CONNECTION;

SUPL_CONNECTION *supl_connection_init(void);

int supl_connection_connect(
    SUPL_CONNECTION *supl_connection,
    int timeout_seconds,
    const char *supl_server_name, 
    int supl_server_port, 
    int use_tls,
    const char *cert_file
);

int supl_connection_get_fd(SUPL_CONNECTION *supl_connection);

int supl_connection_read(
    SUPL_CONNECTION *supl_connection,
    unsigned char *buffer,
    long amount);

int supl_connection_write(
    SUPL_CONNECTION *supl_connection,
    const unsigned char *buffer,
    long amount);

void supl_connection_dealloc(SUPL_CONNECTION *supl_connection);

#endif /* __SUPL_CONNECTION_H__ */

// supl_connection.h
