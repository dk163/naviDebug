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
 * Project: libParser
 * Purpose: Library providing functions to parse u-blox GNSS receiver messages.
 *
 ******************************************************************************
 * $Id$
 * $HeadURL$
 *****************************************************************************/

#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "database.h"

class CProtocol
{
public:
  typedef enum { UNKNOWN = -1, UBX = 0, NMEA = 1 } PROTOCOL_t;

  CProtocol(void) {}
  virtual ~CProtocol(void) {}

public:
  virtual int Parse(const unsigned char *pBuffer, int iSize) = 0;
  virtual void Process(const unsigned char *pBuffer, int iSize, CDatabase *pDatabase) = 0;
  virtual PROTOCOL_t GetType(void) = 0;
};

#endif //__PROTOCOL_H__
