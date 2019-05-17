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
 * $Id: rrlpdecod.h 79666 2014-03-19 15:45:20Z marcus.picasso $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/supl/rrlpdecod.h $
 *****************************************************************************/

/*!
  \file
  \brief  RRLP message manager interface

  Interface description of the module for managing RRLP messages sent
  from the SPL to the SET
*/

#ifndef __RRLPDECODE_H__
#define __RRLPDECODE_H__

#include "PDU.h"

///////////////////////////////////////////////////////////////////////////////
// Functions
struct PDU *rrlpDecode(unsigned char *pBuffer, int size);

#endif /* __RRLPDECODE_H__ */
