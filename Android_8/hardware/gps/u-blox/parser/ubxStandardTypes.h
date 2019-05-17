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
 * $Id: ubxStandardTypes.h 79666 2014-03-19 15:45:20Z marcus.picasso $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/PRODUCTS/libParser/tags/libParser_v1.04/ubxStandardTypes.h $
 *****************************************************************************/

#ifndef __UBX_STANDARD_TYPES_H__
#define __UBX_STANDARD_TYPES_H__

// ubx specific type definitions used
#ifdef _MSC_VER
typedef __int8					I1;		//!< signed 1 byte integer
typedef __int16					I2;		//!< signed 2 byte integer
typedef __int32					I4;		//!< signed 4 byte integer
typedef __int64					I8;		//!< signed 8 byte integer
typedef unsigned __int8			U1;		//!< unsigned 1 byte integer
typedef unsigned __int16		U2;		//!< unsigned 2 byte integer
typedef unsigned __int32		U4;		//!< unsigned 4 byte integer
typedef unsigned __int64		U8;		//!< unsigned 8 byte integer
#else
#include <stdint.h>
typedef int8_t					I1; 	//!< signed 1 byte integer
typedef int16_t					I2; 	//!< signed 2 byte integer
typedef int32_t					I4; 	//!< signed 4 byte integer
typedef int64_t					I8;		//!< signed 8 byte integer
typedef uint8_t					U1; 	//!< unsigned 1 byte integer
typedef uint16_t				U2; 	//!< unsigned 2 byte integer
typedef uint32_t				U4; 	//!< unsigned 4 byte integer
typedef uint64_t				U8; 	//!< unsigned 8 byte integer
#endif
typedef char					CH; 	//!< character

#endif //__UBX_STANDARD_TYPES_H__
