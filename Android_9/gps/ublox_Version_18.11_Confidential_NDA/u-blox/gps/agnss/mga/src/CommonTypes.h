/******************************************************************************
 * Copyright (C) u-blox AG
 * u-blox AG, Thalwil, Switzerland
 *
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose without fee is hereby granted, provided that this entire notice
 * is included in all copies of any software which is or includes a copy
 * or modification of this software and in all copies of the supporting
 * documentation for such software.
 *
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY
 * OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 ******************************************************************************
 *
 * Project: libMGA
 * Purpose: Library providing functions to help a host application to download
 *          MGA assistance data and pass it on to a u-blox GNSS receiver.
 *
 *****************************************************************************/

#ifndef __COMMON_H__
#define __COMMON_H__  //!< multiple inclusion guard

//! Standard u-blox types
/*! These internal standard types are defined here with UBX_ prepended in order to avoid
    conflicts when integrating with other languages.
*/
typedef signed char            UBX_I1;  //!< signed 1 byte integer
typedef signed short           UBX_I2;  //!< signed 2 byte integer
typedef signed int             UBX_I4;  //!< signed 4 byte integer
typedef signed long long int   UBX_I8;  //!< signed 8 byte integer
typedef unsigned char          UBX_U1;  //!< unsigned 1 byte integer
typedef unsigned short         UBX_U2;  //!< unsigned 2 byte integer
typedef unsigned int           UBX_U4;  //!< unsigned 4 byte integer
typedef unsigned long long int UBX_U8;  //!< unsigned 8 byte integer
typedef float                  UBX_R4;  //!< 4 byte floating point
typedef double                 UBX_R8;  //!< 8 byte floating point
typedef char                   UBX_CH;  //!< ASCII character

#endif //__COMMON_H__

//@}
