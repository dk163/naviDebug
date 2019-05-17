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
 *****************************************************************************/
/*!
\file
\brief Extends gps.h for Android 8 to be able to use HIDL-only features

Some newly introduced features in Android 8 are HIDL-only. To enable
the usage of these features with the HIDL abstraction layer in this driver,
the gps.h structs are extended accordingly.
*/
#pragma once

#include <hardware/gps.h>

typedef struct
{
  double agc_level_db;
} GnssMeasurementExt;

typedef struct
{
  GnssData gnss_data;
  GnssMeasurementExt measurements[GNSS_MAX_MEASUREMENT];
} GnssDataExt;

typedef struct
{
  GpsLocation gps_location;
  float vertical_accuracy_meters;
  float speed_accuracy_meters_per_second;
  float bearing_accuracy_degrees;
} GpsLocationExt;
