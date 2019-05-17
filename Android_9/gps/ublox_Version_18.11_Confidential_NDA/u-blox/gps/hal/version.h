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
 * $Id$
 * $HeadURL$
 *****************************************************************************/

#pragma once

#include <string>

#define BASE_VERSION "18.11"

// Catch cases where there was a problem defining GIT_VERSION (e.g. no .git
// directory)
#ifndef GIT_VERSION
#define GIT_VERSION ""
#endif

// determine the length of a const array at compile time
template <class T, size_t N> constexpr std::size_t length(const T (&)[N]) { return N; };

// Determine final version string
// Output different string if GIT_VERSION is empty (only null character)
static const std::string ANDROID_GNSS_DRIVER_VERSION{ length(GIT_VERSION) > 1 ? BASE_VERSION
                                                        " (" GIT_VERSION ")" :
                                                                                BASE_VERSION };
