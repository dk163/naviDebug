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
******************************************************************************
 * $Id: $
 * $comments: $
 *****************************************************************************/
#pragma once

namespace ublox
{
  namespace Units
  {

    /**
    These functions convert all string literals to their SI based counterparts.
    E.g. 6_hours = 6*3600 (seconds)
         1_kmh = 3.6 (m/s)
    And so on
    */

    // Distances
    constexpr long double operator"" _miles(long double const distance)
    {
      return static_cast<long double>(distance * 1.852);
    }

    constexpr long double operator"" _km(long double const distance)
    {
      return static_cast<long double>(distance * 1000);
    }

    // Time
    constexpr long double operator"" _nanoseconds(long double const time)
    {
      return static_cast<long double>(time * 1e-9);
    }

    constexpr long double operator"" _microseconds(long double const time)
    {
      return static_cast<long double>(time * 1e-6);
    }

    constexpr long double operator"" _milliseconds(long double const time)
    {
      return static_cast<long double>(time * 1e-3);
    }
    constexpr long double operator"" _minutes(long double const time)
    {
      return static_cast<long double>(time * 60);
    }

    constexpr long double operator"" _hours(long double const time)
    {
      return static_cast<long double>(time * 60 * 1.0_minutes);
    }

    constexpr long double operator"" _days(long double const time)
    {
      return static_cast<long double>(time * 24 * 1.0_hours);
    }

    constexpr long double operator"" _weeks(long double const time)
    {
      return static_cast<long double>(time * 7 * 1.0_days);
    }

    /* A year having 365 days is debatable, but I found it in our code like that
     */
    constexpr long double operator"" _years(long double const time)
    {
      return static_cast<long double>(time * 365 * 1.0_days);
    }

    // Speed
    constexpr long double operator"" _kmh(long double const speed)
    {
      return static_cast<long double>(speed * (1.0_hours / 1.0_km));
    }

    constexpr long double operator"" _knts(long double const speed)
    {
      return static_cast<long double>(speed * 1.0_miles / 1.0_kmh);
    }

    constexpr long double operator"" _knts(unsigned long long const speed)
    {
      return static_cast<long double>(speed * 1.0_knts);
    }
  }
}
