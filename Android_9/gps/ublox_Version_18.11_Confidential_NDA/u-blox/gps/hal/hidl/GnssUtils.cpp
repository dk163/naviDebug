/*
 * NOTICE TO THE ORIGINAL WORK:
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *_________________________________________________________
 * THIS WORK WAS MODIFIED BY U-BLOX AG.
 * NOTICE TO THE DERIVATIVE WORK:
 * Copyright (C) u-blox AG, Thalwil, Switzerland
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
 */

#include "GnssUtils.h"
#include "gps_ext.h"

namespace android
{
  namespace hardware
  {
    namespace gnss
    {
      namespace V1_0
      {
        namespace implementation
        {

          using android::hardware::gnss::V1_0::GnssLocation;

          GnssLocation convertToGnssLocation(GpsLocation *location)
          {
            GnssLocation gnssLocation = {};
            GpsLocationExt *locationExt = (GpsLocationExt *)location;
            if (location != nullptr)
            {
              gnssLocation = {.gnssLocationFlags = static_cast<uint16_t>(location->flags),
                              .latitudeDegrees = location->latitude,
                              .longitudeDegrees = location->longitude,
                              .altitudeMeters = location->altitude,
                              .speedMetersPerSec = location->speed,
                              .bearingDegrees = location->bearing,
                              .horizontalAccuracyMeters = location->accuracy,
                              .verticalAccuracyMeters = locationExt->vertical_accuracy_meters,
                              .speedAccuracyMetersPerSecond =
                                locationExt->speed_accuracy_meters_per_second,
                              .bearingAccuracyDegrees = locationExt->bearing_accuracy_degrees,
                              .timestamp = location->timestamp };
            }

            return gnssLocation;
          }

          GnssLocation convertToGnssLocation(FlpLocation *location)
          {
            GnssLocation gnssLocation = {};
            if (location != nullptr)
            {
              gnssLocation = {
                // Bit mask applied (and 0's below) for same reason as above with GpsLocation
                .gnssLocationFlags = static_cast<uint16_t>(location->flags & 0x1f),
                .latitudeDegrees = location->latitude,
                .longitudeDegrees = location->longitude,
                .altitudeMeters = location->altitude,
                .speedMetersPerSec = location->speed,
                .bearingDegrees = location->bearing,
                .horizontalAccuracyMeters = location->accuracy,
                .verticalAccuracyMeters = 0,
                .speedAccuracyMetersPerSecond = 0,
                .bearingAccuracyDegrees = 0,
                .timestamp = location->timestamp
              };
            }

            return gnssLocation;
          }

        } // namespace implementation
      }   // namespace V1_0
    }     // namespace gnss
  }       // namespace hardware
} // namespace android
