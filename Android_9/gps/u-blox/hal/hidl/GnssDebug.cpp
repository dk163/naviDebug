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

#include <ubx_log.h>

#include "GnssDebug.h"

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

          GnssDebug::GnssDebug(const GpsDebugInterface *gpsDebugIface)
            : mGnssDebugIface(gpsDebugIface)
          {
          }

          // Methods from ::android::hardware::gnss::V1_0::IGnssDebug follow.
          Return<void> GnssDebug::getDebugData(getDebugData_cb _hidl_cb)
          {
            /*
     * This is a new interface and hence there is no way to retrieve the
     * debug data from the HAL.
     */
            DebugData data = {};

            _hidl_cb(data);

            /*
     * Log the debug data sent from the conventional Gnss HAL. This code is
     * moved here from GnssLocationProvider.
     */
            if (mGnssDebugIface)
            {
              char buffer[kMaxDebugStrLen + 1];
              size_t length = mGnssDebugIface->get_internal_state(buffer, kMaxDebugStrLen);
              length = std::max(length, kMaxDebugStrLen);
              buffer[length] = '\0';
              ALOGD("Gnss Debug Data: %s", buffer);
            }
            return Void();
          }

        } // namespace implementation
      }   // namespace V1_0
    }     // namespace gnss
  }       // namespace hardware
} // namespace android
