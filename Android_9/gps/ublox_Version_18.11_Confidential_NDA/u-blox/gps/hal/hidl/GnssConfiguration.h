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

#ifndef android_hardware_gnss_V1_0_GnssConfiguration_H_
#define android_hardware_gnss_V1_0_GnssConfiguration_H_

#include <android/hardware/gnss/1.0/IGnssConfiguration.h>
#include <hardware/gps.h>
#include <hidl/Status.h>

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

          using ::android::hardware::gnss::V1_0::IGnssConfiguration;
          using ::android::hardware::Return;
          using ::android::hardware::Void;
          using ::android::hardware::hidl_vec;
          using ::android::hardware::hidl_string;
          using ::android::sp;

          /*
 * Interface for passing GNSS configuration info from platform to HAL.
 */
          struct GnssConfiguration : public IGnssConfiguration
          {
            GnssConfiguration(const GnssConfigurationInterface *gnssConfigIface);

            /*
     * Methods from ::android::hardware::gnss::V1_0::IGnssConfiguration follow.
     * These declarations were generated from IGnssConfiguration.hal.
     */
            Return<bool> setSuplVersion(uint32_t version) override;
            Return<bool> setSuplMode(uint8_t mode) override;
            Return<bool> setSuplEs(bool enabled) override;
            Return<bool> setLppProfile(uint8_t lppProfile) override;
            Return<bool> setGlonassPositioningProtocol(uint8_t protocol) override;
            Return<bool> setEmergencySuplPdn(bool enable) override;
            Return<bool> setGpsLock(uint8_t lock) override;

          private:
            const GnssConfigurationInterface *mGnssConfigIface = nullptr;
          };

        } // namespace implementation
      }   // namespace V1_0
    }     // namespace gnss
  }       // namespace hardware
} // namespace android

#endif // android_hardware_gnss_V1_0_GnssConfiguration_H_
