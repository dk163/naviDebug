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

#ifndef ANDROID_HARDWARE_GNSS_V1_0_GNSSBATCHING_H
#define ANDROID_HARDWARE_GNSS_V1_0_GNSSBATCHING_H

#include <android/hardware/gnss/1.0/IGnssBatching.h>
#include <hardware/fused_location.h>
#include <hidl/MQDescriptor.h>
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

          using ::android::hardware::gnss::V1_0::IGnssBatching;
          using ::android::hardware::gnss::V1_0::IGnssBatchingCallback;
          using ::android::hidl::base::V1_0::IBase;
          using ::android::hardware::hidl_array;
          using ::android::hardware::hidl_memory;
          using ::android::hardware::hidl_string;
          using ::android::hardware::hidl_vec;
          using ::android::hardware::Return;
          using ::android::hardware::Void;
          using ::android::sp;

          struct GnssBatching : public IGnssBatching
          {
            GnssBatching(const FlpLocationInterface *flpLocationIface);

            // Methods from ::android::hardware::gnss::V1_0::IGnssBatching follow.
            Return<bool> init(const sp<IGnssBatchingCallback> &callback) override;
            Return<uint16_t> getBatchSize() override;
            Return<bool> start(const IGnssBatching::Options &options) override;
            Return<void> flush() override;
            Return<bool> stop() override;
            Return<void> cleanup() override;

            /*
     * Callback methods to be passed into the conventional FLP HAL by the default
     * implementation. These methods are not part of the IGnssBatching base class.
     */
            static void locationCb(int32_t locationsCount, FlpLocation **locations);
            static void acquireWakelockCb();
            static void releaseWakelockCb();
            static int32_t setThreadEventCb(ThreadEvent event);
            static void flpCapabilitiesCb(int32_t capabilities);
            static void flpStatusCb(int32_t status);

            /*
     * Holds function pointers to the callback methods.
     */
            static FlpCallbacks sFlpCb;

          private:
            const FlpLocationInterface *mFlpLocationIface = nullptr;
            static sp<IGnssBatchingCallback> sGnssBatchingCbIface;
            static bool sFlpSupportsBatching;
          };

          extern "C" IGnssBatching *HIDL_FETCH_IGnssBatching(const char *name);

        } // namespace implementation
      }   // namespace V1_0
    }     // namespace gnss
  }       // namespace hardware
} // namespace android

#endif // ANDROID_HARDWARE_GNSS_V1_0_GNSSBATCHING_H
