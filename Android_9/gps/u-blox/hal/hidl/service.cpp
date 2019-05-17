/*
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
 */

#include "Gnss.h"
#include <hidl/HidlTransportSupport.h>

using android::sp;

// libhwbinder:
using android::hardware::configureRpcThreadpool;
using android::hardware::joinRpcThreadpool;

// Generated HIDL files
using android::hardware::gnss::V1_0::IGnss;
using android::hardware::gnss::V1_0::implementation::Gnss;

using android::status_t;
using android::OK;

int main()
{
  android::sp<IGnss> service = new Gnss();

  configureRpcThreadpool(1, true);
  status_t status = service->registerAsService();

  if (status != OK)
  {
    ALOGE("Cannot register GNSS HAL service");
    return 1;
  }

  ALOGI("GNSS HAL Ready.");
  joinRpcThreadpool();
  // Under normal cases, execution will not reach this line.
  ALOGI("GNSS HAL failed to join thread pool.");
  return 1;
}
