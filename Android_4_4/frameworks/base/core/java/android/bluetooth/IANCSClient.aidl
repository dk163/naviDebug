/*
 * Copyright (C) 2008 The Android Open Source Project
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
 */

package android.bluetooth;

import android.bluetooth.BluetoothDevice;
import android.bluetooth.IANCSClientCallback;

/**
 * System private API for Apple Notification Center Service
 *
 * {@hide}
 */
interface IANCSClient {
    boolean enable();
    boolean disable();
    int getAncsState(in BluetoothDevice device);
    boolean getNotificationAttribute(in byte[] uid, byte attrId, int maxAttrLen);
    boolean getAppAttribute(in String appId, byte attrId);
    boolean performAction(in byte[] uid, byte action);
    boolean registerCallback(in IANCSClientCallback callback);
    boolean unregisterCallback(in IANCSClientCallback callback);
}
