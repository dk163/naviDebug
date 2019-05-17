/*
 * Copyright (C) 2008, The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package android.bluetooth;

interface IANCSClientCallback {
    void onANCSConnectionStateChanged(boolean connected, in byte[] address);
    void onANCSAppAttribute(byte commandId, in String appId,
            byte attributeId, int attributeLen, in String attribute);
    void onANCSNotificationAttribute(byte commandId,
            in byte[] notificationUid, byte attributeId, int attributeLen,
            in String attribute);
    void onANCSiOSNotificationEvent(byte eventId, byte eventFlag,
            byte categoryId, byte categoryCount, in byte[] notificationUid);
}
