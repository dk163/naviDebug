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
import android.bluetooth.BluetoothFtpObject;
import android.bluetooth.IBluetoothFtpCallback;

/**
 * System private API for Bluetooth Ftp service
 *
 * {@hide}
 */
interface IBluetoothFtp {
    int getConnectionState(in BluetoothDevice device);
    boolean connect(in BluetoothDevice device);
    void disconnect(in BluetoothDevice device);
	boolean cancelSyncObject(in BluetoothFtpObject object);
	boolean browseRemoteFolder();
	boolean changeRemoteFolder(in String path);
	boolean changeLocalFolder(in String path);
	boolean deleteObject(in BluetoothFtpObject object);
	boolean pullObject(in BluetoothFtpObject object);
	boolean pushObject(in BluetoothFtpObject object);
    List<BluetoothDevice> getConnectedDevices();
    List<BluetoothDevice> getDevicesMatchingConnectionStates(in int[] states);

    boolean registerCallback(in IBluetoothFtpCallback callback);
    boolean unregisterCallback(in IBluetoothFtpCallback callback);
}
