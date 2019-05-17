package android.bluetooth;

import android.bluetooth.BluetoothDevice;

/**
 * API for Bluetooth Dun service
 *
 * {@hide}
 */
interface IBluetoothDun {
    // Public API
    boolean connect(in BluetoothDevice device);
    boolean disconnect(in BluetoothDevice device);
    List<BluetoothDevice> getConnectedDevices();
    List<BluetoothDevice> getDevicesMatchingConnectionStates(in int[] states);
    int getConnectionState(in BluetoothDevice device);
}
