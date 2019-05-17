// IBluetoothMapEventListener.aidl
package android.bluetooth;
import android.bluetooth.BluetoothSms;

// Declare any non-default types here with import statements

interface IBluetoothMapEventListener {
    void onMapEvent(int event, in BluetoothSms sms, int result);
}
