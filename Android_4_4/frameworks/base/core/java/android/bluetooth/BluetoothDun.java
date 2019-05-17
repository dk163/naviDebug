package android.bluetooth;

import android.annotation.SdkConstant;
import android.annotation.SdkConstant.SdkConstantType;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.IBinder;
import android.os.RemoteException;
import android.util.Log;

import java.util.ArrayList;
import java.util.List;

public class BluetoothDun implements BluetoothProfile {
    private static final String TAG = "BluetoothDun";
    private static final boolean DBG = true;
    private static final boolean VDBG = false;

    /**
     * Intent used to broadcast the change in connection state of the Dun
     * profile.
     *
     * <p>This intent will have 4 extras:
     * <ul>
     *   <li> {@link #EXTRA_STATE} - The current state of the profile. </li>
     *   <li> {@link #EXTRA_PREVIOUS_STATE}- The previous state of the profile.</li>
     *   <li> {@link BluetoothDevice#EXTRA_DEVICE} - The remote device. </li>
     * </ul>
     *
     * <p>{@link #EXTRA_STATE} or {@link #EXTRA_PREVIOUS_STATE} can be any of
     * {@link #STATE_DISCONNECTED}, {@link #STATE_CONNECTING},
     * {@link #STATE_CONNECTED}, {@link #STATE_DISCONNECTING}.
     *
     * <p>Requires {@link android.Manifest.permission#BLUETOOTH} permission to
     * receive.
     */
    @SdkConstant(SdkConstantType.BROADCAST_INTENT_ACTION)
    public static final String ACTION_CONNECTION_STATE_CHANGED =
        "android.bluetooth.dun.profile.action.CONNECTION_STATE_CHANGED";

    private Context mContext;
    private ServiceListener mServiceListener;
    private BluetoothAdapter mAdapter;
    private IBluetoothDun mDunService;

    /**
     * Create a BluetoothDun proxy object for interacting with the local
     * Bluetooth Service which handles the Dun profile
     *
     */
    /*package*/ BluetoothDun(Context context, ServiceListener l) {
        mContext = context;
        mServiceListener = l;
        mAdapter = BluetoothAdapter.getDefaultAdapter();
        try {
            mAdapter.getBluetoothManager().registerStateChangeCallback(mStateChangeCallback);
        } catch (RemoteException re) {
            Log.w(TAG,"Unable to register BluetoothStateChangeCallback",re);
        }
        Log.d(TAG, "BluetoothDun() call bindService");
        if (!context.bindService(new Intent(IBluetoothDun.class.getName()),
                                 mConnection, 0)) {
            Log.e(TAG, "Could not bind to Bluetooth DUN Service");
        }
        Log.d(TAG, "BluetoothDun(), bindService called");
    }

    /*package*/ void close() {
        if (VDBG) log("close()");
        if (mConnection != null) {
            mContext.unbindService(mConnection);
            mConnection = null;
        }
        mServiceListener = null;
        try {
            mAdapter.getBluetoothManager().unregisterStateChangeCallback(mStateChangeCallback);
        } catch (RemoteException re) {
            Log.w(TAG,"Unable to register BluetoothStateChangeCallback",re);
        }
    }

    protected void finalize() {
        close();
    }

    private IBluetoothStateChangeCallback mStateChangeCallback = new IBluetoothStateChangeCallback.Stub() {

        @Override
        public void onBluetoothStateChange(boolean on) throws RemoteException {
            //Handle enable request to bind again.
            if (on) {
                Log.d(TAG, "onBluetoothStateChange(on) call bindService");
                if (!mContext.bindService(new Intent(IBluetoothDun.class.getName()),
                                     mConnection, 0)) {
                    Log.e(TAG, "Could not bind to Bluetooth DUN Service");
                }
                Log.d(TAG, "BluetoothDun(), bindService called");
            } else {
                if (VDBG) Log.d(TAG,"Unbinding service...");
                synchronized (mConnection) {
                    try {
                        mDunService = null;
                        mContext.unbindService(mConnection);
                    } catch (Exception re) {
                        Log.e(TAG,"",re);
                    }
                }
            }
        }
    };

    /**
     * Initiate connection to a profile of the remote bluetooth device.
     *
     * <p> This API returns false in scenarios like the profile on the
     * device is already connected or Bluetooth is not turned on.
     * When this API returns true, it is guaranteed that
     * connection state intent for the profile will be broadcasted with
     * the state. Users can get the connection state of the profile
     * from this intent.
     *
     * <p>Requires {@link android.Manifest.permission#BLUETOOTH_ADMIN}
     * permission.
     *
     * @param device Remote Bluetooth Device
     * @return false on immediate error,
     *               true otherwise
     * @hide
     */
    public boolean connect(BluetoothDevice device) {
        if (DBG) log("connect(" + device + ")");
        if (mDunService != null && isEnabled() &&
            isValidDevice(device)) {
            try {
                return mDunService.connect(device);
            } catch (RemoteException e) {
                Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
                return false;
            }
        }
        if (mDunService == null) Log.w(TAG, "Proxy not attached to service");
        return false;
    }

    /**
     * Initiate disconnection from a profile
     *
     * <p> This API will return false in scenarios like the profile on the
     * Bluetooth device is not in connected state etc. When this API returns,
     * true, it is guaranteed that the connection state change
     * intent will be broadcasted with the state. Users can get the
     * disconnection state of the profile from this intent.
     *
     * <p> If the disconnection is initiated by a remote device, the state
     * will transition from {@link #STATE_CONNECTED} to
     * {@link #STATE_DISCONNECTED}. If the disconnect is initiated by the
     * host (local) device the state will transition from
     * {@link #STATE_CONNECTED} to state {@link #STATE_DISCONNECTING} to
     * state {@link #STATE_DISCONNECTED}. The transition to
     * {@link #STATE_DISCONNECTING} can be used to distinguish between the
     * two scenarios.
     *
     * <p>Requires {@link android.Manifest.permission#BLUETOOTH_ADMIN}
     * permission.
     *
     * @param device Remote Bluetooth Device
     * @return false on immediate error,
     *               true otherwise
     * @hide
     */
    public boolean disconnect(BluetoothDevice device) {
        if (DBG) log("disconnect(" + device + ")");
        if (mDunService != null && isEnabled() &&
            isValidDevice(device)) {
            try {
                return mDunService.disconnect(device);
            } catch (RemoteException e) {
                Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
                return false;
            }
        }
        if (mDunService == null) Log.w(TAG, "Proxy not attached to service");
        return false;
    }

    /**
     * {@inheritDoc}
     */
    public List<BluetoothDevice> getConnectedDevices() {
        if (VDBG) log("getConnectedDevices()");
        if (mDunService != null && isEnabled()) {
            try {
                return mDunService.getConnectedDevices();
            } catch (RemoteException e) {
                Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
                return new ArrayList<BluetoothDevice>();
            }
        }
        if (mDunService == null) Log.w(TAG, "Proxy not attached to service");
        return new ArrayList<BluetoothDevice>();
    }

    /**
     * {@inheritDoc}
     */
    public List<BluetoothDevice> getDevicesMatchingConnectionStates(int[] states) {
        if (VDBG) log("getDevicesMatchingStates()");
        if (mDunService != null && isEnabled()) {
            try {
                return mDunService.getDevicesMatchingConnectionStates(states);
            } catch (RemoteException e) {
                Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
                return new ArrayList<BluetoothDevice>();
            }
        }
        if (mDunService == null) Log.w(TAG, "Proxy not attached to service");
        return new ArrayList<BluetoothDevice>();
    }

    /**
     * {@inheritDoc}
     */
    public int getConnectionState(BluetoothDevice device) {
        if (VDBG) log("getState(" + device + ")");
        if (mDunService != null && isEnabled()
            && isValidDevice(device)) {
            try {
                return mDunService.getConnectionState(device);
            } catch (RemoteException e) {
                Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
                return BluetoothProfile.STATE_DISCONNECTED;
            }
        }
        if (mDunService == null) Log.w(TAG, "Proxy not attached to service");
        return BluetoothProfile.STATE_DISCONNECTED;
    }

    private ServiceConnection mConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName className, IBinder service) {
            if (DBG) Log.d(TAG, "BluetoothDun Proxy object connected");
            mDunService = IBluetoothDun.Stub.asInterface(service);

            if (mServiceListener != null) {
                mServiceListener.onServiceConnected(BluetoothProfile.DUN,
                                                    BluetoothDun.this);
            }
        }
        public void onServiceDisconnected(ComponentName className) {
            if (DBG) Log.d(TAG, "BluetoothDun Proxy object disconnected");
            mDunService = null;
            if (mServiceListener != null) {
                mServiceListener.onServiceDisconnected(BluetoothProfile.DUN);
            }
        }
    };

    private boolean isEnabled() {
       if (mAdapter.getState() == BluetoothAdapter.STATE_ON) return true;
       return false;
    }

    private boolean isValidDevice(BluetoothDevice device) {
       if (device == null) return false;

       if (BluetoothAdapter.checkBluetoothAddress(device.getAddress())) return true;
       return false;
    }

    private static void log(String msg) {
      Log.d(TAG, msg);
    }
}
