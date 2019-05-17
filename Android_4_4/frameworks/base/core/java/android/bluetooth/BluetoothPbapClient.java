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

import android.annotation.SdkConstant;
import android.annotation.SdkConstant.SdkConstantType;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.os.RemoteException;
import android.util.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * The Android Bluetooth API is not finalized, and *will* change. Use at your
 * own risk.
 *
 * Public API for controlling the Bluetooth Pbap Service. This includes
 * Bluetooth Phone book Access profile. BluetoothPbapClient is a proxy object for
 * controlling the Bluetooth Pbap Service via IPC.
 *
 * Creating a BluetoothPbapClient object will create a binding with the BluetoothPbap
 * service. Users of this object should call close() when they are finished with
 * the BluetoothPbapClient, so that this proxy object can unbind from the service.
 *
 * This BluetoothPbapClient object is not immediately bound to the BluetoothPbap
 * service. Use the ServiceListener interface to obtain a notification when it
 * is bound, this is especially important if you wish to immediately call
 * methods on BluetoothPbapClient after construction.
 *
 * Android only supports one connected Bluetooth Pce at a time.
 *
 * @hide
 */
public class BluetoothPbapClient implements BluetoothProfile {

    private static final String TAG = "BluetoothPbapClient";
    private static final boolean DBG = true;
    private static final boolean VDBG = false;

    /**
     * Intent used to broadcast the change in connection state of the PBAP
     * profile.
     *
     * <p>This intent will have 3 extras:
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
        "android.bluetooth.pbap.profile.action.CONNECTION_STATE_CHANGED";
    /** There was an error trying to obtain the state */
    public static final int STATE_ERROR        = -1;
    /** No client currently connected */
    public static final int STATE_DISCONNECTED = 0;
    /** Connection attempt in progress */
    public static final int STATE_CONNECTING   = 1;
    /** Client is currently connected */
    public static final int STATE_CONNECTED    = 2;
    /** Client is currently connected and sync nothing*/
    public static final int STATE_SYNC_IDLE    = 3;
    /** Client is currently connected and sync calllog*/
    public static final int STATE_SYNC_CALLLOG = 4;
    /** Client is currently connected and sync contact*/
    public static final int STATE_SYNC_CONTACT = 5;

    public static final int RESULT_FAILURE = 0;
    public static final int RESULT_SUCCESS = 1;
    /** Connection canceled before completion. */
    public static final int RESULT_CANCELED = 2;

    /** Contact info mask*/
    /*In order to get the Formatted Name*/
    public static final int CONTACT_INFO_MASK_FN = (0x0001 << 0);
    /*In order to get the Structured Presentation of Name*/
    public static final int CONTACT_INFO_MASK_N =  (0x0001 << 1);
    /*In order to get the Telephone Number*/
    public static final int CONTACT_INFO_MASK_TEL = (0x0001 << 2);
    /*In order to get the Electronic Mail Address*/
    public static final int CONTACT_INFO_MASK_MAIL = (0x0001 << 3);
    /*In order to get the Associated Image or Photo , mybe not support*/
    public static final int CONTACT_INFO_MASK_PHOTO = (0x0001 << 4);

    /** Contact type define **/
    /*Sync contact from Phone and SIM cards*/
    public static final int CONTACT_SYNC_TYPE_ALL = 0;
    /*Sync contact from SIM cards*/
    public static final int CONTACT_SYNC_TYPE_SIM = 1;
    /*Sync contact from Phone */
    public static final int CONTACT_SYNC_TYPE_PHONE = 2;

	private static final int MESSAGE_BLUETOOTH_PBAP_STATE_CHANGED = 0x01;
	private static final int MESSAGE_BLUETOOTH_PBAP_CONTACT_COUNT_DETERMINATED = 0x02;
	private static final int MESSAGE_BLUETOOTH_PBAP_CONTACT_FETCHED = 0x03;
	private static final int MESSAGE_BLUETOOTH_PBAP_CALLLOG_COUNT_DETERMINATED = 0x04;
	private static final int MESSAGE_BLUETOOTH_PBAP_CALLLOG_FETCHED = 0x05;

	/**
	 * An event listener can implements this interface to receive the PBAP
	 * events. Such as PBAP synchronization started, finished and some
	 * Contact/CallLog item is fetched from the remote device.
	 *
	 */
	public interface EventListener {
		enum SyncState {
			SYNC_STATE_IDLE,
			/**
			 * PBAP synchronization procedure started.
			 */
			SYNC_STATE_STARTED,
			/**
			 * PBAP synchronization procedure finished.
			 */
			SYNC_STATE_FINISHED,
			/**
			 * PBAP synchornization procedure aborted.
			 */
			SYNC_STATE_ABORTED
		}

		/**
		 * This method is called when the {@linkplain SyncState state} of PBAP
		 * synchronization precedure changed.
		 *
		 * @param preState
		 *            the previous state
		 * @param state
		 *            the current state
		 */
		public void onSyncStateChanged(SyncState preState, SyncState state);

		/**
		 * This method is called when the total count of Contacts of remote
		 * device is determined.
		 *
		 * @param count
		 *            the total count of Contacts
		 */
		public void onContactItemCountDetermined(int count);

		/**
		 * This method is called when a Contact item has been fetched from
		 * remote device.
		 *
		 * @param item
		 *            The fetched Contact item.
		 */
		public void onContactItemFetched(BluetoothContact item);

		/**
		 * This method is called when the total count of CallLogs of remote
		 * device is determined.
		 *
		 * @param count
		 *            the total count of CallLog
		 */
		public void onCallLogItemCountDetermined(int count);

		/**
		 * This method is called when a CallLog item has been fetched from
		 * remote device.
		 *
		 * @param item
		 *            The fetched CallLog item.
		 */
		public void onCallLogItemFetched(BluetoothCallLog item);
	}

	private IBluetoothPbap mService;
	private final Context mContext;
	private ServiceListener mServiceListener;
	private BluetoothAdapter mAdapter;
	private List<EventListener> mPbapListeners;

	final private IBluetoothStateChangeCallback mBluetoothStateChangeCallback = new IBluetoothStateChangeCallback.Stub() {
		public void onBluetoothStateChange(boolean up) {
			if (DBG)
				Log.d(TAG, "onBluetoothStateChange: up=" + up);
			if (!up) {
				if (VDBG)
					Log.d(TAG, "[debug>>] Unbinding service...");
				doUnbind();
			} else {
				synchronized (mConnection) {
					try {
						if (mService == null) {
							if (VDBG)
								Log.d(TAG, "Binding service...");
							if (!mContext.bindService(new Intent(
									IBluetoothPbap.class.getName()),
									mConnection, 0)) {
								Log.e(TAG,
										"Could not bind to Bluetooth PBAP Service");
							}
						}
					} catch (Exception re) {
						Log.e(TAG, "", re);
					}
				}
			}
		}
	};

	final private Handler mHandler = new Handler() {
		@Override
		public void handleMessage(Message msg) {
			for (EventListener l : mPbapListeners) {
				switch (msg.what) {
				case MESSAGE_BLUETOOTH_PBAP_STATE_CHANGED:
					int p = msg.arg1;
					int c = msg.arg2;
					EventListener.SyncState preState = EventListener.SyncState
							.values()[p];
					EventListener.SyncState state = EventListener.SyncState
							.values()[c];
					l.onSyncStateChanged(preState, state);
					break;
				case MESSAGE_BLUETOOTH_PBAP_CONTACT_COUNT_DETERMINATED:
					l.onContactItemCountDetermined(msg.arg1);
					break;
				case MESSAGE_BLUETOOTH_PBAP_CONTACT_FETCHED:
					l.onContactItemFetched((BluetoothContact) msg.obj);
					break;
				case MESSAGE_BLUETOOTH_PBAP_CALLLOG_COUNT_DETERMINATED:
					l.onCallLogItemCountDetermined(msg.arg1);
					break;
				case MESSAGE_BLUETOOTH_PBAP_CALLLOG_FETCHED:
					l.onCallLogItemFetched((BluetoothCallLog) msg.obj);
					break;
				}
			}
		}
	};

    /**
     * Create a BluetoothPbapClient proxy object.
     */
    /*package*/ BluetoothPbapClient(Context context, android.bluetooth.BluetoothProfile.ServiceListener l) {
        mContext = context;
        mServiceListener = l;
        mAdapter = BluetoothAdapter.getDefaultAdapter();
        IBluetoothManager mgr = mAdapter.getBluetoothManager();
        if (mgr != null) {
            try {
                mgr.registerStateChangeCallback(mBluetoothStateChangeCallback);
            } catch (RemoteException e) {
                Log.e(TAG,"",e);
            }
        }
        if (!context.bindService(new Intent(IBluetoothPbap.class.getName()), mConnection, 0)) {
            Log.e(TAG, "Could not bind to Bluetooth Pbap Service");
        }

		mPbapListeners = new ArrayList<EventListener>();
    }

	private IBluetoothPbapCallback mPbapCallback = new IBluetoothPbapCallback.Stub() {

		@Override
		public void onSyncStateChanged(int preState, int state)
				throws RemoteException {
			Message msg = mHandler
					.obtainMessage(MESSAGE_BLUETOOTH_PBAP_STATE_CHANGED, preState,
							state, null);
			mHandler.sendMessage(msg);
		}

		@Override
		public void onContactItemCountDetermined(int count)
				throws RemoteException {
			Message msg = mHandler.obtainMessage(
					MESSAGE_BLUETOOTH_PBAP_CONTACT_COUNT_DETERMINATED, count,
					0, null);
			mHandler.sendMessage(msg);
		}

		@Override
		public void onContactItemFetched(BluetoothContact item)
				throws RemoteException {
			Message msg = mHandler.obtainMessage(
					MESSAGE_BLUETOOTH_PBAP_CONTACT_FETCHED, 0, 0, item);
			mHandler.sendMessage(msg);
		}

		@Override
		public void onCallLogItemCountDetermined(int count)
				throws RemoteException {
			Message msg = mHandler.obtainMessage(
					MESSAGE_BLUETOOTH_PBAP_CALLLOG_COUNT_DETERMINATED, count,
					0, null);
			mHandler.sendMessage(msg);
		}

		@Override
		public void onCallLogItemFetched(BluetoothCallLog item)
				throws RemoteException {
			Message msg = mHandler.obtainMessage(
					MESSAGE_BLUETOOTH_PBAP_CALLLOG_FETCHED, 0, 0, item);
			mHandler.sendMessage(msg);
		}

	};

	protected void finalize() throws Throwable {
		try {
			close();
		} finally {
			super.finalize();
		}
	}

	/**
	 * Close the connection to the backing service. Other public functions of
	 * BluetoothPbapClient will return default error results once close() has been
	 * called. Multiple invocations of close() are ok.
	 */
	public synchronized void close() {
		IBluetoothManager mgr = mAdapter.getBluetoothManager();
		if (mgr != null) {
			try {
				mgr.unregisterStateChangeCallback(mBluetoothStateChangeCallback);
			} catch (Exception e) {
				Log.e(TAG, "", e);
			}
		}

		doUnbind();
		mServiceListener = null;
		mConnection = null;
		log("[debug>>] closed:" + Thread.currentThread().getId());
	}

	private void doUnbind() {
		if (DBG)
			log("[debug>>] doUnbind:" + Thread.currentThread().getId());
				synchronized (mConnection) {
					if (mService != null) {
						try {
							mService.unregisterCallback(mPbapCallback);
								mContext.unbindService(mConnection);
                            mService = null;
						} catch (Exception re) {
							Log.e(TAG, "", re);
						}
					}
		}
	}

	/**
	 * Register a {@link EventListener} which will be notified when some
	 * PBAP event takes place.
	 *
	 * @param listener
	 *            The event listener.
	 */
	public void registerEventListener(EventListener listener) {
		mPbapListeners.add(listener);
	}

	/**
	 * Unregister a {@link EventListener}.
	 *
	 * @param listener
	 *            the event listener
	 */
	public void unregisterEventListener(EventListener listener) {
		mPbapListeners.remove(listener);
	}

	@Override
	/**
	 * {@inheritDoc}
	 */
	public List<BluetoothDevice> getConnectedDevices() {
		if (VDBG)
			log("getConnectedDevices()");
		if (mService != null && isEnabled()) {
			try {
				return mService.getConnectedDevices();
			} catch (RemoteException e) {
				Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
				return new ArrayList<BluetoothDevice>();
			}
		}
		if (mService == null)
			Log.w(TAG, "Proxy not attached to service");
		return new ArrayList<BluetoothDevice>();
	}

	@Override
	/**
	 * {@inheritDoc}
	 */
	public List<BluetoothDevice> getDevicesMatchingConnectionStates(int[] states) {
		if (VDBG)
			log("getDevicesMatchingStates()");
		if (mService != null && isEnabled()) {
			try {
				return mService.getDevicesMatchingConnectionStates(states);
			} catch (RemoteException e) {
				Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
				return new ArrayList<BluetoothDevice>();
			}
		}
		if (mService == null)
			Log.w(TAG, "Proxy not attached to service");
		return new ArrayList<BluetoothDevice>();
	}

	@Override
	/**
	 * {@inheritDoc}
	 */
	public int getConnectionState(BluetoothDevice device) {
		if (VDBG)
			log("getState(" + device + ")");
		if (mService != null && isEnabled() && isValidDevice(device)) {
			try {
				return mService.getConnectionState(device);
			} catch (RemoteException e) {
				Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
				return BluetoothProfile.STATE_DISCONNECTED;
			}
		}
		if (mService == null)
			Log.w(TAG, "Proxy not attached to service");
		return BluetoothProfile.STATE_DISCONNECTED;
	}

	/**
	 * Initiate connection to a profile of the remote bluetooth device.
	 *
	 * <p>
	 * Currently, the system supports only 1 connection to the PBAP profile. The
	 * API will automatically disconnect connected devices before connecting.
	 *
	 * <p>
	 * This API returns false in scenarios like the profile on the device is
	 * already connected or Bluetooth is not turned on. When this API returns
	 * true, it is guaranteed that connection state intent for the profile will
	 * be broadcasted with the state. Users can get the connection state of the
	 * profile from this intent.
	 *
	 * <p>
	 * Requires {@link android.Manifest.permission#BLUETOOTH_ADMIN} permission.
	 *
	 * @param device
	 *            Remote Bluetooth Device
	 * @return false on immediate error, true otherwiseconnect
	 * @hide
	 */
	public boolean connect(BluetoothDevice device) {
		if (DBG)
			log("connect(" + device + ")");
		if (mService != null && isEnabled() && isValidDevice(device)) {
			try {
				return mService.connect(device);
			} catch (RemoteException e) {
				Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
				return false;
			}
		}
		if (mService == null)
			Log.w(TAG, "Proxy not attached to service");
		return false;
	}

	/**
	 * Disconnects the current Pbap client (PCE). Currently this call blocks, it
	 * may soon be made asynchronous. Returns false if this proxy object is not
	 * currently connected to the Pbap service.
	 */
	public boolean disconnect() {
		if (DBG)
			log("disconnect()");
		if (mService != null) {
			try {
				mService.disconnect();
				return true;
			} catch (RemoteException e) {
				Log.e(TAG, e.toString());
			}
		} else {
			Log.w(TAG, "Proxy not attached to service");
			if (DBG)
				log(Log.getStackTraceString(new Throwable()));
		}
		return false;
	}

    /**
     * Check class bits for possible PBAP support.
     * This is a simple heuristic that tries to guess if a device with the
     * given class bits might support PBAP. It is not accurate for all
     * devices. It tries to err on the side of false positives.
     * @return True if this device might support PBAP.
     */
    public static boolean doesClassMatchSink(BluetoothClass btClass) {
        // TODO optimize the rule
        switch (btClass.getDeviceClass()) {
        case BluetoothClass.Device.COMPUTER_DESKTOP:
        case BluetoothClass.Device.COMPUTER_LAPTOP:
        case BluetoothClass.Device.COMPUTER_SERVER:
        case BluetoothClass.Device.COMPUTER_UNCATEGORIZED:
            return true;
        default:
            return false;
        }
    }

	/**
	 * Initials the Contact-synchronization procedure. This method will return
	 * immediatedly. If the request has been received by the remote device, the
	 * subsequent contacts will be notified througth the
	 * {@link EventListener#onContactItemFetched(BluetoothContact)} interface
	 * which is registered by {@link #registerEventListener(EventListener)}.
	 *
	 * @return true if the request has been sent to the remote device
	 *         successfully. false otherwise.
	 */
	public boolean startContactSync() {
		if (DBG)
			log("startContactSync()");
		if (mService != null && isEnabled()) {
			try {
				return mService.startContactSync(CONTACT_SYNC_TYPE_ALL);
			} catch (RemoteException e) {
				Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
				return false;
			}
		}
		if (mService == null)
			Log.w(TAG, "Proxy not attached to service");
		return false;
	}

	/**
	 * Initials the Contact-synchronization procedure. This method will return
	 * immediatedly. If the request has been received by the remote device, the
	 * subsequent contacts will be notified througth the
	 * {@link EventListener#onContactItemFetched(BluetoothContact)} interface
	 * which is registered by {@link #registerEventListener(EventListener)}.
	 * @param type
     * <ul>
     *   <li> {@link #CONTACT_SYNC_TYPE_ALL</li>
     *   <li> {@link #CONTACT_SYNC_TYPE_SIM</li>
     *   <li> {@link #CONTACT_SYNC_TYPE_PHONE</li>
     * <ul>
	 * @return true if the request has been sent to the remote device
	 *         successfully. false otherwise.
	 */
	public boolean startContactSync(int type) {
		if (DBG)
			log("startContactSync, type: " + type);
		if (mService != null && isEnabled()) {
			try {
				return mService.startContactSync(type);
			} catch (RemoteException e) {
				Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
				return false;
			}
		}
		if (mService == null)
			Log.w(TAG, "Proxy not attached to service");
		return false;
	}

	/**
	 * Initials the CallLog-synchronization procedure. This method will return
	 * immediatedly. If the request has been received by the remote device, the
	 * subsequent call logs will be notified through the
	 * {@link EventListener#onCallLogItemFetched(BluetoothCallLog)} interface
	 * which is registered by {@link #registerEventListener(EventListener)}.
	 *
	 * @return true if the request has been sent to the remote device
	 *         successfully. false otherwise.
	 */
	public boolean startCallLogSync() {
		if (DBG)
			log("startCallLogSync()");
		if (mService != null && isEnabled()) {
			try {
				return mService.startCallLogSync(0xFFFF);
			} catch (RemoteException e) {
				Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
				return false;
			}
		}
		if (mService == null)
			Log.w(TAG, "Proxy not attached to service");
		return false;
	}

	/**
	 * Initials the CallLog-synchronization procedure. This method will return
	 * immediatedly. If the request has been received by the remote device, the
	 * subsequent call logs will be notified through the
	 * {@link EventListener#onCallLogItemFetched(BluetoothCallLog)} interface
	 * which is registered by {@link #registerEventListener(EventListener)}.
	 * @param clLimit
	 *         set maxinum for calllog
	 * @return true if the request has been sent to the remote device
	 *         successfully. false otherwise.
	 */
	public boolean startCallLogSync(int clLimit) {
		if (DBG)
			log("staratCallLogSync, clLimit: " + clLimit);
		if (mService != null && isEnabled()) {
			try {
				return mService.startCallLogSync(clLimit);
			} catch (RemoteException e) {
				Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
				return false;
			}
		}
		if (mService == null)
			Log.w(TAG, "Proxy not attached to service");
		return false;
	}

	/**
	 * Initials the Cancel-synchronization procedure. This method will return
	 * immediatedly. If the request has been received by the remote device, the
	 * the CallLog-synchronization/Contact-synchronization procedure will be cancelled.
	 *
	 * @return true if the request has been sent to the remote device
	 *         successfully. false otherwise.
	 */
	public boolean cancelSync() {
		if (DBG)
			log("cancelSync");
		if (mService != null && isEnabled()) {
			try {
				return mService.cancelSync();
			} catch (RemoteException e) {
				Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
				return false;
			}
		}
		if (mService == null)
			Log.w(TAG, "Proxy not attached to service");
		return false;
	}

	public int getSyncState() {
		if (DBG)
			log("cancelSync");
		if (mService != null && isEnabled()) {
			try {
				return mService.getSyncState();
			} catch (RemoteException e) {
				Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
				return 0;
			}
		}
		if (mService == null)
			Log.w(TAG, "Proxy not attached to service");
		return 0;
	}

    /*
     * This method .
     *
     * @param infoMask: bit manipulation with the following constant
     * <ul>
     *   <li> {@link #CONTACT_INFO_MASK_FN </li>
     *   <li> {@link #CONTACT_INFO_MASK_N </li>
     *   <li> {@link #CONTACT_INFO_MASK_TEL </li>
     *   <li> {@link #CONTACT_INFO_MASK_EMAIL </li>
     *   <li> {@link #CONTACT_INFO_MASK_PHOTO </li>
     * </ul>
     * @return
     * */
    public boolean setRequestContactInfo(long infoMask)
    {
        if(DBG)
            log("setRequestContactInfo");
        if(mService != null && isEnabled()) {
            try {
                return mService.setPbapInfoMask(infoMask);
            } catch (RemoteException e) {
                Log.e(TAG, "Unable to setRequestContactInfo", e);
            }
        }
        return false;
    }

	private ServiceConnection mConnection = new ServiceConnection() {
		public void onServiceConnected(ComponentName className, IBinder service) {
			if (DBG)
				log("[debug>>] Proxy object connected");
			mService = IBluetoothPbap.Stub.asInterface(service);
			if (mServiceListener != null) {
				mServiceListener.onServiceConnected(BluetoothProfile.PBAP,
						BluetoothPbapClient.this);
			}

			// Register callback object
			try {
				mService.registerCallback(mPbapCallback);
			} catch (RemoteException re) {
				Log.e(TAG, "Unable to register BluetoothPbapCallback", re);
			}
		}

		public void onServiceDisconnected(ComponentName className) {
			if (DBG)
				log("[debug>>]Proxy object disconnected:" + Thread.currentThread().getId());
			// Unregister callback object
			mService = null;
			if (mServiceListener != null) {
				mServiceListener.onServiceDisconnected(BluetoothProfile.PBAP);
			}
		}
	};

	private boolean isEnabled() {
		if (mAdapter.getState() == BluetoothAdapter.STATE_ON)
			return true;
		return false;
	}

	private boolean isValidDevice(BluetoothDevice device) {
		if (device == null)
			return false;

		if (BluetoothAdapter.checkBluetoothAddress(device.getAddress()))
			return true;
		return false;
	}

	private static void log(String msg) {
		Log.d(TAG, msg);
	}

}
