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
 * Public API for controlling the Bluetooth Ftp Service. BluetoothFtp is a proxy
 * object for controlling the Bluetooth Ftp Service via IPC.
 *
 * Creating a BluetoothFtp object will create a binding with the BluetoothFtp
 * service. Users of this object should call close() when they are finished with
 * the BluetoothFtp, so that this proxy object can unbind from the service.
 *
 * This BluetoothFtp object is not immediately bound to the BluetoothFtp
 * service. Use the ServiceListener interface to obtain a notification when it
 * is bound, this is especially important if you wish to immediately call
 * methods on BluetoothFtp after construction.
 *
 * Android only supports one connected Bluetooth Fce at a time.
 *
 * @hide
 */
public class BluetoothFtp implements BluetoothProfile {

	private static final String TAG = "BluetoothFtp";
	private static final boolean DBG = true;
	private static final boolean VDBG = false;

	/**
	 * Intent used to broadcast the change in connection state of the FTP
	 * profile.
	 *
	 * <p>
	 * This intent will have 3 extras:
	 * <ul>
	 * <li> {@link #EXTRA_STATE} - The current state of the profile.</li>
	 * <li> {@link #EXTRA_PREVIOUS_STATE}- The previous state of the profile.</li>
	 * <li> {@link BluetoothDevice#EXTRA_DEVICE} - The remote device.</li>
	 * </ul>
	 *
	 * <p>
	 * {@link #EXTRA_STATE} or {@link #EXTRA_PREVIOUS_STATE} can be any of
	 * {@link #STATE_DISCONNECTED}, {@link #STATE_CONNECTING},
	 * {@link #STATE_CONNECTED}, {@link #STATE_DISCONNECTING}.
	 *
	 * <p>
	 * Requires {@link android.Manifest.permission#BLUETOOTH} permission to
	 * receive.
	 */
	@SdkConstant(SdkConstantType.BROADCAST_INTENT_ACTION)
	public static final String ACTION_CONNECTION_STATE_CHANGED = "android.bluetooth.ftp.profile.action.CONNECTION_STATE_CHANGED";
	/** There was an error trying to obtain the state */
	public static final int STATE_ERROR = -1;
	/** No client currently connected */
	public static final int STATE_DISCONNECTED = 0;
	/** Connection attempt in progress */
	public static final int STATE_CONNECTING = 1;
	/** Client is currently connected */
	public static final int STATE_CONNECTED = 2;
	/** Client is currently connected and do nothing */
	public static final int STATE_IDLE = 3;
	/** Client is currently connected and pulling object */
	public static final int STATE_PULLING = 4;
	/** Client is currently connected and pushing object */
	public static final int STATE_PUSHING = 5;
	/** Client is currently connected and deleting object */
	public static final int STATE_DELETING = 6;

	private static final int MESSAGE_OPERATION_RESULT = 0x01;
	private static final int MESSAGE_BLUETOOTH_FTP_PROGRESS_IN_PUSH_IND = 0x02;
	private static final int MESSAGE_BLUETOOTH_FTP_PROGRESS_IN_PULL_IND = 0x03;
	private static final int MESSAGE_BLUETOOTH_FTP_OBJECT_INFORMATION_IND = 0x04;

	/**
	 * An event listener can implements this interface to receive the Ftp
	 * events. Such as FTP synchronization started, finished and some
	 * Contact/CallLog item is fetched from the remote device.
	 *
	 */
	public interface EventListener {
		enum Operation {
			OPERATION_CHANGE_FOLDER,
			OPERATION_BROWSE,
			OPERATION_DELETE,
			OPERATION_PULL,
			OPERATION_PUSH,
		};

		enum Result {
			STARTED, ABORTED, FINISHED,
		}

		/**
		 *
		 * @param object
		 * @param operation
		 * @param result
		 */
		public void onOperationResult(BluetoothFtpObject object,
				Operation operation, Result result);

		/**
		 *
		 * @param object
		 * @param pushedSize
		 */
		public void onPushInProgress(BluetoothFtpObject object, int pushedSize);

		/**
		 *
		 * @param object
		 * @param pulledSize
		 */
		public void onPullInProgress(BluetoothFtpObject object, int pulledSize);

		/**
		 *
		 * @param object
		 */
		public void onObjectBrowsed(BluetoothFtpObject object);

	}

	private IBluetoothFtp mService;
	private final Context mContext;
	private ServiceListener mServiceListener;
	private BluetoothAdapter mAdapter;
	private List<EventListener> mFtpListeners;

	final private IBluetoothStateChangeCallback mBluetoothStateChangeCallback = new IBluetoothStateChangeCallback.Stub() {
		public void onBluetoothStateChange(boolean up) {
			if (DBG)
				Log.d(TAG, "onBluetoothStateChange: up=" + up);
			if (!up) {
				if (VDBG)
					Log.d(TAG, "Unbinding service...");
				synchronized (mConnection) {
					try {
						mService = null;
						mContext.unbindService(mConnection);
					} catch (Exception re) {
						Log.e(TAG, "", re);
					}
				}
			} else {
				synchronized (mConnection) {
					try {
						if (mService == null) {
							if (VDBG)
								Log.d(TAG, "Binding service...");
							if (!mContext.bindService(new Intent(
									IBluetoothFtp.class.getName()),
									mConnection, 0)) {
								Log.e(TAG,
										"Could not bind to Bluetooth FTP Service");
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
			for (EventListener l : mFtpListeners) {
				switch (msg.what) {
				case MESSAGE_OPERATION_RESULT:
					int oIdx = msg.arg1;
					int rIdx = msg.arg2;
					BluetoothFtpObject obj = (BluetoothFtpObject) msg.obj;
					EventListener.Result result = EventListener.Result
							.values()[rIdx];
					EventListener.Operation operation = EventListener.Operation.values()[oIdx];
					l.onOperationResult(obj, operation, result);
					break;
				case MESSAGE_BLUETOOTH_FTP_PROGRESS_IN_PUSH_IND:
					l.onPushInProgress((BluetoothFtpObject) msg.obj, msg.arg1);
					break;
				case MESSAGE_BLUETOOTH_FTP_PROGRESS_IN_PULL_IND:
					l.onPullInProgress((BluetoothFtpObject) msg.obj, msg.arg1);
					break;
				case MESSAGE_BLUETOOTH_FTP_OBJECT_INFORMATION_IND:
					l.onObjectBrowsed((BluetoothFtpObject) msg.obj);
					break;
				}
			}
		}
	};

	/**
	 * Create a BluetoothFtp proxy object.
	 */
	/* package */BluetoothFtp(Context context,
			android.bluetooth.BluetoothProfile.ServiceListener l) {
		mContext = context;
		mServiceListener = l;
		mAdapter = BluetoothAdapter.getDefaultAdapter();
		IBluetoothManager mgr = mAdapter.getBluetoothManager();
		if (mgr != null) {
			try {
				mgr.registerStateChangeCallback(mBluetoothStateChangeCallback);
			} catch (RemoteException e) {
				Log.e(TAG, "", e);
			}
		}
		if (!context.bindService(new Intent(IBluetoothFtp.class.getName()),
				mConnection, 0)) {
			Log.e(TAG, "Could not bind to Bluetooth Ftp Service");
		}

		mFtpListeners = new ArrayList<EventListener>();
	}

	private IBluetoothFtpCallback mFtpCallback = new IBluetoothFtpCallback.Stub() {

		@Override
		public void onOperationResult(BluetoothFtpObject object, int operation,
				int result) throws RemoteException {
			Message msg = mHandler.obtainMessage(MESSAGE_OPERATION_RESULT,
					operation, result, object);

			mHandler.sendMessage(msg);
		}

		@Override
		public void onPushInProgress(BluetoothFtpObject object, int pushedSize)
				throws RemoteException {
			Message msg = mHandler.obtainMessage(
					MESSAGE_BLUETOOTH_FTP_PROGRESS_IN_PUSH_IND, pushedSize, 0,
					object);
			mHandler.sendMessage(msg);
		}

		@Override
		public void onPullInProgress(BluetoothFtpObject object, int pulledSize)
				throws RemoteException {
			Message msg = mHandler.obtainMessage(
					MESSAGE_BLUETOOTH_FTP_PROGRESS_IN_PULL_IND, pulledSize, 0,
					object);
			mHandler.sendMessage(msg);
		}

		@Override
		public void onObjectBrowsed(BluetoothFtpObject object)
				throws RemoteException {
			Message msg = mHandler.obtainMessage(
					MESSAGE_BLUETOOTH_FTP_OBJECT_INFORMATION_IND, object);
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
	 * BluetoothFtp will return default error results once close() has been
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

		synchronized (mConnection) {
			if (mService != null) {
				try {
					mService = null;
					mContext.unbindService(mConnection);
					mConnection = null;
				} catch (Exception re) {
					Log.e(TAG, "", re);
				}
			}
		}
		mServiceListener = null;
	}

	/**
	 * Register a {@link EventListener} which will be notified when some Ftp
	 * event takes place.
	 *
	 * @param listener
	 *            The event listener.
	 */
	public void registerEventListener(EventListener listener) {
		mFtpListeners.add(listener);
	}

	/**
	 * Unregister a {@link EventListener}.
	 *
	 * @param listener
	 *            the event listener
	 */
	public void unregisterEventListener(EventListener listener) {
		mFtpListeners.remove(listener);
	}

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

	/**
	 * Initiate connection to a profile of the remote bluetooth device.
	 *
	 * <p>
	 * Currently, the system supports only 1 connection to the FTP profile. The
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
	 * @return false on immediate error, true otherwise
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
	 * Disconnects the current Ftp client (PCE). Currently this call blocks, it
	 * may soon be made asynchronous. Returns false if this proxy object is not
	 * currently connected to the Ftp service. *
	 * <p>
	 * Requires {@link android.Manifest.permission#BLUETOOTH_ADMIN} permission.
	 *
	 * @param device
	 *            Remote Bluetooth Device
	 * @return false on immediate error, true otherwise
	 * @hide
	 */
	public boolean disconnect(BluetoothDevice device) {
		if (DBG)
			log("disconnect()");
		if (mService != null) {
			try {
				mService.disconnect(device);
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

	public boolean cancelSyncObject(BluetoothFtpObject object) {
		if (DBG)
			log("cancelSyncObject()");
		if (mService != null && isEnabled()) {
			try {
				return mService.cancelSyncObject(object);
			} catch (RemoteException e) {
				Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
				return false;
			}
		}
		if (mService == null)
			Log.w(TAG, "Proxy not attached to service");
		return false;
	}

	public boolean browseRemoteFolder() {
		if (DBG)
			log("browseRemoteFolder()");
		if (mService != null && isEnabled()) {
			try {
				return mService.browseRemoteFolder();
			} catch (RemoteException e) {
				Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
				return false;
			}
		}
		if (mService == null)
			Log.w(TAG, "Proxy not attached to service");
		return false;
	}

	public boolean changeRemoteFolder(String path) {
		if (DBG)
			log("changeRemoteFolder(" + path + ")");
		if (mService != null && isEnabled()) {
			try {
				return mService.changeRemoteFolder(path);
			} catch (RemoteException e) {
				Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
				return false;
			}
		}
		if (mService == null)
			Log.w(TAG, "Proxy not attached to service");
		return false;
	}

	public boolean changeLocalFolder(String path) {
		if (DBG)
			log("cahngeLocalFolder(" + path + ")");
		if (mService != null && isEnabled()) {
			try {
				return mService.changeLocalFolder(path);
			} catch (RemoteException e) {
				Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
				return false;
			}
		}
		if (mService == null)
			Log.w(TAG, "Proxy not attached to service");
		return false;
	}

	public boolean deleteObject(BluetoothFtpObject object) {
		if (DBG)
			log("deleteObject(" + object + ")");
		if (mService != null && isEnabled()) {
			try {
				return mService.deleteObject(object);
			} catch (RemoteException e) {
				Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
				return false;
			}
		}
		if (mService == null)
			Log.w(TAG, "Proxy not attached to service");
		return false;
	}

	public boolean pullObject(BluetoothFtpObject object) {
		if (DBG)
			log("pullObject(" + object + ")");
		if (mService != null && isEnabled()) {
			try {
				return mService.pullObject(object);
			} catch (RemoteException e) {
				Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
				return false;
			}
		}
		if (mService == null)
			Log.w(TAG, "Proxy not attached to service");
		return false;
	}

	public boolean pushObject(BluetoothFtpObject object) {
		if (DBG)
			log("pushObject(" + object + ")");
		if (mService != null && isEnabled()) {
			try {
				return mService.pushObject(object);
			} catch (RemoteException e) {
				Log.e(TAG, "Stack:" + Log.getStackTraceString(new Throwable()));
				return false;
			}
		}
		if (mService == null)
			Log.w(TAG, "Proxy not attached to service");
		return false;
	}

	private ServiceConnection mConnection = new ServiceConnection() {
		public void onServiceConnected(ComponentName className, IBinder service) {
			if (DBG)
				log("Proxy object connected");
			mService = IBluetoothFtp.Stub.asInterface(service);
			if (mServiceListener != null) {
				mServiceListener.onServiceConnected(BluetoothProfile.FTP,
						BluetoothFtp.this);
			}

			// Register callback object
			try {
				mService.registerCallback(mFtpCallback);
			} catch (RemoteException re) {
				Log.e(TAG, "Unable to register BluetoothFtpCallback", re);
			}
		}

		public void onServiceDisconnected(ComponentName className) {
			if (DBG)
				log("Proxy object disconnected");
			// Unregister callback object
			try {
				mService.unregisterCallback(mFtpCallback);
			} catch (RemoteException re) {
				Log.e(TAG, "Unable to register BluetoothFtpCallback", re);
			}
			mService = null;
			if (mServiceListener != null) {
				mServiceListener.onServiceDisconnected(BluetoothProfile.FTP);
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
