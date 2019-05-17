package android.bluetooth;

import android.annotation.SdkConstant;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.os.RemoteException;
import android.util.Log;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

/**
 * This class provides the public APIs to receive ANCS(Apple Notification Center
 * Service) messages and take action on these messages.
 */
public final class BluetoothANCSClient {
    private static final String TAG = "BluetoothANCSClient";
    private static final boolean DBG = true;

    private static final int MESSAGE_IOS_NOTIFICATION = 0x01;
    private static final int MESSAGE_NOTIFICATION_ATTRIBUTE = 0x02;
    private static final int MESSAGE_APP_ATTRIBUTE = 0x03;
    private static final int MESSAGE_CONNECTION_STATE_CHANGED = 0x04;

    /**
     * Intent used to broadcast the change in connection state of the ANCS
     * <p/>
     * <p/>
     * This intent will have 2 extras:
     * <ul>
     * <li> {@link #EXTRA_STATE} - The current state of the profile.</li>
     * <li> {@link BluetoothDevice#EXTRA_DEVICE} - The remote device.</li>
     * </ul>
     * <p/>
     * <p/>
     * {@link #EXTRA_STATE}  can be
     * {@link #STATE_DISCONNECTED},or
     * {@link #STATE_CONNECTED}
     * <p/>
     * Requires {@link android.Manifest.permission#BLUETOOTH} permission to
     * receive.
     */
    @SdkConstant(SdkConstant.SdkConstantType.BROADCAST_INTENT_ACTION)
    public static final String ACTION_CONNECTION_STATE_CHANGED = "android.bluetooth.ancs.action.CONNECTION_STATE_CHANGED";
    /**
     * No client currently connected
     */
    public static final int STATE_DISCONNECTED = 0;
    /**
     * Client is currently connected
     */
    public static final int STATE_CONNECTED = 1;

    /**
     * Extra for the connection state intents of the ANCS.
     * <p/>
     * This extra represents the current connection state of the ANCS service of the
     * Bluetooth device.
     */
    public static final String EXTRA_STATE = "android.bluetooth.ancs.extra.STATE";

    /**
     * Category IDs for iOS notifications.
     */
    public enum ANCSNotificationCategory {
        BLE_ANCS_CATEGORY_ID_OTHER,
        BLE_ANCS_CATEGORY_ID_INCOMING_CALL,
        BLE_ANCS_CATEGORY_ID_MISSED_CALL,
        BLE_ANCS_CATEGORY_ID_VOICE_MAIL,
        BLE_ANCS_CATEGORY_ID_SOCIAL,
        BLE_ANCS_CATEGORY_ID_SCHEDULE,
        BLE_ANCS_CATEGORY_ID_EMAIL,
        BLE_ANCS_CATEGORY_ID_NEWS,
        BLE_ANCS_CATEGORY_ID_HEALTH_AND_FITNESS,
        BLE_ANCS_CATEGORY_ID_BUSINESS_AND_FINANCE,
        BLE_ANCS_CATEGORY_ID_LOCATION,
        BLE_ANCS_CATEGORY_ID_ENTERTAINMENT
    }

    /**
     * Event IDs for iOS notifications.
     */
    public enum ANCSNotificationEventID {
        BLE_ANCS_EVENT_ID_NOTIFICATION_ADDED,
        BLE_ANCS_EVENT_ID_NOTIFICATION_MODIFIED,
        BLE_ANCS_EVENT_ID_NOTIFICATION_REMOVED
    }

    /**
     * Event flags for iOS notifications.
     */
    public static final int BLE_ANCS_EVENT_FLAG_SILENT = 1;
    public static final int BLE_ANCS_EVENT_FLAG_IMPORTANT = (1 << 1);
    public static final int BLE_ANCS_EVENT_FLAG_PRE_EXISTING = (1 << 2);
    public static final int BLE_ANCS_EVENT_FLAG_POSITIVE_ACTION = (1 << 3);
    public static final int BLE_ANCS_EVENT_FLAG_NEGATIVE_ACTION = (1 << 4);

    /**
     * Notification Attribute IDs.
     */
    public enum ANCSNotificationAttributeID {
        BLE_ANCS_NOTIFICATION_ATTRIBUTE_ID_APP_IDENTIFIER,
        BLE_ANCS_NOTIFICATION_ATTRIBUTE_ID_TITLE, /* Needs to be followed by a 2-bytes max length parameter */
        BLE_ANCS_NOTIFICATION_ATTRIBUTE_ID_SUBTITLE, /* Needs to be followed by a 2-bytes max length parameter */
        BLE_ANCS_NOTIFICATION_ATTRIBUTE_ID_MESSAGE, /* Needs to be followed by a 2-bytes max length parameter */
        BLE_ANCS_NOTIFICATION_ATTRIBUTE_ID_MESSAGE_SIZE, /*a string that represents the integral value of the message size. */
        BLE_ANCS_NOTIFICATION_ATTRIBUTE_ID_DATE, /* a string that uses the Unicode Technical Standard
                                                  (UTS) #35 date format pattern yyyyMMdd'T'HHmmSS.*/
        BLE_ANCS_NOTIFICATION_ATTRIBUTE_ID_POSITIVE_ACTION_LABEL,
        BLE_ANCS_NOTIFICATION_ATTRIBUTE_ID_NEGATIVE_ACTION_LABEL
    }

    public enum ANCSAppAttributeID {
        APP_ATTRIBUTE_ID_DISPLAY_NAME,
        /*Reserved AppAttributeID values = 1–255*/
    }

    public enum ANCSActionID {
        ACTION_ID_POSITIVE,
        ACTION_ID_NEGATIVE
        /* Reserved ActionID values = 2–255*/
    }

    /**
     * This interface defines callback which will notify clients when some
     * ANCS event takes place.
     */
    public interface ANCSEventListener {
        /**
         * A response from the ANCS Server to Get App Attributes command has arrived
         *
         * @param attribute the Notification attribute
         */
        void onANCSAppAttribute(ApplicationAttribute attribute);

        /**
         * A response from the ANCS Server to Get Notification Attributes command has arrived
         *
         * @param attribute the Application attribute
         */
        void onANCSNotificationAttribute(NotificationAttribute attribute);

        /**
         * An iOS notification has arrived
         *
         * @param notification the iOS notification
         */
        void onANCSiOSNotificationEvent(IOSNotification notification);
    }

    private final Context mContext;
    private IANCSClient mService;
    private final BluetoothAdapter mAdapter;
    private final List<ANCSEventListener> mListeners;

    private final ServiceConnection mConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName className, IBinder service) {
            if (DBG)
                Log.d(TAG, "Proxy object connected");
            mService = IANCSClient.Stub.asInterface(service);

            if (mService != null) {
                try {
                    mService.registerCallback(mANCSCallback);
                } catch (RemoteException re) {
                    Log.e(TAG,
                            "Unable to register IBluetoothAvrcpEventListener",
                            re);
                }
            }
        }

        public void onServiceDisconnected(ComponentName className) {
            if (DBG)
                Log.d(TAG, "Proxy object disconnected");
            if (mService != null) {
                try {
                    mService.unregisterCallback(mANCSCallback);
                } catch (RemoteException re) {
                    Log.e(TAG,
                            "Unable to register IBluetoothAvrcpEventListener",
                            re);
                }
            }
            mService = null;
        }
    };

    final private IBluetoothStateChangeCallback mBluetoothStateChangeCallback = new IBluetoothStateChangeCallback.Stub() {
        public void onBluetoothStateChange(boolean up) {
            if (DBG)
                Log.d(TAG, "onBluetoothStateChange: up=" + up);
            if (!up) {
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
                            if (!mContext.bindService(new Intent(
                                            IANCSClient.class.getName()), mConnection,
                                    0)) {
                                Log.e(TAG,
                                        "Could not bind to ANCS Client Service");
                            }
                        }
                    } catch (Exception re) {
                        Log.e(TAG, "", re);
                    }
                }
            }
        }
    };

    public static final class IOSNotification {
        public int uid;
        public ANCSNotificationEventID eventId;
        public int eventFlag;
        public ANCSNotificationCategory categoryId;
        public int categoryCount;
    }

    public static final class NotificationAttribute {
        public int uid;
        public ANCSNotificationAttributeID attributeId;
        public int attributeLen;
        public String attribute;
    }

    public static final class ApplicationAttribute {
        public String appId;
        public ANCSAppAttributeID attributeId;
        public int attributeLen;
        public String attribute;
    }

    final private Handler mHandler = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            for (ANCSEventListener l : mListeners) {
                switch (msg.what) {
                    case MESSAGE_CONNECTION_STATE_CHANGED:
                        Intent intent = new Intent(ACTION_CONNECTION_STATE_CHANGED);
                        intent.putExtra(EXTRA_STATE, msg.arg1);
                        BluetoothAdapter adapter = BluetoothAdapter.getDefaultAdapter();
                        intent.putExtra(BluetoothDevice.EXTRA_DEVICE, adapter.getRemoteDevice((byte[]) msg.obj));
                        mContext.sendBroadcast(intent);
                        break;
                    case MESSAGE_APP_ATTRIBUTE:
                        ApplicationAttribute appAttr = (ApplicationAttribute) msg.obj;
                        l.onANCSAppAttribute(appAttr);
                        break;
                    case MESSAGE_NOTIFICATION_ATTRIBUTE:
                        NotificationAttribute notifAttr = (NotificationAttribute) msg.obj;
                        l.onANCSNotificationAttribute(notifAttr);
                        break;
                    case MESSAGE_IOS_NOTIFICATION:
                        IOSNotification notif = (IOSNotification) msg.obj;
                        l.onANCSiOSNotificationEvent(notif);
                        break;
                }
            }
        }
    };

    private final IANCSClientCallback mANCSCallback = new IANCSClientCallback.Stub() {
        @Override
        public void onANCSConnectionStateChanged(boolean connected, byte[] address) {
            int state = connected ? STATE_CONNECTED : STATE_DISCONNECTED;
            Message msg = mHandler.obtainMessage(MESSAGE_CONNECTION_STATE_CHANGED, state, 0, address);
            mHandler.sendMessage(msg);
        }

        @Override
        public void onANCSAppAttribute(byte commandId, String appId, byte attributeId, int attributeLen, String attribute) throws RemoteException {
            ApplicationAttribute attr = new ApplicationAttribute();
            attr.appId = appId;
            attr.attribute = attribute;
            attr.attributeId = ANCSAppAttributeID.values()[attributeId];
            attr.attributeLen = attributeLen;

            Message msg = mHandler.obtainMessage(MESSAGE_APP_ATTRIBUTE, attr);
            mHandler.sendMessage(msg);
        }

        @Override
        public void onANCSNotificationAttribute(byte commandId, byte[] notificationUid, byte attributeId, int attributeLen, String attribute) throws RemoteException {
            NotificationAttribute attr = new NotificationAttribute();
            attr.attribute = attribute;
            attr.attributeId = ANCSNotificationAttributeID.values()[attributeId];
            attr.attributeLen = attributeLen;
            attr.uid = ByteBuffer.wrap(notificationUid).order(ByteOrder.LITTLE_ENDIAN).getInt();

            Message msg = mHandler.obtainMessage(MESSAGE_NOTIFICATION_ATTRIBUTE, attr);
            mHandler.sendMessage(msg);
        }

        @Override
        public void onANCSiOSNotificationEvent(byte eventId, byte eventFlag, byte categoryId, byte categoryCount, byte[] notificationUid) throws RemoteException {
            IOSNotification notification = new IOSNotification();
            notification.eventId = ANCSNotificationEventID.values()[eventId];;
            notification.eventFlag = eventFlag & 0xFF;
            notification.categoryId = ANCSNotificationCategory.values()[categoryId];;
            notification.categoryCount = categoryCount & 0xFF;
            notification.uid = ByteBuffer.wrap(notificationUid).order(ByteOrder.LITTLE_ENDIAN).getInt();

            Message msg = mHandler.obtainMessage(MESSAGE_IOS_NOTIFICATION, notification);
            mHandler.sendMessage(msg);
        }
    };

    /**
     * Create a ANCSClient proxy object.
     */
    public BluetoothANCSClient(Context context) {
        mContext = context;
        mAdapter = BluetoothAdapter.getDefaultAdapter();
        IBluetoothManager mgr = mAdapter.getBluetoothManager();
        if (mgr != null) {
            try {
                mgr.registerStateChangeCallback(mBluetoothStateChangeCallback);
            } catch (RemoteException e) {
                Log.e(TAG, "", e);
            }
        }
        if (!context.bindService(new Intent(IANCSClient.class.getName()),
                mConnection, 0)) {
            Log.e(TAG, "Could not bind to Bluetooth ANCS Service");
        }

        mListeners = new ArrayList<ANCSEventListener>();
    }

    protected void finalize() throws Throwable {
        try {
            close();
        } finally {
            super.finalize();
        }
    }

    /**
     * Close the connection to the backing service.
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
                } catch (Exception re) {
                    Log.e(TAG, "", re);
                }
            }
        }
    }

    public boolean enable() {
        if (mService != null) {
            try {
                mService.enable();
                return true;
            } catch (RemoteException e) {
                Log.e(TAG, e.toString());
            }
        } else {
            Log.w(TAG, "Proxy not attached to service");
            if (DBG)
                Log.e(TAG, Log.getStackTraceString(new Throwable()));
        }
        return false;
    }

    public boolean disable() {
        if (mService != null) {
            try {
                mService.disable();
                return true;
            } catch (RemoteException e) {
                Log.e(TAG, e.toString());
            }
        } else {
            Log.w(TAG, "Proxy not attached to service");
            if (DBG)
                Log.e(TAG, Log.getStackTraceString(new Throwable()));
        }
        return false;
    }

    public int getAncsState(BluetoothDevice device){
        if (mService != null) {
            try {
                return mService.getAncsState(device);
            } catch (RemoteException e) {
                Log.e(TAG, e.toString());
            }
        } else {
            Log.w(TAG, "Proxy not attached to service");
            if (DBG)
                Log.e(TAG, Log.getStackTraceString(new Throwable()));
            return STATE_DISCONNECTED;
        }
		return STATE_DISCONNECTED;
    }

    public boolean getNotificationAttribute(int uid, ANCSNotificationAttributeID attrId) {
        if (mService != null) {
            try {
                byte uuid[] = new byte[4];
                uuid[0] = (byte) (uid & 0xFF);
                uuid[1] = (byte) ((uid >> 8) & 0xFF);
                uuid[2] = (byte) ((uid >> 16) & 0xFF);
                uuid[3] = (byte) ((uid >> 24) & 0xFF);
                mService.getNotificationAttribute(uuid, (byte) attrId.ordinal(), 63);
                return true;
            } catch (RemoteException e) {
                Log.e(TAG, e.toString());
            }
        } else {
            Log.w(TAG, "Proxy not attached to service");
            if (DBG)
                Log.e(TAG, Log.getStackTraceString(new Throwable()));
        }
        return false;
    }

    public boolean getAppAttribute(String appId, ANCSAppAttributeID attrId) {
        if (mService != null) {
            try {
                mService.getAppAttribute(appId, (byte) attrId.ordinal());
                return true;
            } catch (RemoteException e) {
                Log.e(TAG, e.toString());
            }
        } else {
            Log.w(TAG, "Proxy not attached to service");
            if (DBG)
                Log.e(TAG, Log.getStackTraceString(new Throwable()));
        }
        return false;
    }

    public boolean performAction(int uid, ANCSActionID action) {
        if (mService != null) {
            try {
                byte[] uuid = new byte[4];
                uuid[0] = (byte) (uid & 0xFF);
                uuid[1] = (byte) ((uid >> 8) & 0xFF);
                uuid[2] = (byte) ((uid >> 16) & 0xFF);
                uuid[3] = (byte) ((uid >> 24) & 0xFF);
                mService.performAction(uuid, (byte) action.ordinal());
                return true;
            } catch (RemoteException e) {
                Log.e(TAG, e.toString());
            }
        } else {
            Log.w(TAG, "Proxy not attached to service");
            if (DBG)
                Log.e(TAG, Log.getStackTraceString(new Throwable()));
        }
        return false;
    }

    /**
     * Register a {@link ANCSEventListener} which will be notified when some ANCS
     * event takes place.
     *
     * @param listener The event listener.
     */
    public void registerEventListener(ANCSEventListener listener) {
        mListeners.add(listener);
    }

    /**
     * Unregister a {@link ANCSEventListener}.
     *
     * @param listener the event listener
     */
    public void unregisterEventListener(ANCSEventListener listener) {
        mListeners.remove(listener);
    }

}
