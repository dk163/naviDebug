/*
 * Copyright (C) 2010 The Android Open Source Project
 * Copyright (C) 2013-2014 Freescale Semiconductor, Inc.
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

package android.net;

import android.content.Context;
import android.net.NetworkInfo.DetailedState;
import android.os.Handler;
import android.os.IBinder;
import android.os.INetworkManagementService;
import android.os.Message;
import android.os.Messenger;
import android.os.RemoteException;
import android.os.ServiceManager;
import android.os.SystemProperties;
import android.util.Log;
import android.os.PowerManager;

import com.android.server.net.BaseNetworkObserver;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import java.net.InetAddress;
import android.os.Build;

/**
 * This class tracks the data connection associated with Ethernet
 * This is a singleton class and an instance will be created by
 * ConnectivityService.
 * @hide
 */
public class EthernetDataTracker extends BaseNetworkStateTracker {
    private static final String NETWORKTYPE = "ETHERNET";
    private static final String TAG = "Ethernet";

    private AtomicBoolean mTeardownRequested = new AtomicBoolean(false);
    private AtomicBoolean mPrivateDnsRouteSet = new AtomicBoolean(false);
    private AtomicInteger mDefaultGatewayAddr = new AtomicInteger(0);
    private AtomicBoolean mDefaultRouteSet = new AtomicBoolean(false);

    private static boolean mLinkUp;
    private static boolean mNfsmode;
    private static boolean mAlwayson;
    private LinkProperties mLinkProperties;
    private LinkCapabilities mLinkCapabilities;
    private NetworkInfo mNetworkInfo;
    private InterfaceObserver mInterfaceObserver;
    private String mHwAddr;
    private static PowerManager.WakeLock mEthernetWakeLock;

    /* For sending events to connectivity service handler */
    private Handler mCsHandler;

    private static EthernetDataTracker sInstance;
    private static String sIfaceMatch = "";
    private static String mIface = "";

    private INetworkManagementService mNMService;
	static final boolean IS_S111_6D_BUILD = "chinatsp_s111_6d".equals(Build.PRODUCT);
    private static class InterfaceObserver extends BaseNetworkObserver {
        private EthernetDataTracker mTracker;

        InterfaceObserver(EthernetDataTracker tracker) {
            super();
            mTracker = tracker;
        }

        @Override
        public void interfaceStatusChanged(String iface, boolean up) {
            Log.d(TAG, "Interface status changed: " + iface + (up ? "up" : "down"));
        }

        @Override
        public void interfaceLinkStateChanged(String iface, boolean up) {
            if (mIface.equals(iface)) {
                Log.d(TAG, "Interface " + iface + " link " + (up ? "up" : "down"));
                mLinkUp = up;
                mAlwayson = "yes".equals(SystemProperties.get("ro.ethernet.alwayson.mode", "yes"));
                mTracker.mNetworkInfo.setIsAvailable(up);

                // use DHCP
                if (up) {
                    if (mAlwayson)
                        mEthernetWakeLock.acquire();
                    mTracker.reconnect();
                } else {
                    if (mAlwayson)
                        mEthernetWakeLock.release();
                    mTracker.disconnect();
                }
            }
        }

        @Override
        public void interfaceAdded(String iface) {
            mTracker.interfaceAdded(iface);
        }

        @Override
        public void interfaceRemoved(String iface) {
            mTracker.interfaceRemoved(iface);
        }
    }

    private EthernetDataTracker() {
        mNetworkInfo = new NetworkInfo(ConnectivityManager.TYPE_ETHERNET, 0, NETWORKTYPE, "");
        mLinkProperties = new LinkProperties();
        mLinkCapabilities = new LinkCapabilities();
		
		if (!IS_S111_6D_BUILD) return;
		// tbox net tracker for BQ03/BQ04
		
		// start native tracker
		SystemProperties.set("ctl.start", "tbox_net_tracker");
		
		// start framework tracker
		new Thread(new Runnable() {
            public void run() {
				while (true) {
		            boolean tbox_net = "1".equals(SystemProperties.get("sys.chinatsp.tbox.net", "1"));
					Log.d(TAG, "********tbox_net: " + tbox_net + ", mNetworkInfo.isAvailable(): " + mNetworkInfo.isAvailable());
					if (tbox_net != mNetworkInfo.isAvailable()) {
				        mNetworkInfo.setIsAvailable(tbox_net);
	    				Message msg = mCsHandler.obtainMessage(EVENT_TBOX_NETWORK_STATE_CHANGED, mNetworkInfo);
	    				msg.sendToTarget();
				    }
					try {
		                Thread.sleep(5000);  // 5 seconds
		            } catch (InterruptedException e) {}
				}
            }
        }).start();
    }

    private void interfaceAdded(String iface) {
        if (!iface.matches(sIfaceMatch))
            return;

        Log.d(TAG, "Adding " + iface);

        synchronized(this) {
            if(!mIface.isEmpty())
                return;
            mIface = iface;
        }

        // we don't get link status indications unless the iface is up - bring it up
        try {
            mNMService.setInterfaceUp(iface);
        } catch (Exception e) {
            Log.e(TAG, "Error upping interface " + iface + ": " + e);
        }

        mNetworkInfo.setIsAvailable(true);
        Message msg = mCsHandler.obtainMessage(EVENT_CONFIGURATION_CHANGED, mNetworkInfo);
        msg.sendToTarget();
    }

    public void disconnect() {

        NetworkUtils.stopDhcp(mIface);

        mLinkProperties.clear();
        mNetworkInfo.setIsAvailable(false);
        mNetworkInfo.setDetailedState(DetailedState.DISCONNECTED, null, mHwAddr);

        Message msg = mCsHandler.obtainMessage(EVENT_CONFIGURATION_CHANGED, mNetworkInfo);
        msg.sendToTarget();

        msg = mCsHandler.obtainMessage(EVENT_STATE_CHANGED, mNetworkInfo);
        msg.sendToTarget();

        if (!mNfsmode) {
           IBinder b = ServiceManager.getService(Context.NETWORKMANAGEMENT_SERVICE);
           INetworkManagementService service = INetworkManagementService.Stub.asInterface(b);
           try {
               service.clearInterfaceAddresses(mIface);
           } catch (Exception e) {
               Log.e(TAG, "Failed to clear addresses or disable ipv6" + e);
           }
        }
    }

    private void interfaceRemoved(String iface) {
        if (!iface.equals(mIface))
            return;

        Log.d(TAG, "Removing " + iface);
        disconnect();
        mIface = "";
    }

    private void runDhcp() {
        Thread dhcpThread = new Thread(new Runnable() {
            public void run() {
                DhcpResults dhcpResults = new DhcpResults();
                if (!NetworkUtils.runDhcp(mIface, dhcpResults)) {
                    Log.e(TAG, "DHCP request error:" + NetworkUtils.getDhcpError());

					if ("usb0".equals(mIface)) {
						try {
							Log.e(TAG, "*********runDhcp: construct link according to static dhcp policy for usb0.");
						
						    // construct lock link properties
							InetAddress address_local = InetAddress.parseNumericAddress("10.0.0.2");
							LinkAddress linkAddress_local = new LinkAddress(address_local, 24); 				
							
							// construct route 1 properties
							InetAddress address_dest_1 = InetAddress.parseNumericAddress("10.0.0.0");
							LinkAddress linkAddress_dest_1 = new LinkAddress(address_dest_1, 24);
							InetAddress address_gateway_1 = InetAddress.parseNumericAddress("0.0.0.0");
							RouteInfo routeInfo_dest_1 = new RouteInfo(linkAddress_dest_1, address_gateway_1, "usb0");
							
							// construct route 2 properties
							InetAddress address_dest_2 = InetAddress.parseNumericAddress("0.0.0.0");
							LinkAddress linkAddress_dest_2 = new LinkAddress(address_dest_2, 0);
							InetAddress address_gateway_2 = InetAddress.parseNumericAddress("10.0.0.1");
							RouteInfo routeInfo_dest_2 = new RouteInfo(linkAddress_dest_2, address_gateway_2, "usb0");
							
							// construct dns 1 properties
							InetAddress address_dns_1 = InetAddress.parseNumericAddress("8.8.8.8");
							// construct dns 2 properties
							InetAddress address_dns_2 = InetAddress.parseNumericAddress("8.8.8.8");
							
							// fill in all link properties
							// local link
							dhcpResults.linkProperties.addLinkAddress(linkAddress_local);

							// iface
							dhcpResults.linkProperties.setInterfaceName("usb0");
							
							// mtu
							dhcpResults.linkProperties.setMtu(0);
							
							// route
							dhcpResults.linkProperties.addRoute(routeInfo_dest_1);
							dhcpResults.linkProperties.addRoute(routeInfo_dest_2);
							
							// dns
							dhcpResults.linkProperties.addDns(address_dns_1);
							dhcpResults.linkProperties.addDns(address_dns_2);

							// domain
							dhcpResults.linkProperties.setDomains("");

							// set usb0 dns property for vm
							SystemProperties.set("dhcp.usb0.dns1", "8.8.8.8");
							SystemProperties.set("dhcp.usb0.dns2", "8.8.8.8");
						} catch (Exception e) {
							// just return for inter errors
							return;
						}
					} else {
						// just return for other ifaces
                    	return;
					}
                }
                mLinkProperties = dhcpResults.linkProperties;

                mNetworkInfo.setIsAvailable(true);
                mNetworkInfo.setDetailedState(DetailedState.CONNECTED, null, mHwAddr);
                Message msg = mCsHandler.obtainMessage(EVENT_STATE_CHANGED, mNetworkInfo);
                msg.sendToTarget();
				Log.d(TAG, "********mLinkProperties: " + mLinkProperties.toString());
            }
        });
        dhcpThread.start();
    }

    public static synchronized EthernetDataTracker getInstance() {
        if (sInstance == null) sInstance = new EthernetDataTracker();
        return sInstance;
    }

    public Object Clone() throws CloneNotSupportedException {
        throw new CloneNotSupportedException();
    }

    public void setTeardownRequested(boolean isRequested) {
        mTeardownRequested.set(isRequested);
    }

    public boolean isTeardownRequested() {
        return mTeardownRequested.get();
    }

    /**
     * Begin monitoring connectivity
     */
    public void startMonitoring(Context context, Handler target) {
        mContext = context;
        mCsHandler = target;

        final PowerManager powerManager = (PowerManager)context.getSystemService(
                Context.POWER_SERVICE);
        mEthernetWakeLock = powerManager.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, TAG);
        // register for notifications from NetworkManagement Service
        IBinder b = ServiceManager.getService(Context.NETWORKMANAGEMENT_SERVICE);
        mNMService = INetworkManagementService.Stub.asInterface(b);

        mNfsmode = "yes".equals(SystemProperties.get("ro.nfs.mode", "no"));
        mInterfaceObserver = new InterfaceObserver(this);

        // enable and try to connect to an ethernet interface that
        // already exists
        sIfaceMatch = context.getResources().getString(
            com.android.internal.R.string.config_ethernet_iface_regex);
        try {
            final String[] ifaces = mNMService.listInterfaces();
            for (String iface : ifaces) {
                if (iface.matches(sIfaceMatch)) {
                    mIface = iface;
                    mNMService.setInterfaceUp(iface);
					// should be inited and will be triggered by NMS later
					mLinkUp = true;
                    InterfaceConfiguration config = mNMService.getInterfaceConfig(iface);
                    if (config != null && mHwAddr == null) {
                        mHwAddr = config.getHardwareAddress();
                        if (mHwAddr != null) {
                            mNetworkInfo.setExtraInfo(mHwAddr);
                        }
                    }

                    // if a DHCP client had previously been started for this interface, then stop it
                    NetworkUtils.stopDhcp(mIface);

                    reconnect();
                }
            }
        } catch (RemoteException e) {
            Log.e(TAG, "Could not get list of interfaces " + e);
        }

        try {
            mNMService.registerObserver(mInterfaceObserver);
        } catch (RemoteException e) {
            Log.e(TAG, "Could not register InterfaceObserver " + e);
        }
    }

    /**
     * Disable connectivity to a network
     * TODO: do away with return value after making MobileDataStateTracker async
     */
    public boolean teardown() {
    	Log.d(TAG, "usb0 NEVER need tear down.");
        //mTeardownRequested.set(true);
        //NetworkUtils.stopDhcp(mIface);
        return true;
    }

    /**
     * Re-enable connectivity to a network after a {@link #teardown()}.
     */
    public boolean reconnect() {
        if (mLinkUp || mNfsmode) {
            mTeardownRequested.set(false);
            runDhcp();
        }
        return mLinkUp;
    }

    @Override
    public void captivePortalCheckComplete() {
        // not implemented
    }

    @Override
    public void captivePortalCheckCompleted(boolean isCaptivePortal) {
        // not implemented
    }

    /**
     * Turn the wireless radio off for a network.
     * @param turnOn {@code true} to turn the radio on, {@code false}
     */
    public boolean setRadio(boolean turnOn) {
        return true;
    }

    /**
     * @return true - If are we currently tethered with another device.
     */
    public synchronized boolean isAvailable() {
        return mNetworkInfo.isAvailable();
    }

    /**
     * Tells the underlying networking system that the caller wants to
     * begin using the named feature. The interpretation of {@code feature}
     * is completely up to each networking implementation.
     * @param feature the name of the feature to be used
     * @param callingPid the process ID of the process that is issuing this request
     * @param callingUid the user ID of the process that is issuing this request
     * @return an integer value representing the outcome of the request.
     * The interpretation of this value is specific to each networking
     * implementation+feature combination, except that the value {@code -1}
     * always indicates failure.
     * TODO: needs to go away
     */
    public int startUsingNetworkFeature(String feature, int callingPid, int callingUid) {
        return -1;
    }

    /**
     * Tells the underlying networking system that the caller is finished
     * using the named feature. The interpretation of {@code feature}
     * is completely up to each networking implementation.
     * @param feature the name of the feature that is no longer needed.
     * @param callingPid the process ID of the process that is issuing this request
     * @param callingUid the user ID of the process that is issuing this request
     * @return an integer value representing the outcome of the request.
     * The interpretation of this value is specific to each networking
     * implementation+feature combination, except that the value {@code -1}
     * always indicates failure.
     * TODO: needs to go away
     */
    public int stopUsingNetworkFeature(String feature, int callingPid, int callingUid) {
        return -1;
    }

    @Override
    public void setUserDataEnable(boolean enabled) {
        Log.w(TAG, "ignoring setUserDataEnable(" + enabled + ")");
    }

    @Override
    public void setPolicyDataEnable(boolean enabled) {
        Log.w(TAG, "ignoring setPolicyDataEnable(" + enabled + ")");
    }

    /**
     * Check if private DNS route is set for the network
     */
    public boolean isPrivateDnsRouteSet() {
        return mPrivateDnsRouteSet.get();
    }

    /**
     * Set a flag indicating private DNS route is set
     */
    public void privateDnsRouteSet(boolean enabled) {
        mPrivateDnsRouteSet.set(enabled);
    }

    /**
     * Fetch NetworkInfo for the network
     */
    public synchronized NetworkInfo getNetworkInfo() {
        return mNetworkInfo;
    }

    /**
     * Fetch LinkProperties for the network
     */
    public synchronized LinkProperties getLinkProperties() {
        return new LinkProperties(mLinkProperties);
    }

   /**
     * A capability is an Integer/String pair, the capabilities
     * are defined in the class LinkSocket#Key.
     *
     * @return a copy of this connections capabilities, may be empty but never null.
     */
    public LinkCapabilities getLinkCapabilities() {
        return new LinkCapabilities(mLinkCapabilities);
    }

    /**
     * Fetch default gateway address for the network
     */
    public int getDefaultGatewayAddr() {
        return mDefaultGatewayAddr.get();
    }

    /**
     * Check if default route is set
     */
    public boolean isDefaultRouteSet() {
        return mDefaultRouteSet.get();
    }

    /**
     * Set a flag indicating default route is set for the network
     */
    public void defaultRouteSet(boolean enabled) {
        mDefaultRouteSet.set(enabled);
    }

    /**
     * Return the system properties name associated with the tcp buffer sizes
     * for this network.
     */
    public String getTcpBufferSizesPropName() {
        return "net.tcp.buffersize.ethernet";
    }

    public void setDependencyMet(boolean met) {
        // not supported on this network
    }

    @Override
    public void addStackedLink(LinkProperties link) {
        mLinkProperties.addStackedLink(link);
    }

    @Override
    public void removeStackedLink(LinkProperties link) {
        mLinkProperties.removeStackedLink(link);
    }

    @Override
    public void supplyMessenger(Messenger messenger) {
        // not supported on this network
    }
}
