
SUMMARY
=======
Here lists files affected to integrate Unicore GNSS support driver.
According to your Android version, patch frameworks and HAL driver.
"Developer options" provides an menu option to choose among three positioning
modes of Unicore GNSS chips(e.g., GPS only, BeiDou only or both).
Board specific files should be changed accordingly to work properly.


Android 4.4.x files(changed)
============================
frameworks/base/location/java/android/location/GpsStatus.java
frameworks/base/location/java/android/location/IGpsStatusListener.aidl
frameworks/base/location/java/android/location/LocationManager.java
frameworks/base/services/java/com/android/server/location/GpsLocationProvider.java
frameworks/base/services/jni/com_android_server_location_GpsLocationProvider.cpp

Android 5.0/6.0 files(changed)
===========================
frameworks/base/location/java/android/location/GpsStatus.java
frameworks/base/location/java/android/location/IGpsStatusListener.aidl
frameworks/base/location/java/android/location/LocationManager.java
frameworks/base/services/core/java/com/android/server/location/GpsLocationProvider.java
frameworks/base/services/core/java/com/android/server/location/GpsStatusListenerHelper.java
frameworks/base/services/core/jni/com_android_server_location_GpsLocationProvider.cpp

Developer options(optional, skip if not used)
=============================================
packages/apps/Settings/res/values/arrays.xml
packages/apps/Settings/res/values/strings.xml
packages/apps/Settings/res/xml/development_prefs.xml
packages/apps/Settings/src/com/android/settings/DevelopmentSettings.java

HAL driver files(added):
======================
hardware/libhardware/include/hardware/gps.h
hardware/unicore/Android.mk
hardware/unicore/bd_gps.c
hardware/unicore/gps.c

board specific files(relative to target root dir)
=================================================
/init.rc
/ueventd.rc
/system/etc/gps/start    <== (optional)
/system/etc/gps/stop     <== (optional)
/sepolicy                <== (platform dependant)

