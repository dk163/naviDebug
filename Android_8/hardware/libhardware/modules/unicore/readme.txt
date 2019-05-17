
SUMMARY
=======
Here lists files affected to integrate Unicore GNSS support driver.
According to your Android version, patch frameworks and HAL driver.
Board specific files should be changed accordingly to work properly.


INSTALL and COMPILE
===================
1. cp hardware/unicore to android source tree, e.g., directory /path/to/android-source-tree/hardware/unicore
2. make sure your BoardConfig.mk contains gps.default.so or gps.<boardname>.so:
	PRODUCT_PACKAGES += \
		gps.<boardname> \
		gps.default
   <boardname> is your board target name.
3. Apply android frameowork patch files(not needed for Nougat/7.0 and above versions)
4. Change board specific settings like com port, gps power/on off pins, etc.
5. compile android source code and download
(NOTE: For Android version 8.0 and above, remove Android.mk)

HAL driver files(added):
======================
hardware/unicore/Android.mk	(Android 7.0 make file)
hardware/unicore/Android.bp	(Android 8.0+ make file)
hardware/unicore/bd_gps.c
hardware/unicore/gps.c
hardware/unicore/geoid.c

board specific files(relative to target root dir)
=================================================
/init.rc
/ueventd.rc
/system/etc/gps/start    <== (optional)
/system/etc/gps/stop     <== (optional)
/sepolicy                <== (platform dependant)

