#ublox, system/lib/hw
1.mek_8q.mk add
#GPS GNSS HAL
PRODUCT_PACKAGES += \
    android.hardware.gnss@1.0-impl \
	android.hardware.gnss@1.0-service
    
#u-blox GPS
PRODUCT_COPY_FILES += \
		hardware/gps/u-blox/gps.conf:system/etc/gps.conf        \
		hardware/gps/u-blox/u-blox.conf:system/etc/u-blox.conf      \
		hardware/gps/u-blox/u-blox-tbox.conf:system/etc/u-blox-tbox.conf
        
2.device/fsl/imx8/sepolicy/hal_gnss_default.te
vndbinder_use(hal_gnss_default)
allow hal_gnss_default system_data_file:file { write create open read getattr };
allow hal_gnss_default system_data_file:dir { write add_name create open };
dontaudit hal_gnss_default self:udp_socket create;

3.device/fsl/mek_8q/init.rc
# Create directories for Location services
mkdir /data/misc/location 0770 gps gps
mkdir /data/misc/location/mq 0770 gps gps
mkdir /data/misc/location/xtwifi 0770 gps gps
mkdir /data/misc/location/gpsone_d 0770 system gps
mkdir /data/misc/location/quipc 0770 gps system
mkdir /data/misc/location/gsiff 0770 gps gps

# migrating the GNSS hal to vendor requires this to be relabeled; the
# directory itself is created by /vendor/bin/xtra-daemon
chown gps gps /data/misc/location/xtra
chmod 0750 /data/misc/location/xtra
chown gps gps /data/misc/location/xtra/socket_hal_xtra
chmod 0660 /data/misc/location/xtra/socket_hal_xtra
chown gps gps /data/misc/location/xtra/xtra.sqlite

4.device/fsl/mek_8q/manifest.xml
<hal format="hidl">
    <name>android.hardware.gnss</name>
    <transport>hwbinder</transport>
    <version>1.0</version>
    <interface>
        <name>IGnss</name>
        <instance>default</instance>
    </interface>
</hal>
<hal format="hidl">

5.device/fsl/mek_8q/ueventd.freescale.rc, add
/dev/ttyLP2               0660   system     gps

6.system/sepolicy/public/domain.te,add
neverallow {
  domain
  -system_server
  -system_app
  -hal_gnss
  -init
  -installd # for relabelfrom and unlink, check for this in explicit neverallow
  with_asan(`-asan_extract')
} system_data_file:file no_w_file_perms;