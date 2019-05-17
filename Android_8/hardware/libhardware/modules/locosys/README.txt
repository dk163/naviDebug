1. 确认 :    (手册 2.A、2.B)
        a. Android.bp + locosys_gps.c 在此文件夹下 : android_build/hardware/libhardware/gps/    
         b. android_build/hardware/libhardware/Android.mk 加入 "gps" 

2. 编译 GNSS HIDL :
    执行source build/envsetup.sh 与lunch sabresd_6dq-eng 后，再去编译android_build/hardware/interfaces/gnss/X.X/default 该文件
    夹，方可编译出HIDL 所需文件android.hardware.gnss@1.0-service 与android.hardware.gnss@1.0-impl.so。( X.X 原始版本为1.0 )
    Ex :
    root@ubuntu:~/android_build# source build/envsetup.sh
    root@ubuntu:~/android_build# lunch sabresd_6dq-eng
    root@ubuntu:~/android_build# mmm hardware/interfaces/gnss/1.0/default

    ==> 确认 out/.../vendor/bin/hw/android.hardware.gnss@1.0-service 是否为当下编译时间
    ==> 确认 out/.../vendor/lib/hw/android.hardware.gnss@1.0-impl.so 是否为当下编译时间

3. 编译 GPS Lib库 :
    root@ubuntu:~/android_build# mmm hardware/libhardware/modules/gps/

        ==> 确认 out/.../vendor/lib/hw/gps.default.so 是否为当下编译时间

4. 删除  "vendor.img" 
rm -rf out/target/product/sabresd_6dq/vendor.img

    ==> 删除此 vendor.img 目的为确保此 vendor.img 挡案为当前编译完成的数据。

5. 编译集成 安卓系统 :
    make  2>&1 | tee build-log.txt