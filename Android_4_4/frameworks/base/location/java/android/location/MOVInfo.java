/******************************************************************************

  Copyright (C), 2001-2018, DCN Co., Ltd.

 ******************************************************************************
  File Name     : MOVInfo.java
  Version       : Initial Draft
  Author        : kang
  Created       : 2018/12/24
  Last Modified :
  Description   : MOVinfo
  Function List :
  History       :
  1.Date        : 2018/12/24
    Author      : kang
    Modification: Created file

******************************************************************************/

/*----------------------------------------------*
 * external variables                           *
 *----------------------------------------------*/

/*----------------------------------------------*
 * external routine prototypes                  *
 *----------------------------------------------*/

/*----------------------------------------------*
 * internal routine prototypes                  *
 *----------------------------------------------*/

/*----------------------------------------------*
 * project-wide global variables                *
 *----------------------------------------------*/

/*----------------------------------------------*
 * module-wide global variables                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * constants                                    *
 *----------------------------------------------*/

/*----------------------------------------------*
 * macros                                       *
 *----------------------------------------------*/

/*----------------------------------------------*
 * routines' implementations                    *
 *----------------------------------------------*/


package android.location;

import android.os.Bundle;
import android.os.Parcel;
import android.os.Parcelable;
import android.os.SystemClock;
import android.util.Printer;
import android.util.TimeUtils;

import java.text.DecimalFormat;
import java.util.StringTokenizer;

public class MOVInfo implements Parcelable{

    //@MOV 1818501 100 72.646004 !7A
    //private String type = "MOV";
    private long ticktime = 0; /* 时间戳 */
    private int interval = 0;       /* 间隔 */
    private float speed = 0.0f;            /* 车速 */

    public long getTicktime() {
        return ticktime;
    }

    public void setTicktime(long ticktime) {
        this.ticktime = ticktime;
    }

    public int getInterval() {
        return interval;
    }

    public void setInterval(int interval) {
        this.interval = interval;
    }

    public float getSpeed() {
        return speed;
    }

    public void setSpeed(float speed) {
        this.speed = speed;
    }

    public MOVInfo() {
    }

    /**
     * Construct a new MOVInfo object that is copied from an existing one.
     */
    public MOVInfo(MOVInfo info) {
        set(info);
    }
    
    /**
     * Sets the contents of the MOVInfo to the values from the given location.
     */
    public void set(MOVInfo info) {
        ticktime = info.ticktime;
        speed = info.speed;
        interval = info.interval;
    }
    
    public static final Creator<MOVInfo> CREATOR =
        new Creator<MOVInfo>() {
        @Override
        public MOVInfo createFromParcel(Parcel in) {
            MOVInfo MOV = new MOVInfo();
            MOV.ticktime = in.readLong();
            MOV.speed = in.readFloat();
            MOV.interval = in.readInt();
           
            return MOV;
        }

        @Override
        public MOVInfo[] newArray(int size) {
            return new MOVInfo[size];
        }
    };

    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel parcel, int flags) {
       parcel.writeLong(ticktime);
       parcel.writeFloat(speed);
       parcel.writeInt(interval);
    }

    @Override
    public String toString() {
        StringBuilder s = new StringBuilder();
        s.append("MOVInfo[ type=MOV ");
        s.append(",ticktime=").append(this.ticktime);
        s.append(",speed=").append(this.speed);
        s.append(",interval=").append(this.interval);
        
        s.append(']');
        return s.toString();
    }
}
