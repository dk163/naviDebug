/******************************************************************************

  Copyright (C), 2001-2018, DCN Co., Ltd.

 ******************************************************************************
  File Name     : GYRInfo.java
  Version       : Initial Draft
  Author        : kang
  Created       : 2018/10/24
  Last Modified :
  Description   : GYRinfo
  Function List :
  History       :
  1.Date        : 2018/10/24
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


public class GYRInfo implements Parcelable{
    //@GYR 1539300 4 100 42.660000 -0.105000 -0.509000 0.037000 !4E
    //private String type = "GYR";
    private long ticktime; /* 时间戳 */
    private int axis = 0;           /* 轴数 */
    private  int interval = 0;       /* 间隔 */
    private float temp = 0.0f;;         /* 温度 */
    private float z = 0.0f;            /* yaw */
    private float y = 0.0f;            /* roll */
    private float x = 0.0f;            /* pitch */

    public void setTicktime(long ticktime) {
        this.ticktime = ticktime;
    }

    public void setAxis(int axis) {
        this.axis = axis;
    }

    public void setInterval(int interval) {
        this.interval = interval;
    }

    public void setTemp(float temp) {
        this.temp = temp;
    }

    public void setZ(float z) {
        this.z = z;
    }

    public void setY(float y) {
        this.y = y;
    }

    public void setX(float x) {
        this.x = x;
    }

    public long getTicktime() {
        return ticktime;
    }

    public int getAxis() {
        return axis;
    }

    public int getInterval() {
        return interval;
    }

    public float getTemp() {
        return temp;
    }

    public float getZ() {
        return z;
    }

    public float getY() {
        return y;
    }

    public float getX() {
        return x;
    }
    
    public GYRInfo() {
    }

    /**
     * Construct a new GYRInfo object that is copied from an existing one.
     */
    public GYRInfo(GYRInfo l) {
        set(l);
    }

    /**
     * Sets the contents of the GYRInfo to the values from the given location.
     */
    public void set(GYRInfo l) {
        ticktime = l.ticktime;
        axis = l.axis;
        interval = l.interval;
        temp = l.temp;
        z = l.z;
        x = l.x;
        y = l.y;
    }
    
    public static final Parcelable.Creator<GYRInfo> CREATOR =
        new Parcelable.Creator<GYRInfo>() {
        @Override
        public GYRInfo createFromParcel(Parcel in) {
            GYRInfo GYR = new GYRInfo();
            
            GYR.ticktime = in.readLong();
            GYR.axis = in.readInt();
            GYR.interval = in.readInt();
            GYR.temp = in.readFloat();
            GYR.z = in.readFloat();
            GYR.y = in.readFloat();
            GYR.x = in.readFloat();
        
            return GYR;
        }

        @Override
        public GYRInfo[] newArray(int size) {
            return new GYRInfo[size];
        }
    };

    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel parcel, int flags) {
        parcel.writeLong(ticktime);
        parcel.writeInt(axis);
        parcel.writeInt(interval);
        parcel.writeFloat(temp);
        parcel.writeFloat(z);
        parcel.writeFloat(y);
        parcel.writeFloat(x);
    }

    @Override
    public String toString() {
        StringBuilder s = new StringBuilder();
        s.append("GYRInfo[ type=GYR ");
        s.append(",ticktime=").append(this.ticktime);
        s.append(",axis=").append(this.axis);
        s.append(",interval=").append(this.interval);
        s.append(",temperature=").append(this.temp);
        s.append(",z=").append(this.z);
        s.append(",y=").append(this.y);
        s.append(",x=").append(this.x);
        
        s.append(']');
        return s.toString();
    }
}
