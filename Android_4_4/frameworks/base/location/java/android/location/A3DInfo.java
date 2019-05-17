/******************************************************************************

  Copyright (C), 2001-2018, DCN Co., Ltd.

 ******************************************************************************
  File Name     : A3DInfo.java
  Version       : Initial Draft
  Author        : kang
  Created       : 2018/10/24
  Last Modified :
  Description   : A3Dinfo
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

public class A3DInfo implements Parcelable{

    //@A3D 1539200 4 100 0.966000 -0.001000 0.029000 !1F
    //private String type = "A3D";
    private long ticktime = 0; /* 时间戳 */
    private int axis = 0;           /* 轴数 */
    private int interval = 0;       /* 间隔 */
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

    public float getZ() {
        return z;
    }

    public float getY() {
        return y;
    }

    public float getX() {
        return x;
    }
    
    public A3DInfo() {
    }

    /**
     * Construct a new A3DInfo object that is copied from an existing one.
     */
    public A3DInfo(A3DInfo info) {
        set(info);
    }
    
    /**
     * Sets the contents of the A3DInfo to the values from the given location.
     */
    public void set(A3DInfo info) {
        ticktime = info.ticktime;
        axis = info.axis;
        interval = info.interval;
        z = info.z;
        y = info.y;
        x = info.x;
    }
    
    public static final Parcelable.Creator<A3DInfo> CREATOR =
        new Parcelable.Creator<A3DInfo>() {
        @Override
        public A3DInfo createFromParcel(Parcel in) {
            A3DInfo A3D = new A3DInfo();
            A3D.ticktime = in.readLong();
            A3D.axis = in.readInt();
            A3D.interval = in.readInt();
            A3D.z = in.readFloat();
            A3D.y = in.readFloat();
            A3D.x = in.readFloat();
           
            return A3D;
        }

        @Override
        public A3DInfo[] newArray(int size) {
            return new A3DInfo[size];
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
       parcel.writeFloat(z);
       parcel.writeFloat(y);
       parcel.writeFloat(x);
    }

    @Override
    public String toString() {
        StringBuilder s = new StringBuilder();
        s.append("A3DInfo[ type=A3D ");
        s.append(",ticktime=").append(this.ticktime);
        s.append(",axis=").append(this.axis);
        s.append(",interval=").append(this.interval);
        s.append(",z=").append(this.z);
        s.append(",y=").append(this.y);
        s.append(",x=").append(this.x);
        
        s.append(']');
        return s.toString();
    }
}
