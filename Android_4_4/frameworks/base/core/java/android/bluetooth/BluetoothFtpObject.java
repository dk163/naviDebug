package android.bluetooth;

import android.os.Parcel;
import android.os.Parcelable;

public class BluetoothFtpObject implements Parcelable {
	public int attr; /* mandatory */
	public long size; /* mandatory */
	public long time; /* mandatory */
	public String name; /* mandatory */

	public BluetoothFtpObject() {
		attr = 0;
		size = 0;
		time = 0;
		name = "/";
	}

	public BluetoothFtpObject(Parcel source) {
		attr = source.readInt();
		size = source.readLong();
		time = source.readLong();
		name = source.readString();
	}

	@Override
	public int describeContents() {
		return 0;
	}

	@Override
	public void writeToParcel(Parcel dest, int flag) {
		dest.writeInt(attr);
		dest.writeLong(size);
		dest.writeLong(time);
		String content = (name == null ? "" : name);
		dest.writeString(content);
	}

	public static final Parcelable.Creator<BluetoothFtpObject> CREATOR = new Parcelable.Creator<BluetoothFtpObject>() {

		public BluetoothFtpObject createFromParcel(Parcel source) {
			return new BluetoothFtpObject(source);
		}

		public BluetoothFtpObject[] newArray(int size) {
			return new BluetoothFtpObject[size];
		}

	};
}
