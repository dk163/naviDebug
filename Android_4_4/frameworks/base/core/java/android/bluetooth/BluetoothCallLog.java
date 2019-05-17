package android.bluetooth;

import android.os.Parcel;
import android.os.Parcelable;

public class BluetoothCallLog implements Parcelable {
	public static final int CALLLOG_TYPE_OUTGOING = 0x01;
	public static final int CALLLOG_TYPE_INCOMING = 0x02;
	public static final int CALLLOG_TYPE_MISSED = 0x03;
	public static final int CALLLOG_TYPE_ALL = 0x04;

	public int type; /* mandatory */
	public String number; /* mandatory */
	public String name;
	public String firstName;
	public String middleName;
	public String lastName;
	public String time;
    public String vcardRawData;

	public BluetoothCallLog() {
        vcardRawData = new String();
	}

	public BluetoothCallLog(Parcel source) {
		type = source.readInt();
		number = source.readString();
		name = source.readString();
		firstName = source.readString();
		middleName = source.readString();
		lastName = source.readString();
		time = source.readString();
		vcardRawData = source.readString();
	}

	@Override
	public int describeContents() {
		return 0;
	}

	@Override
	public void writeToParcel(Parcel dest, int flag) {
		dest.writeInt(type);
		String content = (number == null ? "" : number);
		dest.writeString(content);
		content = (name == null ? "" : name);
		dest.writeString(content);
		content = (firstName == null ? "" : firstName);
		dest.writeString(content);
		content = (middleName == null ? "" : middleName);
		dest.writeString(content);
		content = (lastName == null ? "" : lastName);
		dest.writeString(content);
		content = (time == null ? "" : time);
		dest.writeString(content);
		content = (vcardRawData == null ? "" : vcardRawData);
        dest.writeString(vcardRawData);
	}

	public static final Parcelable.Creator<BluetoothCallLog> CREATOR = new Parcelable.Creator<BluetoothCallLog>() {

		public BluetoothCallLog createFromParcel(Parcel source) {
			return new BluetoothCallLog(source);
		}

		public BluetoothCallLog[] newArray(int size) {
			return new BluetoothCallLog[size];
		}

	};
}
