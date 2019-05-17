package android.bluetooth;

import android.os.Parcel;
import android.os.Parcelable;

import java.util.UUID;

public class BluetoothSms implements Parcelable {
	public static final int SMS_TYPE_RECEIVED = 0x01;
	public static final int SMS_TYPE_SENT = 0x02;
	public static final int SMS_TYPE_DRAFT = 0x03;
	public static final int SMS_TYPE_DELETED = 0x04;

	private UUID uuid;
	public String subject;
	public String dateTime;
	public String sendNumber;
	public String sendName;
	public String recipientNumber;
	public boolean readed;
	public int size;
	public int type;
	public byte content[];

	public BluetoothSms(UUID uuid) {
		this.uuid = uuid;
	}

	public BluetoothSms(Parcel source) {
		String ustr = source.readString();
		uuid = UUID.fromString(ustr);
		subject = source.readString();
		dateTime = source.readString();
		sendNumber = source.readString();
		sendName = source.readString();
		recipientNumber = source.readString();
		readed = source.readInt() == 1 ? true : false;
		size = source.readInt();
		type = source.readInt();
		int len = source.readInt();

		if (len > 0) {
			content = new byte[len];
			source.readByteArray(content);
		}

	}

    public UUID getUuid() {
        return uuid;
    }

	@Override
	public int describeContents() {
		return 0;
	}

	@Override
	public void writeToParcel(Parcel dest, int flag) {
		dest.writeString(uuid.toString());
		dest.writeString(subject == null ? "" : subject);
		dest.writeString(dateTime == null ? "" : dateTime);
		dest.writeString(sendNumber == null ? "" : sendNumber);
		dest.writeString(sendName == null ? "" : sendName);
		dest.writeString(recipientNumber == null ? "" : recipientNumber);
		dest.writeInt(readed ? 1 : 0);
		dest.writeInt(size);
		dest.writeInt(type);
		int len = 0;
		if (content != null) {
			len = content.length;
		}
		dest.writeInt(len);
		if (len > 0) {
			dest.writeByteArray(content);
		}
	}

	public static final Parcelable.Creator<BluetoothSms> CREATOR = new Parcelable.Creator<BluetoothSms>() {

		@Override
		public BluetoothSms createFromParcel(Parcel source) {
			return new BluetoothSms(source);
		}

		@Override
		public BluetoothSms[] newArray(int size) {
			return new BluetoothSms[size];
		}

	};

	@Override
	public String toString() {
		StringBuffer sb = new StringBuffer();
		sb.append("uuid:");
		sb.append(uuid == null ? "null" : uuid.toString());
		sb.append(",subject:");
		sb.append(subject == null ? "null" : subject);
		sb.append(",time:");
		sb.append(dateTime == null ? "null" : dateTime);
		sb.append(",sendName:");
		sb.append(sendName == null ? "null" : sendName);
		sb.append(",sendNumber:");
		sb.append(sendNumber == null ? "null" : sendNumber);
		sb.append(",recipientNumber:");
		sb.append(recipientNumber == null ? "null" : recipientNumber);
		sb.append(",readed:");
		sb.append(readed);
		sb.append(",size:");
		sb.append(size);
		sb.append(",type:");
		sb.append(type);
		sb.append(",content:");
		sb.append(content == null ? "null" : new String(content));

		return sb.toString();
	}
}
