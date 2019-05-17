package android.bluetooth;

import android.os.Parcel;
import android.os.Parcelable;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

public class BluetoothContact implements Parcelable {
	public enum NumberType {
		NUMBER_TYPE_HOME,
		NUMBER_TYPE_MSG,
		NUMBER_TYPE_WORK,
		NUMBER_TYPE_PREF,
		NUMBER_TYPE_FAX,
		NUMBER_TYPE_CELL,
		NUMBER_TYPE_VIDEO,
		NUMBER_TYPE_PAGER,
		NUMBER_TYPE_BBS,
		NUMBER_TYPE_MODEM,
		NUMBER_TYPE_CAR,
		NUMBER_TYPE_ISDN,
		NUMBER_TYPE_PCS,
		NUMBER_TYPE_VOICE,
		NUMBER_TYPE_OTHER,
	}

	public String name; /* mandatory */
	public String firstName;
	public String middleName;
	public String lastName;
	public EnumMap<NumberType, List<String>> numberList = new EnumMap<NumberType, List<String>>(
			NumberType.class);

    public String vcardRawData;
    public byte[] photoData;
	public BluetoothContact() {
        vcardRawData = new String();
	}

	public BluetoothContact(Parcel source) {
		name = source.readString();
        firstName = source.readString();
        middleName = source.readString();
        lastName = source.readString();
        vcardRawData = source.readString();//new String(source.readString);
		source.readMap(numberList, BluetoothContact.class.getClassLoader());
        int len = source.readInt();
        if(len > 0) {
            photoData = new byte[len];
            source.readByteArray(photoData);
        }
	}

	public void AddNumber(NumberType type, String number) {
		List<String> numbers = numberList.get(type);
		if (numbers == null) {
			numbers = new ArrayList<String>();
			numberList.put(type, numbers);
		}
		numbers.add(number);
	}

	public List<String> getCellNumbers() {
		return numberList.get(NumberType.NUMBER_TYPE_CELL);
	}

	public List<String> getWorkNumbers() {
		return numberList.get(NumberType.NUMBER_TYPE_WORK);
	}

	public List<String> getHomeNumbers() {
		return numberList.get(NumberType.NUMBER_TYPE_HOME);
	}
	public List<String> getOtherNumbers() {
		return numberList.get(NumberType.NUMBER_TYPE_OTHER);
	}

	@Override
	public int describeContents() {
		return 0;
	}

	@Override
	public void writeToParcel(Parcel dest, int flags) {
		String content = (name == null ? "" : name);
		dest.writeString(content);
		dest.writeString(firstName == null ? "" : firstName);
		dest.writeString(middleName == null ? "" : middleName);
		dest.writeString(lastName == null ? "" : lastName);
		content = (vcardRawData == null ? "" : vcardRawData);
        dest.writeString(vcardRawData);
		dest.writeMap(numberList);
		int len = 0;
        if(photoData != null) {
            len = photoData.length;
        }
        dest.writeInt(len);
        if (len > 0) {
            dest.writeByteArray(photoData);
        }

	}

	public static final Parcelable.Creator<BluetoothContact> CREATOR = new Parcelable.Creator<BluetoothContact>() {

		public BluetoothContact createFromParcel(Parcel source) {
			return new BluetoothContact(source);
		}

		public BluetoothContact[] newArray(int size) {
			return new BluetoothContact[size];
		}

	};
}
