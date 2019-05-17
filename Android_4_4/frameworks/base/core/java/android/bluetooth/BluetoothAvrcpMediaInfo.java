/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package android.bluetooth;

import android.os.Parcel;
import android.os.Parcelable;

/**
 * This class encapsulates the basic information delivered from remote
 * device(AVRCP TG).
 *
 */
public class BluetoothAvrcpMediaInfo implements Parcelable {
	/**
	 * The title of the media currently playing in remote device.
	 */
	public String meidaTitle;
	/**
	 * The author name of the media currently playing in remote device.
	 */
	public String artistName;

	/**
	 * The album name of the media currently playing in remote device.
	 */
	public String albumName;

	/**
	 * The playing time in millisecond of the media currently playing in remote
	 * device. 150000 for 2 minutes 30 seconds for example.
	 */
	public long songLength;

	public BluetoothAvrcpMediaInfo() {
	}

	public BluetoothAvrcpMediaInfo(Parcel source) {
		meidaTitle = source.readString();
		artistName = source.readString();
		albumName = source.readString();
		songLength = source.readLong();
	}

	@Override
	public int describeContents() {
		return 0;
	}

	@Override
	public void writeToParcel(Parcel dest, int flag) {
		String content = (meidaTitle == null ? "" : meidaTitle);
		dest.writeString(content);
		content = (artistName == null ? "" : artistName);
		dest.writeString(content);
		content = (albumName == null ? "" : albumName);
		dest.writeString(content);
		dest.writeLong(songLength);
	}

	public static final Parcelable.Creator<BluetoothAvrcpMediaInfo> CREATOR = new Parcelable.Creator<BluetoothAvrcpMediaInfo>() {

		@Override
		public BluetoothAvrcpMediaInfo createFromParcel(Parcel source) {
			return new BluetoothAvrcpMediaInfo(source);
		}

		@Override
		public BluetoothAvrcpMediaInfo[] newArray(int size) {
			return new BluetoothAvrcpMediaInfo[size];
		}

	};

	/* TODO more attributes may be included in needed. */
}
