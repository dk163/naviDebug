/*
 * Copyright (C) 2013 The Android Open Source Project
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

package com.android.server.wm;

import android.view.InputChannel;
import android.view.InputDevice;
import android.view.InputEvent;
import android.view.InputEventReceiver;
import android.view.MotionEvent;
import android.view.WindowManagerPolicy.PointerEventListener;
import android.util.Slog;
import android.util.Log;
import com.chinatsp.mcumanager.McuManager;


import com.android.server.UiThread;

import java.util.ArrayList;

public class PointerEventDispatcher extends InputEventReceiver {
    ArrayList<PointerEventListener> mListeners = new ArrayList<PointerEventListener>();
    PointerEventListener[] mListenersArray = new PointerEventListener[0];
    private long mLastTouchTime = 0;
	private long mCurrentTouchTime = 0;
	private McuManager.ScreenTouchListener mListener = null;
    public PointerEventDispatcher(InputChannel inputChannel) {
        super(inputChannel, UiThread.getHandler().getLooper());
		 
    }

    @Override
    public void onInputEvent(InputEvent event) {
        try {
            if (event instanceof MotionEvent
                    && (event.getSource() & InputDevice.SOURCE_CLASS_POINTER) != 0) {
                final MotionEvent motionEvent = (MotionEvent)event;
                PointerEventListener[] listeners;
                synchronized (mListeners) {
                    if (mListenersArray == null) {
                        mListenersArray = new PointerEventListener[mListeners.size()];
                        mListeners.toArray(mListenersArray);
                    }
                    listeners = mListenersArray;
                }
                for (int i = 0; i < listeners.length; ++i) {
                    listeners[i].onPointerEvent(motionEvent);
                }	
				if(motionEvent.getAction() == MotionEvent.ACTION_UP){
				//	mCurrentTouchTime = System.currentTimeMillis();
				//	if((mCurrentTouchTime - mLastTouchTime)>500){
				       if(mListener == null){
					   mListener = McuManager.getInstance().getScreenTouchListener();
					  }else if(mListener!=null){
				        mListener.onScreenTouched();
					 	}
				//	  mLastTouchTime = mCurrentTouchTime;
				//   }
					
				}
            }
        } finally {
            finishInputEvent(event, false);
        }
    }

    /**
     * Add the specified listener to the list.
     * @param listener The listener to add.
     */
    public void registerInputEventListener(PointerEventListener listener) {
        synchronized (mListeners) {
            if (mListeners.contains(listener)) {
                throw new IllegalStateException("registerInputEventListener: trying to register" +
                        listener + " twice.");
            }
            mListeners.add(listener);
            mListenersArray = null;
        }
    }

    /**
     * Remove the specified listener from the list.
     * @param listener The listener to remove.
     */
    public void unregisterInputEventListener(PointerEventListener listener) {
        synchronized (mListeners) {
            if (!mListeners.contains(listener)) {
                throw new IllegalStateException("registerInputEventListener: " + listener +
                        " not registered.");
            }
            mListeners.remove(listener);
            mListenersArray = null;
        }
    }
}
