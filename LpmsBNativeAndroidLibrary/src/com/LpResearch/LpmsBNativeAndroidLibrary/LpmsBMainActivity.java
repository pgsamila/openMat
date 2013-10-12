/***********************************************************************
** Copyright (C) 2012 LP-Research
** All rights reserved.
** Contact: LP-Research (klaus@lp-research.com)
**
** Redistribution and use in source and binary forms, with 
** or without modification, are permitted provided that the 
** following conditions are met:
**
** Redistributions of source code must retain the above copyright 
** notice, this list of conditions and the following disclaimer.
** Redistributions in binary form must reproduce the above copyright 
** notice, this list of conditions and the following disclaimer in 
** the documentation and/or other materials provided with the 
** distribution.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
** FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************************/

/* 	Before using this application please ensure the following:
	1. 	The Bluetooth ID in this program equals the Bluetooth ID of your 
		LPMS-B sensor (LpmsBThread.connect)
	2. 	The data acquisition settings correspond to the settings in 
		the LpmsControl application (LpmsBThread.setAcquisitionParameters) */

package com.LpResearch.LpmsBNativeAndroidLibrary;

import java.net.*;
import java.util.*;
import java.io.*;
import java.text.*;
import java.lang.*;

import android.app.*;
import android.view.*;
import android.widget.*;
import android.os.*;
import android.util.*;
import android.content.*;
import android.content.pm.ActivityInfo;
import android.bluetooth.*;
import android.graphics.*;

// Main activity. Connects to LPMS-B and displays orientation values
public class LpmsBMainActivity extends Activity
{
	BluetoothAdapter mAdapter;
	LpmsBThread mLpmsB;
	Timer mTimer;
	TextView gyrXText, gyrYText, gyrZText;
	TextView accXText, accYText, accZText;
	TextView magXText, magYText, magZText;
	TextView quatXText, quatYText, quatZText, quatWText;
	TextView eulerXText, eulerYText, eulerZText;
	
	Handler handler = new Handler();	
	
	// Initializes application
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
		
		setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        setContentView(R.layout.main);	
		
		// Associates TextViews with resource identifiers for data display
		gyrXText = (TextView) findViewById(R.id.gyrXText);
		gyrYText = (TextView) findViewById(R.id.gyrYText);	
		gyrZText = (TextView) findViewById(R.id.gyrZText);
		accXText = (TextView) findViewById(R.id.accXText);
		accYText = (TextView) findViewById(R.id.accYText);	
		accZText = (TextView) findViewById(R.id.accZText);
		magXText = (TextView) findViewById(R.id.magXText);
		magYText = (TextView) findViewById(R.id.magYText);	
		magZText = (TextView) findViewById(R.id.magZText);
		quatXText = (TextView) findViewById(R.id.quatXText);
		quatYText = (TextView) findViewById(R.id.quatYText);	
		quatZText = (TextView) findViewById(R.id.quatZText);
		quatWText = (TextView) findViewById(R.id.quatWText);
		eulerXText = (TextView) findViewById(R.id.eulerXText);
		eulerYText = (TextView) findViewById(R.id.eulerYText);	
		eulerZText = (TextView) findViewById(R.id.eulerZText);
		
		// Sets number font to monospace
		gyrXText.setTypeface(Typeface.MONOSPACE);
		gyrYText.setTypeface(Typeface.MONOSPACE);
		gyrZText.setTypeface(Typeface.MONOSPACE);
		accXText.setTypeface(Typeface.MONOSPACE);
		accYText.setTypeface(Typeface.MONOSPACE);	
		accZText.setTypeface(Typeface.MONOSPACE);
		magXText.setTypeface(Typeface.MONOSPACE);
		magYText.setTypeface(Typeface.MONOSPACE);
		magZText.setTypeface(Typeface.MONOSPACE);
		quatXText.setTypeface(Typeface.MONOSPACE);
		quatYText.setTypeface(Typeface.MONOSPACE);
		quatZText.setTypeface(Typeface.MONOSPACE);
		quatWText.setTypeface(Typeface.MONOSPACE);
		eulerXText.setTypeface(Typeface.MONOSPACE);
		eulerYText.setTypeface(Typeface.MONOSPACE);
		eulerZText.setTypeface(Typeface.MONOSPACE);				
		
		// Gets default Bluetooth adapter
		mAdapter = BluetoothAdapter.getDefaultAdapter();				
    }

	// Timer method that is periodically called every 100ms to display data
	private void timerMethod()
	{
		handler.post(new Runnable() {
			public void run() {
				// Adjusts decimal format to keep numbers in line
				DecimalFormat f0 = new DecimalFormat(" 000.00;-000.00");			
				
				// Retrieves data from LPMS-B sensor
				LpmsBData d = mLpmsB.getLpmsBData();
				
				// Dsiplays data in TextViews
				gyrXText.setText(f0.format(d.gyr[0]));
				gyrYText.setText(f0.format(d.gyr[1]));
				gyrZText.setText(f0.format(d.gyr[2]));

				accXText.setText(f0.format(d.acc[0]));
				accYText.setText(f0.format(d.acc[1]));
				accZText.setText(f0.format(d.acc[2]));

				magXText.setText(f0.format(d.mag[0]));
				magYText.setText(f0.format(d.mag[1]));
				magZText.setText(f0.format(d.mag[2]));

				quatXText.setText(f0.format(d.quat[0]));
				quatYText.setText(f0.format(d.quat[1]));
				quatZText.setText(f0.format(d.quat[2]));
				quatWText.setText(f0.format(d.quat[3]));

				eulerXText.setText(f0.format(d.euler[0]));
				eulerYText.setText(f0.format(d.euler[1]));
				eulerZText.setText(f0.format(d.euler[2]));
			}
		});
	}
	
	// Called when acvtivity is started 
    @Override
    protected void onStart() {
		// Creates timer and initalizes to 100ms period
		mTimer = new Timer();
		mTimer.schedule(new TimerTask() {
			@Override
			public void run() {
				timerMethod();
			}
		}, 100, 100);	
	
        super.onStart();
    }
	
	// Everytime the activity is resumed re-connect to LPMS-B
    @Override
    protected void onResume() {
		if (mAdapter != null) {
			// Creates LPMS-B controller object using Bluetooth adapter mAdapter
			mLpmsB = new LpmsBThread(mAdapter);
			
			// Sets acquisition paramters (Must be the same as set in LpmsControl app)
			mLpmsB.setAcquisitionParameters(true, true, true, true, true, false);			
			
			// Tries to connect to LPMS-B with Bluetooth ID 00:06:66:48:E3:7A
			mLpmsB.connect("00:06:66:48:E3:62", 0);
		}	
	
        super.onResume();
    }
	
	// Called when activity is paused or screen orientation changes
    @Override
    protected void onPause() {
		// Disconnects LPMS-B
		if (mLpmsB != null) mLpmsB.close();
	
        super.onPause();
    }
}