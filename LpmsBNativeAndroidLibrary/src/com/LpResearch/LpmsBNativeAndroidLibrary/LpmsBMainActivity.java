/***********************************************************************
** Copyright (C) 2013 LP-Research
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
import java.util.HashMap;
import java.util.Map;

import android.app.*;
import android.app.Activity;
import android.app.ActionBar;
import android.app.Activity;
import android.app.FragmentTransaction;
import android.view.*;
import android.widget.*;
import android.widget.Adapter;
import android.os.*;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.util.*;
import android.content.*;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.bluetooth.*;
import android.graphics.*;
import android.opengl.GLSurfaceView;
import android.annotation.TargetApi;
import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentActivity;
import android.support.v4.view.ViewPager;

import com.LpResearch.LpmsBNativeAndroidLibrary.AppSectionsPagerAdapter;
import com.LpResearch.LpmsBNativeAndroidLibrary.MyFragment;
import com.LpResearch.LpmsBNativeAndroidLibrary.MyFragment.MyFragmentListener;
import com.LpResearch.LpmsBNativeAndroidLibrary.ConnectionFragment.OnConnectListener;

// Main activity. Connects to LPMS-B and displays orientation values
public class LpmsBMainActivity extends FragmentActivity implements ActionBar.TabListener, MyFragmentListener, OnConnectListener
{
	Timer mTimer;
	TextView gyrXText, gyrYText, gyrZText;
	TextView accXText, accYText, accZText;
	TextView magXText, magYText, magZText;
	TextView quatXText, quatYText, quatZText, quatWText;
	TextView eulerXText, eulerYText, eulerZText;
	LpmsBThread lpmsB;
	BluetoothAdapter btAdapter;
	boolean isLpmsBConnected = false;
	int imuStatus = 0;
	
	Handler handler = new Handler();	
	
	private int updateRate = 25;
	private boolean getImage = true;

	Handler updateFragmentsHandler = new Handler();
	
	LpmsBData imuData = new LpmsBData();
	
	private Map<Integer, String> mFragmentMap = new HashMap<Integer, String>();
	private AppSectionsPagerAdapter mAppSectionsPagerAdapter;
	ViewPager mViewPager;	
	
	private Runnable mUpdateFragmentsTask = new Runnable() {
		public void run() {	
			synchronized (imuData) {
				updateFragment(imuData, imuStatus);
			}
			updateFragmentsHandler.postDelayed(mUpdateFragmentsTask, updateRate);
		}
	};	
	
	// Initializes application
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
		setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);		
		
		btAdapter = BluetoothAdapter.getDefaultAdapter();		
		
        setContentView(R.layout.main);
		initializeViews();
    }

	void initializeViews() {
		mAppSectionsPagerAdapter = new AppSectionsPagerAdapter(getSupportFragmentManager());
		
		final ActionBar actionBar = getActionBar();
		
		actionBar.setHomeButtonEnabled(false);
		actionBar.setNavigationMode(ActionBar.NAVIGATION_MODE_TABS);
		
		mViewPager = (ViewPager) findViewById(R.id.pager);
		mViewPager.setAdapter(mAppSectionsPagerAdapter);
		mViewPager.setOffscreenPageLimit(16);
		
		mViewPager.setOnPageChangeListener(new ViewPager.SimpleOnPageChangeListener() {
			@Override
			public void onPageSelected(int position) {
				actionBar.setSelectedNavigationItem(position);
			}
		});

		for (int i = 0; i < mAppSectionsPagerAdapter.getCount(); i++) {
			actionBar.addTab(actionBar.newTab().setText(mAppSectionsPagerAdapter.getPageTitle(i)).setTabListener(this));
		}
		
		mViewPager.setCurrentItem(0);
	}

	public void startUpdateFragments() {
		updateFragmentsHandler.removeCallbacks(mUpdateFragmentsTask);
		updateFragmentsHandler.postDelayed(mUpdateFragmentsTask, 100);
	}

	public void updateFragment(LpmsBData d, int s) {
		int key = mViewPager.getCurrentItem();
		
		MyFragment statusFragment = (MyFragment) getSupportFragmentManager().findFragmentByTag(mFragmentMap.get(key));
		
		if (statusFragment != null) {
			statusFragment.updateView(d, s);
		} else {
		}
	}

	void stopUpdateFragments() {
		updateFragmentsHandler.removeCallbacks(mUpdateFragmentsTask);
	}

	// Timer method that is periodically called every 100ms to display data
	private void timerMethod() {
		handler.post(new Runnable() {
			public void run() {	
				if (lpmsB != null && isLpmsBConnected == true) {
					// Retrieves data from LPMS-B sensor
					synchronized (imuData) {
						imuData = lpmsB.getLpmsBData();
					}
				}
			}
		});
	}
	
	@Override
	public void onAttachFragment(Fragment fragment) {
		super.onAttachFragment(fragment);
		mFragmentMap.put(((MyFragment) fragment).getMyFragmentTag(), fragment.getTag());
	}	
	
	@Override
	public void onUserInput(int input, String data) {
	}	
	
	@Override
	public void onTabUnselected(ActionBar.Tab tab, FragmentTransaction fragmentTransaction) {
	}

	@Override
	public void onTabSelected(ActionBar.Tab tab, FragmentTransaction fragmentTransaction) {
		mViewPager.setCurrentItem(tab.getPosition());
	}

	@Override
	public void onTabReselected(ActionBar.Tab tab, FragmentTransaction fragmentTransaction) {
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
		}, 25, 25);	
	
        super.onStart();
    }
	
	// Everytime the activity is resumed re-connect to LPMS-B
    @Override
    protected void onResume() {	
		startUpdateFragments();	
	
        super.onResume();
    }
	
	// Called when activity is paused or screen orientation changes
    @Override
    protected void onPause() {
		// Disconnects LPMS-B
		if (lpmsB != null && isLpmsBConnected == true) {
			isLpmsBConnected = false;
			lpmsB.close();
			imuStatus = 0;
		}
	
        super.onPause();
    }
	
    @Override
	public void onConnect(String address) {
		if (isLpmsBConnected == true) return;
	
		lpmsB = new LpmsBThread(btAdapter);
		
		lpmsB.setAcquisitionParameters(true, true, true, true, true, true);			
		lpmsB.connect(address, 0);
		
		isLpmsBConnected = true;
		imuStatus = 1;
		
		Log.d("lpms", "Connected");
	}
	
    @Override
	public void onDisconnect() {
		if (isLpmsBConnected == false) return;
		
		isLpmsBConnected = false;
		if (lpmsB != null) lpmsB.close();
		imuStatus = 0;
	}
}