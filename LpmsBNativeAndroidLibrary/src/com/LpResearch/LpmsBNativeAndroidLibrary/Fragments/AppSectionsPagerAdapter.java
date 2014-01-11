package com.LpResearch.LpmsBNativeAndroidLibrary.Fragments;

import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentManager;
import android.support.v4.app.FragmentPagerAdapter;

import android.util.Log;

public class AppSectionsPagerAdapter extends FragmentPagerAdapter {
	final String TAG ="AppSectionsPagerAdapter"; 

	public AppSectionsPagerAdapter(FragmentManager fm) {
		super(fm); 
		Log.d(TAG,"AppSectionsPagerAdapter()");
	}

	@Override
	public Fragment getItem(int i) {
		switch (i) {
		case 0:
			Log.d("AppSectionsPagerAdapter", "new ImageFragment()"); 	 
			return new DummySectionFragment();
			
		case 1:
			Log.d("AppSectionsPagerAdapter", "new ControlSectionFragment()");  
			return new DummySectionFragment();
			
		case 2:
			Log.d("AppSectionsPagerAdapter", "new ImageFragment()"); 	 
			return new DummySectionFragment();
	 
		default:
			Log.d("AppSectionsPagerAdapter", "new Dummy Fragment()"); 
			return new DummySectionFragment();
		}
	}

	@Override
	public int getCount() {
		return 3;
	}

	@Override
	public CharSequence getPageTitle(int position) {
		switch (position) {
		case 0:
			return "Advanced Control"; 
		case 1:
			return "Control";
		case 2:
			return "Image"; 
		}
		return "null"; 
	}

}
