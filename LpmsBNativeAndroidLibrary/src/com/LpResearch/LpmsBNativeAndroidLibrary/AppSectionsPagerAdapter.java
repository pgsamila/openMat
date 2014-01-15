package com.LpResearch.LpmsBNativeAndroidLibrary;

import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentManager;
import android.support.v4.app.FragmentPagerAdapter;

import android.util.Log;

public class AppSectionsPagerAdapter extends FragmentPagerAdapter {
	
	public AppSectionsPagerAdapter(FragmentManager fm) {
		super(fm); 
		Log.d("lpms", "AppSectionsPagerAdapter()");
	}

	@Override
	public Fragment getItem(int i) {
		switch (i) {
		case 0:
			Log.d("AppSectionsPagerAdapter", "new ImageFragment()"); 	 
			return new ThreeDeeCubeFragment();
			
		case 1:
			Log.d("AppSectionsPagerAdapter", "new ControlSectionFragment()");  
			return new DataFragment();
				 
		default:
			Log.d("AppSectionsPagerAdapter", "new ControlSectionFragment()");  
			return new DataFragment();
		}
	}

	@Override
	public int getCount() {
		return 2;
	}

	@Override
	public CharSequence getPageTitle(int position) {
		switch (position) {
		case 0:
			return "3D Cube"; 
		case 1:
			return "Data View";
		}

		return "null"; 
	}

}
