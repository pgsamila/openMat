package com.LpResearch.LpmsBNativeAndroidLibrary;

import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentManager;
import android.support.v4.app.FragmentPagerAdapter;

import android.util.Log;

public class AppSectionsPagerAdapter extends FragmentPagerAdapter {
	
	public AppSectionsPagerAdapter(FragmentManager fm) {
		super(fm);
	}

	@Override
	public Fragment getItem(int i) {
		switch (i) {
		case 0:
			return new ConnectionFragment();

		case 3:
			return new MoreDataFragment();
			
		case 2:
			return new DataFragment();
			
		case 1:
			return new ThreeDeeCubeFragment();
			
		default:
			return new DataFragment();
		}
	}

	@Override
	public int getCount() {
		return 4;
	}

	@Override
	public CharSequence getPageTitle(int position) {
		switch (position) {					
		case 0:
			return "Connection";		
			
		case 3:
			return "Orientation Data";
			
		case 2:
			return "Raw Data";
			
		case 1:
			return "3D Cube";
		}

		return "null"; 
	}

}
