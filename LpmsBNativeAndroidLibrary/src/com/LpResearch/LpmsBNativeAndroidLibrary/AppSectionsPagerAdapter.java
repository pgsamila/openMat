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
			return new ThreeDeeCubeFragment();
			
		case 1:
			return new DataFragment();

		case 2:
			return new MoreDataFragment();
				 
		default:
			return new DataFragment();
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
			return "3D Cube"; 
			
		case 1:
			return "Raw Data";
			
		case 2:
			return "Orientation Data";
		}

		return "null"; 
	}

}
