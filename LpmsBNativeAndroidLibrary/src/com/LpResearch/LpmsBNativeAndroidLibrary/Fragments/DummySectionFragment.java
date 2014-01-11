package com.LpResearch.LpmsBNativeAndroidLibrary.Fragments;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.EditText;
import android.widget.TextView;

import com.LpResearch.LpmsBNativeAndroidLibrary.R;

public class DummySectionFragment extends MyFragment {
	final static String TAG = "DummySectionFragment";
	final int FRAGMENT_TAG = 4; 
	
    public static final String ARG_SECTION_NUMBER = "section_number";

	TextView modeText;
 	EditText ipAddressEdit;
 	View rootView;
 	
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
    	Log.d(TAG, "onCreateView()");
        rootView = inflater.inflate(R.layout.fragment_section_dummy, container, false);
        Bundle args = getArguments(); 
      
        return rootView;
    }
    
	@Override
	public void onAttach(Activity activity) {
		super.onAttach(activity);
		Log.d(TAG, "onAttach()");	 
	}

	@Override
	public void onStart() {
		super.onStart();
		Log.d(TAG, "onStart()");
	}

	@Override
	public void onResume() { 
		super.onResume();
		Log.d(TAG, "onResume()");
	}

	@Override
	public void onPause() {
		ipAddressEdit = null;
		rootView = null;
		super.onPause();
		Log.d(TAG, "onPause()");
	}

	@Override
	public void onStop() {
		super.onStop();
		Log.d(TAG, "onStop()");
	}

	@Override
	public void onDestroyView() {
		super.onDestroyView();
		
		Log.d(TAG, "onDestroyView()");
	}

	@Override
	public void onDestroy() {
		super.onDestroy();
		Log.d(TAG, "onDestroy()");

	}

	@Override
	public void onDetach() {
		super.onDetach();
		Log.d(TAG, "onDetach()");
	}


	@Override
	public int getMyFragmentTag() {
		return  FRAGMENT_TAG; 
	}


	@Override
	public void updateView() {		
	}
}