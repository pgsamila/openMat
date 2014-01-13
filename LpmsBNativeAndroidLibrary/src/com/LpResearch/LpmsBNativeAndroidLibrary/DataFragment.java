package com.LpResearch.LpmsBNativeAndroidLibrary;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.EditText;
import android.widget.TextView;
import android.opengl.GLSurfaceView;

public class DataFragment extends MyFragment {
	final static String TAG = "DataFragment";
	final int FRAGMENT_TAG = 1; 
	
    public static final String ARG_SECTION_NUMBER = "section_number";

 	View rootView;
 	
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
    	Log.d("lpms", "onCreateView()");

        rootView = inflater.inflate(R.layout.fragment_section_dummy, container, false);
        Bundle args = getArguments(); 

        return rootView;
    }
    
	@Override
	public void onAttach(Activity activity) {
		super.onAttach(activity);
		Log.d("lpms", "onAttach()");	 
	}

	@Override
	public void onStart() {
		super.onStart();
		Log.d("lpms", "onStart()");
	}

	@Override
	public void onResume() { 
		super.onResume();
		Log.d("lpms", "onResume()");
	}

	@Override
	public void onPause() {
		super.onPause();
		Log.d("lpms", "onPause()");
	}

	@Override
	public void onStop() {
		super.onStop();
		Log.d("lpms", "onStop()");
	}

	@Override
	public void onDestroyView() {
		super.onDestroyView();
		Log.d("lpms", "onDestroyView()");
	}

	@Override
	public void onDestroy() {
		super.onDestroy();
		Log.d("lpms", "onDestroy()");

	}

	@Override
	public void onDetach() {
		super.onDetach();
		Log.d("lpms", "onDetach()");
	}


	@Override
	public int getMyFragmentTag() {
		return FRAGMENT_TAG; 
	}


	@Override
	public void updateView(LpmsBData d) {
		// Log.d("lpms", "updating data view");
	}
}