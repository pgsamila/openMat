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

public class ThreeDeeCubeFragment extends MyFragment {
	final static String TAG = "3dCubeFragment";
	final int FRAGMENT_TAG = 1; 
	
    public static final String ARG_SECTION_NUMBER = "section_number";

 	View rootView;

	LpmsBSurfaceView glView;
 	
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        rootView = inflater.inflate(R.layout.fragment_section_dummy, container, false);
        Bundle args = getArguments(); 
      
		glView = new LpmsBSurfaceView(getActivity());

        return glView;
    }
    
	@Override
	public void onAttach(Activity activity) {
		super.onAttach(activity);
	}

	@Override
	public void onStart() {
		super.onStart();
	}

	@Override
	public void onResume() { 
		super.onResume();
	}

	@Override
	public void onPause() {
		super.onPause();
	}

	@Override
	public void onStop() {
		super.onStop();
	}

	@Override
	public void onDestroyView() {
		super.onDestroyView();
	}

	@Override
	public void onDestroy() {
		super.onDestroy();
	}

	@Override
	public void onDetach() {
		super.onDetach();
	}

	@Override
	public int getMyFragmentTag() {
		return FRAGMENT_TAG; 
	}

	@Override
	public void updateView(LpmsBData d, ImuStatus s) {
		if (s.measurementStarted == false) return;	

		glView.lmRenderer.q[0] = d.quat[0];
		glView.lmRenderer.q[1] = d.quat[1];
		glView.lmRenderer.q[2] = d.quat[2];
		glView.lmRenderer.q[3] = d.quat[3];		
		
		glView.requestRender();
	}
}