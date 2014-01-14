package com.LpResearch.LpmsBNativeAndroidLibrary;

import java.util.ArrayList;
import java.util.List;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.EditText;
import android.widget.TextView;
import android.opengl.GLSurfaceView;
import android.graphics.Color;

import android.widget.LinearLayout;
import android.widget.Toast;

import java.util.HashMap;
import java.util.Map;

import com.jjoe64.graphview.BarGraphView;
import com.jjoe64.graphview.GraphView;
import com.jjoe64.graphview.GraphView.GraphViewData;
import com.jjoe64.graphview.GraphViewSeries;
import com.jjoe64.graphview.LineGraphView;

public class DataFragment extends MyFragment {
	final static String TAG = "DataFragment";
	final int FRAGMENT_TAG = 1; 
	
    public static final String ARG_SECTION_NUMBER = "section_number";

 	View rootView;

	private GraphView graphView1;
	private GraphViewSeries exampleSeries1;
	private List<GraphViewData> seriesX;
	int dataCount = 1;	
 	
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
    	Log.d("lpms", "onCreateView()");

        rootView = inflater.inflate(R.layout.fragment_section_dummy, container, false);
        Bundle args = getArguments();

		/* seriesX = new ArrayList<GraphViewData>();
		exampleSeries1 = new GraphViewSeries(new GraphViewData[] {}); */
		
		graphView1 = new LineGraphView(getActivity(), "GraphViewDemo");
		
		LinearLayout layout = (LinearLayout) rootView.findViewById(R.id.graph1);
		layout.addView(graphView1);	
		
		GraphViewData[] data = new GraphViewData[100];
		
		float v = 0;
		for (int i=0; i<100; i++) {
		   v += 0.2f;
		   data[i] = new GraphViewData(i, Math.sin(v));
		}		
		
		exampleSeries1 = new GraphViewSeries(data);		

		graphView1.addSeries(exampleSeries1);
		graphView1.setViewPort(1, 100);
	
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
		exampleSeries1.appendData(new GraphViewData(dataCount, d.acc[0]), true, 100);			
		dataCount++;		
		
		if (dataCount > 100) {
			// dataCount = 0;			

			// exampleSeries1.resetData(new GraphViewData[] {});
		}	
				
		Log.d("lpms", "updating data view");
	}
}