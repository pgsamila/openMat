package com.LpResearch.LpmsBNativeAndroidLibrary;

import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;
import java.util.Map;

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

import com.jjoe64.graphview.BarGraphView;
import com.jjoe64.graphview.GraphView;
import com.jjoe64.graphview.GraphViewStyle;
import com.jjoe64.graphview.GraphViewSeries.GraphViewSeriesStyle;
import com.jjoe64.graphview.GraphView.GraphViewData;
import com.jjoe64.graphview.GraphViewSeries;
import com.jjoe64.graphview.LineGraphView;

public class MoreDataFragment extends MyFragment {
	final static String TAG = "MoreDataFragment";
	final int FRAGMENT_TAG = 3; 
	
    public static final String ARG_SECTION_NUMBER = "section_number";

 	View rootView;

	private GraphView quatGraph;
	private GraphViewSeries quatSeries0;
	private GraphViewSeries quatSeries1;
	private GraphViewSeries quatSeries2;
	private GraphViewSeries quatSeries3;
	
	private GraphView eulerGraph;
	private GraphViewSeries eulerSeries0;
	private GraphViewSeries eulerSeries1;
	private GraphViewSeries eulerSeries2;
	
	private GraphView linAccGraph;
	private GraphViewSeries linAccSeries0;
	private GraphViewSeries linAccSeries1;
	private GraphViewSeries linAccSeries2;		

	int dataCount = 100;	
 	
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        rootView = inflater.inflate(R.layout.more_data_fragment_layout, container, false);
        Bundle args = getArguments();
		
		quatGraph = new LineGraphView(getActivity(), "");
		eulerGraph = new LineGraphView(getActivity(), "");
		linAccGraph = new LineGraphView(getActivity(), "");
		
		GraphViewStyle gvStyle = new GraphViewStyle();
		gvStyle.setTextSize(18);
		gvStyle.setNumHorizontalLabels(10);

		quatGraph.setGraphViewStyle(gvStyle);
		eulerGraph.setGraphViewStyle(gvStyle);
		linAccGraph.setGraphViewStyle(gvStyle);
		
		LinearLayout quatLayout = (LinearLayout) rootView.findViewById(R.id.quatGraph);
		LinearLayout eulerLayout = (LinearLayout) rootView.findViewById(R.id.eulerGraph);
		LinearLayout linAccLayout = (LinearLayout) rootView.findViewById(R.id.linAccGraph);

		quatLayout.addView(quatGraph);
		eulerLayout.addView(eulerGraph);
		linAccLayout.addView(linAccGraph);
		
		quatSeries0 = new GraphViewSeries("W-Axis", new GraphViewSeriesStyle(Color.RED, 1), new GraphViewData[] { });
		quatSeries1 = new GraphViewSeries("X-Axis", new GraphViewSeriesStyle(Color.GREEN, 1), new GraphViewData[] { });
		quatSeries2 = new GraphViewSeries("Y-Axis", new GraphViewSeriesStyle(Color.BLUE, 1), new GraphViewData[] { });
		quatSeries3 = new GraphViewSeries("Z-Axis", new GraphViewSeriesStyle(Color.YELLOW, 1), new GraphViewData[] { });

		eulerSeries0 = new GraphViewSeries("X-Axis", new GraphViewSeriesStyle(Color.RED, 1), new GraphViewData[] { });
		eulerSeries1 = new GraphViewSeries("Y-Axis", new GraphViewSeriesStyle(Color.GREEN, 1), new GraphViewData[] { });
		eulerSeries2 = new GraphViewSeries("Z-Axis", new GraphViewSeriesStyle(Color.BLUE, 1), new GraphViewData[] { });

		linAccSeries0 = new GraphViewSeries("X-Axis", new GraphViewSeriesStyle(Color.RED, 1), new GraphViewData[] { });
		linAccSeries1 = new GraphViewSeries("Y-Axis", new GraphViewSeriesStyle(Color.GREEN, 1), new GraphViewData[] { });
		linAccSeries2 = new GraphViewSeries("Z-Axis", new GraphViewSeriesStyle(Color.BLUE, 1), new GraphViewData[] { });

		quatGraph.addSeries(quatSeries0);
		quatGraph.addSeries(quatSeries1);
		quatGraph.addSeries(quatSeries2);
		quatGraph.addSeries(quatSeries3);

		eulerGraph.addSeries(eulerSeries0);
		eulerGraph.addSeries(eulerSeries1);
		eulerGraph.addSeries(eulerSeries2);

		linAccGraph.addSeries(linAccSeries0);
		linAccGraph.addSeries(linAccSeries1);
		linAccGraph.addSeries(linAccSeries2);
				
		quatGraph.setScrollable(true);	
		quatGraph.setManualYAxisBounds(1.0, -1.0);

		eulerGraph.setScrollable(true);	
		eulerGraph.setManualYAxisBounds(180.0, -180.0);

		linAccGraph.setScrollable(true);	
		linAccGraph.setManualYAxisBounds(4.0, -4.0);
		
		String[] xLabels = { "0", "20", "40", "60", "80", "100" };
		String[] yLabels0 = { "1.0", "0.5", "0", "-0.5", "-1.0"};
		String[] yLabels1 = { "180", "90", "0", "-90", "-180"};		
		String[] yLabels2 = { "4", "2", "0", "-2", "-4"};		
		
		quatGraph.setHorizontalLabels(xLabels);
		quatGraph.setVerticalLabels(yLabels0);

		eulerGraph.setHorizontalLabels(xLabels);
		eulerGraph.setVerticalLabels(yLabels1);

		linAccGraph.setHorizontalLabels(xLabels);
		linAccGraph.setVerticalLabels(yLabels2);
	
        return rootView;
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
	
		quatSeries0.appendData(new GraphViewData((double)dataCount, d.quat[0]), true, 200);			
		quatSeries1.appendData(new GraphViewData((double)dataCount, d.quat[1]), true, 200);
		quatSeries2.appendData(new GraphViewData((double)dataCount, d.quat[2]), true, 200);
		quatSeries3.appendData(new GraphViewData((double)dataCount, d.quat[3]), true, 200);
		
		eulerSeries0.appendData(new GraphViewData((double)dataCount, d.euler[0]), true, 200);			
		eulerSeries1.appendData(new GraphViewData((double)dataCount, d.euler[1]), true, 200);
		eulerSeries2.appendData(new GraphViewData((double)dataCount, d.euler[2]), true, 200);		
		
		linAccSeries0.appendData(new GraphViewData((double)dataCount, d.linAcc[0]), true, 200);			
		linAccSeries1.appendData(new GraphViewData((double)dataCount, d.linAcc[1]), true, 200);
		linAccSeries2.appendData(new GraphViewData((double)dataCount, d.linAcc[2]), true, 200);		
				
		quatGraph.setViewPort(dataCount-100, 100);
		eulerGraph.setViewPort(dataCount-100, 100);	
		linAccGraph.setViewPort(dataCount-100, 100);	

		dataCount++;
	}
}