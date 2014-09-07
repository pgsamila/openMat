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
import android.widget.LinearLayout;
import android.widget.Toast;
import android.opengl.GLSurfaceView;
import android.graphics.Color;

import com.jjoe64.graphview.BarGraphView;
import com.jjoe64.graphview.GraphView;
import com.jjoe64.graphview.GraphViewStyle;
import com.jjoe64.graphview.GraphViewSeries.GraphViewSeriesStyle;
import com.jjoe64.graphview.GraphView.GraphViewData;
import com.jjoe64.graphview.GraphViewSeries;
import com.jjoe64.graphview.LineGraphView;

public class DataFragment extends MyFragment {
	final static String TAG = "DataFragment";
	final int FRAGMENT_TAG = 2; 
	
    public static final String ARG_SECTION_NUMBER = "section_number";

 	View rootView;

	private GraphView accGraph;
	private GraphViewSeries accSeries0;
	private GraphViewSeries accSeries1;
	private GraphViewSeries accSeries2;
	
	private GraphView gyrGraph;
	private GraphViewSeries gyrSeries0;
	private GraphViewSeries gyrSeries1;
	private GraphViewSeries gyrSeries2;
	
	private GraphView magGraph;
	private GraphViewSeries magSeries0;
	private GraphViewSeries magSeries1;
	private GraphViewSeries magSeries2;		

	int dataCount = 100;
 	
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        rootView = inflater.inflate(R.layout.data_fragment, container, false);
        Bundle args = getArguments();
		
		accGraph = new LineGraphView(getActivity(), "");
		gyrGraph = new LineGraphView(getActivity(), "");
		magGraph = new LineGraphView(getActivity(), "");
		
		GraphViewStyle gvStyle = new GraphViewStyle();
		gvStyle.setTextSize(18);
		gvStyle.setNumHorizontalLabels(10);

		accGraph.setGraphViewStyle(gvStyle);
		gyrGraph.setGraphViewStyle(gvStyle);
		magGraph.setGraphViewStyle(gvStyle);
		
		LinearLayout accLayout = (LinearLayout) rootView.findViewById(R.id.accGraph);
		LinearLayout gyrLayout = (LinearLayout) rootView.findViewById(R.id.gyrGraph);
		LinearLayout magLayout = (LinearLayout) rootView.findViewById(R.id.magGraph);

		accLayout.addView(accGraph);
		gyrLayout.addView(gyrGraph);
		magLayout.addView(magGraph);
		
		accSeries0 = new GraphViewSeries("X-Axis", new GraphViewSeriesStyle(Color.RED, 1), new GraphViewData[] { });
		accSeries1 = new GraphViewSeries("Y-Axis", new GraphViewSeriesStyle(Color.GREEN, 1), new GraphViewData[] { });
		accSeries2 = new GraphViewSeries("Z-Axis", new GraphViewSeriesStyle(Color.BLUE, 1), new GraphViewData[] { });

		gyrSeries0 = new GraphViewSeries("X-Axis", new GraphViewSeriesStyle(Color.RED, 1), new GraphViewData[] { });
		gyrSeries1 = new GraphViewSeries("Y-Axis", new GraphViewSeriesStyle(Color.GREEN, 1), new GraphViewData[] { });
		gyrSeries2 = new GraphViewSeries("Z-Axis", new GraphViewSeriesStyle(Color.BLUE, 1), new GraphViewData[] { });

		magSeries0 = new GraphViewSeries("X-Axis", new GraphViewSeriesStyle(Color.RED, 1), new GraphViewData[] { });
		magSeries1 = new GraphViewSeries("Y-Axis", new GraphViewSeriesStyle(Color.GREEN, 1), new GraphViewData[] { });
		magSeries2 = new GraphViewSeries("Z-Axis", new GraphViewSeriesStyle(Color.BLUE, 1), new GraphViewData[] { });

		accGraph.addSeries(accSeries0);
		accGraph.addSeries(accSeries1);
		accGraph.addSeries(accSeries2);

		gyrGraph.addSeries(gyrSeries0);
		gyrGraph.addSeries(gyrSeries1);
		gyrGraph.addSeries(gyrSeries2);

		magGraph.addSeries(magSeries0);
		magGraph.addSeries(magSeries1);
		magGraph.addSeries(magSeries2);
				
		accGraph.setScrollable(true);	
		accGraph.setManualYAxisBounds(4.0, -4.0);

		gyrGraph.setScrollable(true);	
		gyrGraph.setManualYAxisBounds(2000.0, -2000.0);

		magGraph.setScrollable(true);	
		magGraph.setManualYAxisBounds(200.0, -200.0);
		
		String[] xLabels = { "0", "20", "40", "60", "80", "100" };
		String[] yLabels0 = { "4", "2", "0", "-2", "-4"};
		String[] yLabels1 = { "2000", "1000", "0", "-1000", "-2000"};		
		String[] yLabels2 = { "200", "100", "0", "-100", "-200"};		
		
		accGraph.setHorizontalLabels(xLabels);
		accGraph.setVerticalLabels(yLabels0);

		gyrGraph.setHorizontalLabels(xLabels);
		gyrGraph.setVerticalLabels(yLabels1);

		magGraph.setHorizontalLabels(xLabels);
		magGraph.setVerticalLabels(yLabels2);
	
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
	public void updateView(LpmsBData d, int s) {
		if (s == 0) return;
	
		accSeries0.appendData(new GraphViewData((double)dataCount, d.acc[0]), true, 200);			
		accSeries1.appendData(new GraphViewData((double)dataCount, d.acc[1]), true, 200);
		accSeries2.appendData(new GraphViewData((double)dataCount, d.acc[2]), true, 200);
		
		gyrSeries0.appendData(new GraphViewData((double)dataCount, d.gyr[0]), true, 200);			
		gyrSeries1.appendData(new GraphViewData((double)dataCount, d.gyr[1]), true, 200);
		gyrSeries2.appendData(new GraphViewData((double)dataCount, d.gyr[2]), true, 200);		
		
		magSeries0.appendData(new GraphViewData((double)dataCount, d.mag[0]), true, 200);			
		magSeries1.appendData(new GraphViewData((double)dataCount, d.mag[1]), true, 200);
		magSeries2.appendData(new GraphViewData((double)dataCount, d.mag[2]), true, 200);		
				
		accGraph.setViewPort(dataCount-100, 100);
		gyrGraph.setViewPort(dataCount-100, 100);	
		magGraph.setViewPort(dataCount-100, 100);	

		dataCount++;
	}
	
	public void clearView() {		
		dataCount = 100;
		
		accSeries0.resetData(new GraphViewData[] { });
		accSeries1.resetData(new GraphViewData[] { });
		accSeries2.resetData(new GraphViewData[] { });
		
		gyrSeries0.resetData(new GraphViewData[] { });
		gyrSeries1.resetData(new GraphViewData[] { });
		gyrSeries2.resetData(new GraphViewData[] { });
		
		magSeries0.resetData(new GraphViewData[] { });
		magSeries1.resetData(new GraphViewData[] { });
		magSeries2.resetData(new GraphViewData[] { });			
	}
}