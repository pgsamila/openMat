package com.LpResearch.LpmsBNativeAndroidLibrary;

import java.net.*;
import java.util.*;
import java.io.*;
import java.text.*;
import java.lang.*;
import java.util.HashMap;
import java.util.Map;

import android.app.*;
import android.app.Activity;
import android.app.ActionBar;
import android.app.Activity;
import android.app.FragmentTransaction;
import android.view.*;
import android.widget.*;
import android.widget.Adapter;
import android.os.*;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.util.*;
import android.content.*;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.bluetooth.*;
import android.graphics.*;
import android.opengl.GLSurfaceView;
import android.annotation.TargetApi;
import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentActivity;
import android.support.v4.view.ViewPager;
import android.view.View.OnClickListener;
import android.widget.AdapterView.OnItemClickListener;

public class ConnectionFragment extends MyFragment implements OnClickListener {
	final int FRAGMENT_TAG = 0;
    public static final String ARG_SECTION_NUMBER = "section_number";
 	View rootView;
	BluetoothAdapter btAdapter;
	ArrayList<String> dcLpms = new ArrayList<String>();	
	ArrayAdapter dcAdapter;
	ListView btList;
	String currentLpms;
	boolean firstDc;
	OnConnectListener connectListener;	
	
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        rootView = inflater.inflate(R.layout.connect_screen, container, false);
        Bundle args = getArguments();
		btAdapter = BluetoothAdapter.getDefaultAdapter();
		
		prepareButtons();
		prepareDcList();	
		
        return rootView;
    }

	void prepareButtons() {
		Button b = (Button) rootView.findViewById(R.id.button_discover);
		b.setOnClickListener(this);
		b = (Button) rootView.findViewById(R.id.button_connect);
		b.setOnClickListener(this);
		b = (Button) rootView.findViewById(R.id.button_disconnect);
		b.setOnClickListener(this);
	}
	
    @Override
    public void onClick(View v) {
        switch (v.getId()) {
        case R.id.button_discover:
			startBtDiscovery();
		break;
		
        case R.id.button_connect:
			startBtConnect();
		break;
		
		case R.id.button_disconnect:
			connectListener.onDisconnect();
		break;
        }
    }
	
	public interface OnConnectListener {
		public void onConnect(String address);
		public void onDisconnect();
	}
	
	void startBtConnect() {
		Log.d("lpms", "Connect: " + currentLpms);
		if (btAdapter != null && !currentLpms.equals("")) {
			connectListener.onConnect(currentLpms);
		}
	}
	
	private final BroadcastReceiver mReceiver = new BroadcastReceiver() {
		public void onReceive(Context context, Intent intent) {
			String action = intent.getAction();

			if (BluetoothDevice.ACTION_FOUND.equals(action)) {
				BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
				
				if (device.getName().equals("LPMS-B")) {
					if (firstDc == true) {
						dcLpms.clear();
						firstDc = false;
					}
				
					Log.d("lpms", "Discovered: " + device.getName());
					currentLpms = device.getAddress();
					dcLpms.add(device.getAddress());
					dcAdapter.notifyDataSetChanged();
				} 
			}
		}
	};
	
	public void prepareDcList() {
		btList = (ListView) rootView.findViewById(R.id.list);
		IntentFilter filter = new IntentFilter(BluetoothDevice.ACTION_FOUND);
		getActivity().registerReceiver(mReceiver, filter);
		dcAdapter = new ArrayAdapter(getActivity(), android.R.layout.simple_list_item_1, dcLpms);
		btList.setAdapter(dcAdapter);
		dcLpms.add("Press Discover button to start discovery..");
		dcAdapter.notifyDataSetChanged();
		firstDc = true;
	}
	
	public void startBtDiscovery() {
		Log.d("lpms", "Start discovery");
		btAdapter.startDiscovery();
		
		btList.setOnItemClickListener(new OnItemClickListener() {
			@Override
			public void onItemClick(AdapterView<?> parent, View view,
			int position, long id) {
				int itemPosition = position;
				currentLpms = (String) btList.getItemAtPosition(position);
				Log.d("lpms", "Select: " + currentLpms);
			}
		}); 		
	}
	
	@Override
	public void onAttach(Activity activity) {
		super.onAttach(activity);
		
        try {
            connectListener = (OnConnectListener) activity;
        } catch (ClassCastException e) {
            throw new ClassCastException(activity.toString() + " must implement OnArticleSelectedListener");
        }		
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
		TextView text = (TextView) rootView.findViewById(R.id.connection_status);
		
		switch (s) {
		case 0:
			text.setText("Not connected");
		break;
		
		default:
			text.setText("Connected to LPMS-B (" + currentLpms + ")");
		break;
		}
	}
}