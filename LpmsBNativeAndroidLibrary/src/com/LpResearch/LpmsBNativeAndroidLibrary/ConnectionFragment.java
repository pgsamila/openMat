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
import android.widget.AbsListView;
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
	OnConnectListener connectListener;
	String currentLpms;
		
	ArrayList<String> dcLpms = new ArrayList<String>();	
	ArrayAdapter dcAdapter;
	ListView btList;
	boolean firstDc = true;
	
	ArrayList<String> connectedDevicesLpms = new ArrayList<String>();	
	ArrayAdapter connectedDevicesAdapter;
	ListView connectedDevicesList;
	boolean firstConnectedDevice = true;
	
	TextView loggingStateText;

	boolean preLogStatus = false;	
	
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        rootView = inflater.inflate(R.layout.connect_screen, container, false);
        Bundle args = getArguments();
		btAdapter = BluetoothAdapter.getDefaultAdapter();
		
		prepareButtons();
		prepareDiscoveredDevicesList();	
		prepareConnectedDevicesList();
		
        return rootView;
    }

	void prepareButtons() {
		Button b = (Button) rootView.findViewById(R.id.button_discover);
		b.setOnClickListener(this);
		b = (Button) rootView.findViewById(R.id.button_connect);
		b.setOnClickListener(this);
		b = (Button) rootView.findViewById(R.id.button_disconnect);
		b.setOnClickListener(this);
		b = (Button) rootView.findViewById(R.id.button_start_logging);
		b.setOnClickListener(this);
		b = (Button) rootView.findViewById(R.id.button_stop_logging);
		b.setOnClickListener(this);
		loggingStateText = (TextView) rootView.findViewById(R.id.logging_status);
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
			onDisconnect();
		break;
		
		case R.id.button_start_logging:
			connectListener.startLogging();
		break;
		
		case R.id.button_stop_logging:
			connectListener.stopLogging();
		break;		
        }
    }
	
	public interface OnConnectListener {
		public void onConnect(String address);
		public void onDisconnect();
		public void startLogging();
		public void stopLogging();		
	}
	
	void startBtConnect() {
		btAdapter.cancelDiscovery();

		if (btAdapter != null && !currentLpms.equals("")) {
			Toast.makeText(getActivity(), "Connecting to " + currentLpms, Toast.LENGTH_SHORT).show();
			
			connectListener.onConnect(currentLpms);
		}
	}
	
	void onDisconnect() {
		synchronized (connectedDevicesLpms) {		
			for (ListIterator<String> it = connectedDevicesLpms.listIterator(); it.hasNext(); ) {
				String e = it.next();
			
				if (currentLpms.equals(e)) {		
					connectedDevicesLpms.remove(e);
					connectedDevicesAdapter.notifyDataSetChanged();				
					
					if (connectedDevicesLpms.size() == 0) {
						connectedDevicesLpms.add("Press connect button to connect to device..");
						connectedDevicesAdapter.notifyDataSetChanged();
						
						firstConnectedDevice = true;
						
						connectedDevicesList.getChildAt(0).setBackgroundColor(getResources().getColor(R.color.black));
					}
					
					return;
				}
			}	
		}
	}
	
	private final BroadcastReceiver mReceiver = new BroadcastReceiver() {
		public void onReceive(Context context, Intent intent) {
			String action = intent.getAction();

			if (BluetoothDevice.ACTION_FOUND.equals(action)) {
				BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
				
				synchronized (dcLpms) {				
					if (device.getName().equals("LPMS-B")) {
						if (firstDc == true) {
							dcLpms.clear();							
							btList.getChildAt(0).setBackgroundColor(getResources().getColor(R.color.navy));
							currentLpms = device.getAddress();							
							
							firstDc = false;							
						}
											
						dcLpms.add(device.getAddress());
						dcAdapter.notifyDataSetChanged();
					}
				}
			}
			
			if (BluetoothDevice.ACTION_ACL_CONNECTED.equals(action)) {
				BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
				
				synchronized (connectedDevicesLpms) {	
					if (device.getName().equals("LPMS-B")) {
						if (firstConnectedDevice == true) {
							connectedDevicesLpms.clear();							
							connectedDevicesList.getChildAt(0).setBackgroundColor(getResources().getColor(R.color.navy));
							((LpmsBMainActivity)getActivity()).onSensorSelectionChanged(device.getName());
							
							firstConnectedDevice = false;							
						}
											
						connectedDevicesLpms.add(device.getAddress());
						connectedDevicesAdapter.notifyDataSetChanged();
					}
				}
			}			
		}
	};
	
	public void prepareDiscoveredDevicesList() {
		btList = (ListView) rootView.findViewById(R.id.list);
		IntentFilter filter = new IntentFilter(BluetoothDevice.ACTION_FOUND);
		getActivity().registerReceiver(mReceiver, filter);
		dcAdapter = new ArrayAdapter(getActivity(), R.layout.list_view_text_item, dcLpms);
		btList.setAdapter(dcAdapter);
		dcLpms.add("Press Discover button to start discovery..");
		dcAdapter.notifyDataSetChanged();
		firstDc = true;			
		
		btList.setOnItemClickListener(new OnItemClickListener() {
			@Override
			public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
				if (firstDc == true) return;
			
				for(int a = 0; a < parent.getChildCount(); a++) {
					parent.getChildAt(a).setBackgroundColor(Color.TRANSPARENT);
				}			
				view.setBackgroundColor(getResources().getColor(R.color.navy));
					
				int itemPosition = position;
				currentLpms = (String) btList.getItemAtPosition(position);
			}
		});
	}
	
	public void prepareConnectedDevicesList() {
		connectedDevicesList = (ListView) rootView.findViewById(R.id.connected_devices_list);
		IntentFilter filter = new IntentFilter(BluetoothDevice.ACTION_ACL_CONNECTED);
		getActivity().registerReceiver(mReceiver, filter);
		connectedDevicesAdapter = new ArrayAdapter(getActivity(), R.layout.list_view_text_item, connectedDevicesLpms);
		connectedDevicesList.setAdapter(connectedDevicesAdapter);
		connectedDevicesLpms.add("Press connect button to connect to device..");
		connectedDevicesAdapter.notifyDataSetChanged();
		firstConnectedDevice = true;
		
		connectedDevicesList.setOnItemClickListener(new OnItemClickListener() {
			@Override
			public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
				if (firstConnectedDevice == true) return;			
			
				for(int a = 0; a < parent.getChildCount(); a++) {
					parent.getChildAt(a).setBackgroundColor(Color.TRANSPARENT);
				}			
				view.setBackgroundColor(getResources().getColor(R.color.navy));
				
				int itemPosition = position;
				String itemValue = (String) connectedDevicesList.getItemAtPosition(position);
				((LpmsBMainActivity)getActivity()).onSensorSelectionChanged(itemValue);
			}
		}); 		
	}
	
	public void startBtDiscovery() {
		Toast.makeText(getActivity(), "Starting discovery..", Toast.LENGTH_SHORT).show();		
		
		btAdapter.cancelDiscovery();
		btAdapter.startDiscovery();		
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
	public void updateView(LpmsBData d, ImuStatus s) {
		if (preLogStatus != s.isLogging) {
			preLogStatus = s.isLogging;
			
			if (s.isLogging == true) {
				loggingStateText.setText("Logging to " + s.logFileName);
			} else {
				loggingStateText.setText("Logging is OFF");
			}
		}
	}
}