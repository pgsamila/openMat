package com.LpResearch.LpmsBNativeAndroidLibrary;

import java.net.*;
import java.util.*;
import java.io.*;
import java.text.*;
import java.nio.ByteBuffer;
import java.io.FileDescriptor;
import java.math.BigDecimal;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.LinkedBlockingDeque;
import java.lang.reflect.Method;

import android.bluetooth.*;
import android.util.*;
import android.app.*;
import android.view.*;
import android.widget.*;
import android.os.*;
import android.util.*;
import android.content.*;
import android.view.inputmethod.*;

public class LpmsBThread extends Thread {
	final String TAG = "lpms";
	
	final UUID MY_UUID_INSECURE = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");	
	
	final int PACKET_ADDRESS0 = 0;
	final int PACKET_ADDRESS1 = 1;
	final int PACKET_FUNCTION0 = 2;
	final int PACKET_FUNCTION1 = 3;
	final int PACKET_RAW_DATA = 4;
	final int PACKET_LRC_CHECK0 = 5;
	final int PACKET_LRC_CHECK1 = 6;
	final int PACKET_END = 7;
	final int PACKET_LENGTH0 = 8;
	final int PACKET_LENGTH1 = 9;
	
	final int LPMS_ACK = 0;
	final int LPMS_NACK = 1;
	final int LPMS_GET_CONFIG = 4;	
	final int LPMS_GET_STATUS = 5;	
	final int LPMS_GOTO_COMMAND_MODE = 6;	
	final int LPMS_GOTO_STREAM_MODE = 7;	
 	final int LPMS_GOTO_SLEEP_MODE = 8;	
	final int LPMS_GET_SENSOR_DATA = 9;
 	final int LPMS_SET_TRANSMIT_DATA = 10;	
	
	final int STATE_IDLE = 0;
	
	int rxState = PACKET_END;
	byte[] rxBuffer = new byte[512];
	byte[] txBuffer = new byte[512];
	byte[] rawTxData = new byte[256];
	byte[] rawRxBuffer = new byte[256];	
	int currentAddress;
	int currentFunction;
	int currentLength;
	int rxIndex = 0;
	byte b = 0;
	int lrcCheck;
    boolean isConnected = false;
	int nBytes;
	int timeout;
	boolean waitForAck;
	boolean waitForData;
	int state;
	byte inBytes[] = new byte[2];    
	InputStream mInStream;
	OutputStream mOutStream;
	BluetoothSocket mSocket;
	BluetoothAdapter mAdapter;
	String mAddress;
	BluetoothDevice mDevice;
	boolean isGetGyroscope = true;
	boolean isGetAcceleration = true;
	boolean isGetMagnetometer = true;
	boolean isGetQuaternion = true;
	boolean isGetEulerAngler = true;
	boolean isGetLinearAcceleration = true;
	int imuId = 0;
	DataOutputStream dos;
	public float startStamp = 0.0f;
	public boolean resetTimestampFlag = true;
	boolean newDataFlag = false;	
	LinkedBlockingDeque<LpmsBData> dataQueue = new LinkedBlockingDeque<LpmsBData>();
	LpmsBData mLpmsBData = new LpmsBData();
	int frameCounter = 0;

	LpmsBThread(BluetoothAdapter adapter) {
		mAdapter = adapter;
	}

	public void connect(String address, int id) {
		mAddress = address;
		imuId = id;

		Log.e(TAG, "[LpmsBThread] Connect to: " + mAddress);

        if (mAdapter == null) {
			Log.e(TAG, "[LpmsBThread] Didn't find Bluetooth adapter");

			return;
        }
	
		mAdapter.cancelDiscovery();

		Log.e(TAG, "[LpmsBThread] Getting device");
		try {
			mDevice = mAdapter.getRemoteDevice(mAddress);
		} catch (IllegalArgumentException e) {
			Log.e(TAG, "[LpmsBThread] Invalid Bluetooth address", e);
			return;
		}

		mSocket = null;
		Log.e(TAG, "[LpmsBThread] Creating socket");
		try {
			mSocket = mDevice.createInsecureRfcommSocketToServiceRecord(MY_UUID_INSECURE);
		} catch (Exception e) {
			Log.e(TAG, "[LpmsBThread] Socket create() failed", e);
			return;
		}
	
		Log.e(TAG, "[LpmsBThread] Trying to connect..");	
		try {
			mSocket.connect();
		} catch (IOException e) {
			Log.e(TAG, "[LpmsBThread] Couldn't connect to device", e);
			return;
		}
	
		Log.e(TAG, "[LpmsBThread] Connected!");		
	
		try {
			mInStream = mSocket.getInputStream();
			mOutStream = mSocket.getOutputStream();
		} catch (IOException e) {
			Log.e(TAG, "[LpmsBThread] Streams not created", e);
			return;
		}			
		
		resetTimestamp();
		
		Thread t = new Thread(new ClientReadThread());
        t.start();
		
		frameCounter = 0;
	}
	
    public class ClientReadThread implements Runnable {
        public void run() {
        	/* Thread t = new Thread(new ClientStateThread());	
        	t.start(); */
	
			while (mSocket.isConnected() == true) {
				try {
					nBytes = mInStream.read(rawRxBuffer);
				} catch (Exception e) {
					break;
				}
				
				parse();
			}
		}
	}	
	
    public class ClientStateThread implements Runnable {
        public void run() {
			try {				
				while (mSocket.isConnected() == true) {
					if (waitForAck == false && waitForData == false) {
						switch (state) {
						case STATE_IDLE:
						break;
						}
					} else if (timeout > 100) {
						Log.d(TAG, "[LpmsBThread] Receive timeout");
						timeout = 0;
						state = STATE_IDLE;
						waitForAck = false;
						waitForData = false;
					} else {
						Thread.sleep(10);
						++timeout;
					} 
			
					try {
						Thread.sleep(1);
					} catch (InterruptedException e) {
					}
				}
			} catch (Exception e) {
				Log.d(TAG, "[LpmsBThread] Connection interrupted");
				isConnected = false;			
			}
		}
	}
	
	void setAcquisitionParameters(	boolean isGetGyroscope,
									boolean isGetAcceleration,
									boolean isGetMagnetometer,
									boolean isGetQuaternion,
									boolean isGetEulerAngler,
									boolean isGetLinearAcceleration) {
		this.isGetGyroscope = isGetGyroscope;
		this.isGetAcceleration = isGetAcceleration;
		this.isGetMagnetometer = isGetMagnetometer;
		this.isGetQuaternion = isGetQuaternion;
		this.isGetEulerAngler = isGetEulerAngler;
		this.isGetLinearAcceleration = isGetLinearAcceleration;
	}
	
	public static float round(float d, int decimalPlace) {
		BigDecimal bd = new BigDecimal(Float.toString(d));
		bd = bd.setScale(decimalPlace, BigDecimal.ROUND_HALF_UP);
		return bd.floatValue();
	}	
	
	void parseSensorData() {
		int o = 0;
		float r2d = 57.2958f;
	
		mLpmsBData.imuId = imuId;
		mLpmsBData.timestamp = convertRxbytesToFloat(o, rxBuffer); o += 4;
		mLpmsBData.frameNumber = frameCounter;
		frameCounter++;
		
	 	if (isGetGyroscope == true) {
			mLpmsBData.gyr[0] = convertRxbytesToFloat(o, rxBuffer) * r2d; o += 4;
			mLpmsBData.gyr[1] = convertRxbytesToFloat(o, rxBuffer) * r2d; o += 4;
			mLpmsBData.gyr[2] = convertRxbytesToFloat(o, rxBuffer) * r2d; o += 4;
		}
	
	 	if (isGetAcceleration == true) {	
			mLpmsBData.acc[0]	= convertRxbytesToFloat(o, rxBuffer); o += 4;
			mLpmsBData.acc[1]	= convertRxbytesToFloat(o, rxBuffer); o += 4;
			mLpmsBData.acc[2]	= convertRxbytesToFloat(o, rxBuffer); o += 4;
		}

	 	if (isGetMagnetometer == true) {
			mLpmsBData.mag[0]	= convertRxbytesToFloat(o, rxBuffer); o += 4;
			mLpmsBData.mag[1]	= convertRxbytesToFloat(o, rxBuffer); o += 4;
			mLpmsBData.mag[2]	= convertRxbytesToFloat(o, rxBuffer); o += 4;
		}
		
	 	if (isGetQuaternion == true) {
			mLpmsBData.quat[0]	= convertRxbytesToFloat(o, rxBuffer); o += 4;
			mLpmsBData.quat[1]	= convertRxbytesToFloat(o, rxBuffer); o += 4;
			mLpmsBData.quat[2]	= convertRxbytesToFloat(o, rxBuffer); o += 4;
			mLpmsBData.quat[3]	= convertRxbytesToFloat(o, rxBuffer); o += 4;
		}
		
	 	if (isGetEulerAngler == true) {
			mLpmsBData.euler[0] = convertRxbytesToFloat(o, rxBuffer) * r2d; o += 4;
			mLpmsBData.euler[1] = convertRxbytesToFloat(o, rxBuffer) * r2d; o += 4;
			mLpmsBData.euler[2] = convertRxbytesToFloat(o, rxBuffer) * r2d; o += 4;
		}
		
	 	if (isGetLinearAcceleration == true) {	
			mLpmsBData.linAcc[0] = convertRxbytesToFloat(o, rxBuffer); o += 4;
			mLpmsBData.linAcc[1] = convertRxbytesToFloat(o, rxBuffer); o += 4;
			mLpmsBData.linAcc[2] = convertRxbytesToFloat(o, rxBuffer); o += 4;
		}
		
		dataQueue.addFirst(new LpmsBData(mLpmsBData));
		
		newDataFlag = true;
	}
	
	public boolean hasNewData() {
		if (dataQueue.peekLast() != null) {
			return true;
		}
		return false;
	}	
	
	public LpmsBData getLpmsBData() {
		if (dataQueue.peekLast() != null) {
			LpmsBData d = new LpmsBData(dataQueue.peekLast());
			dataQueue.removeLast();
			return d;
		}
		return null;
	}
	
	void parseFunction() {	
		switch (currentFunction) {
		case LPMS_ACK:
			Log.d(TAG, "[LpmsBThread] Received ACK");
		break;

		case LPMS_NACK:
			Log.d(TAG, "[LpmsBThread] Received NACK");
		break;
		
		case LPMS_GET_CONFIG:
		break;
		
		case LPMS_GET_STATUS:
		break;
		
		case LPMS_GOTO_COMMAND_MODE:
		break;
		
		case LPMS_GOTO_STREAM_MODE:
		break;
 	
		case LPMS_GOTO_SLEEP_MODE:
		break;
		
	 	case LPMS_GET_SENSOR_DATA:
			parseSensorData();
		break;
		
 		case LPMS_SET_TRANSMIT_DATA:
		break;
		}
		
		waitForAck = false;
		waitForData = false;
	}
	
	void parse() {
		int lrcReceived = 0;
	
		for (int i=0; i<nBytes; i++) {
			b = rawRxBuffer[i];
		
			switch (rxState) {
			case PACKET_END:
				if (b == 0x3a) {
					rxState = PACKET_ADDRESS0;
				}
				break;
				
			case PACKET_ADDRESS0:
				inBytes[0] = b;
				rxState = PACKET_ADDRESS1;
				break;

			case PACKET_ADDRESS1:
				inBytes[1] = b;				
				currentAddress = convertRxbytesToInt16(0, inBytes);
				rxState = PACKET_FUNCTION0;
				break;

			case PACKET_FUNCTION0:
				inBytes[0] = b;
				rxState = PACKET_FUNCTION1;				
				break;

			case PACKET_FUNCTION1:
				inBytes[1] = b;				
				currentFunction = convertRxbytesToInt16(0, inBytes);			
				rxState = PACKET_LENGTH0;	
			break;

			case PACKET_LENGTH0:
				inBytes[0] = b;
				rxState = PACKET_LENGTH1;
			break;
					
			case PACKET_LENGTH1:
				inBytes[1] = b;				
				currentLength = convertRxbytesToInt16(0, inBytes);	
				rxState = PACKET_RAW_DATA;
				rxIndex = 0;
			break;
					
			case PACKET_RAW_DATA:
				if (rxIndex == currentLength) {
					lrcCheck = (currentAddress & 0xffff) + (currentFunction & 0xffff) + (currentLength & 0xffff);
					
					for (int j=0; j<currentLength; j++) {
						lrcCheck += (int) rxBuffer[j] & 0xff;
					}		
						
					inBytes[0] = b;		
					rxState = PACKET_LRC_CHECK1;								
				} else {	
					rxBuffer[rxIndex] = b;
					++rxIndex;
				}
			break;
				
			case PACKET_LRC_CHECK1:
				inBytes[1] = b;
				
				lrcReceived = convertRxbytesToInt16(0, inBytes);
				lrcCheck = lrcCheck & 0xffff;

				if (lrcReceived == lrcCheck) {	
					parseFunction();
				} else {
				}
				
				rxState = PACKET_END;
			break;
			
			default:
				rxState = PACKET_END;
			break;
			}
		}
	}
	
	void sendData(int address, int function, int length) {
		int txLrcCheck;

		txBuffer[0] = 0x3a;
		convertInt16ToTxbytes(address, 1, txBuffer);
		convertInt16ToTxbytes(function, 3, txBuffer);
		convertInt16ToTxbytes(length, 5, txBuffer);
		
		for (int i=0; i < length; ++i) {
			txBuffer[7+i] = rawTxData[i];
		}
		
		txLrcCheck = address;
		txLrcCheck += function;
		txLrcCheck += length;
		
		for (int i=0; i < length; i++) {
			txLrcCheck += (int) rawTxData[i];
		}
		
		convertInt16ToTxbytes(txLrcCheck, 7 + length, txBuffer);
		txBuffer[9 + length] = 0x0d;
		txBuffer[10 + length] = 0x0a;
			
		for (int i=0; i < 11 + length; i++) {
			Log.d(TAG, "[LpmsBThread] Sending: " + Byte.toString(txBuffer[i]));		
		}
			
		try {
			Log.d(TAG, "[LpmsBThread] Sending data");
			mOutStream.write(txBuffer, 0, length+11);
		} catch (Exception e) {
			Log.d(TAG, "[LpmsBThread] Error while sending data");
		}
	}
	
	void sendAck() {
		sendData(0, LPMS_ACK, 0);
	}
	
	void sendNack() {
		sendData(0, LPMS_NACK, 0);
	}
	
	float convertRxbytesToFloat(int offset, byte buffer[]) {
		int v = 0;
		byte[] t = new byte[4];
		
		for (int i=0; i<4; i++) {
			t[3-i] = buffer[i+offset];
		}
		
		return Float.intBitsToFloat(ByteBuffer.wrap(t).getInt(0)); 
	}
	
	int convertRxbytesToInt(int offset, byte buffer[]) {
		int v;
		byte[] t = new byte[4];
		
		for (int i=0; i<4; i++) {
			t[3-i] = buffer[i+offset];
		}
		
		v = ByteBuffer.wrap(t).getInt(0);
		
		return v; 
	}
	
	int convertRxbytesToInt16(int offset, byte buffer[]) {
		int v;
		byte[] t = new byte[2];
		
		for (int i=0; i<2; ++i) {
			t[1-i] = buffer[i+offset];
		}
		
		v = (int) ByteBuffer.wrap(t).getShort(0) & 0xffff;
		
		return v; 
	}	
	
	void convertIntToTxbytes(int v, int offset, byte buffer[]) {
		byte[] t = ByteBuffer.allocate(4).putInt(v).array();
	
		for (int i=0; i<4; i++) {
			buffer[3-i+offset] = t[i];
		}
	}
	
	void convertInt16ToTxbytes(int v, int offset, byte buffer[]) {
		byte[] t = ByteBuffer.allocate(2).putShort((short) v).array();
	
		for (int i=0; i<2; i++) {
			buffer[1-i+offset] = t[i];
		}
	}	

	void convertFloatToTxbytes(float f, int offset, byte buffer[]) {
		int v = Float.floatToIntBits(f);
		byte[] t = ByteBuffer.allocate(4).putInt(v).array();
	
		for (int i=0; i<4; i++) {
			buffer[3-i+offset] = t[i];
		}
	}
	
	public void close() {
		isConnected = false;	
	
		try {
			mSocket.close();
		} catch (Exception e) {
		}
		
		Log.d(TAG, "[LpmsBThread] Connection closed");		
	}
	
	public void resetTimestamp() {
		resetTimestampFlag = true;
	}
	
	public String getAddress() {
		return mAddress;
	}
}	