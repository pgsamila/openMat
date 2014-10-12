package com.LpResearch.LpmsBNativeAndroidLibrary;

public class LpmsBData {
	public int imuId = 0;
	public float timestamp = 0.0f;
	public int frameNumber = 0;
	public float[] gyr = new float[3];
	public float[] acc = new float[3];
	public float[] mag = new float[3];
	public float[] quat = new float[4];
	public float[] euler = new float[3];
	public float[] linAcc = new float[3];
	public float pressure;
	
	public LpmsBData() {
	}
	
	public LpmsBData(LpmsBData d) {
		imuId = d.imuId;
		timestamp = d.timestamp;
		frameNumber = d.frameNumber;
		
		for (int i=0; i<3; i++) gyr[i] = d.gyr[i];
		for (int i=0; i<3; i++) acc[i] = d.acc[i];
		for (int i=0; i<3; i++) mag[i] = d.mag[i];
		for (int i=0; i<4; i++) quat[i] = d.quat[i];
		for (int i=0; i<3; i++) euler[i] = d.euler[i];
		for (int i=0; i<3; i++) linAcc[i] = d.linAcc[i];
		
		pressure = d.pressure;
	}	
}