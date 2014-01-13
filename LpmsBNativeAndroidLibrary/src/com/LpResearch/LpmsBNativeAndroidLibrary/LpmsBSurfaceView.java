package com.LpResearch.LpmsBNativeAndroidLibrary;

import java.net.*;
import java.util.*;
import java.io.*;
import java.text.*;
import java.lang.*;

import android.app.*;
import android.view.*;
import android.widget.*;
import android.os.*;
import android.util.*;
import android.content.*;
import android.content.pm.ActivityInfo;
import android.bluetooth.*;
import android.graphics.*;
import android.opengl.GLSurfaceView;

class LpmsBSurfaceView extends GLSurfaceView {
    public LpmsBRenderer lmRenderer;

	public LpmsBSurfaceView(Context context) {
		super(context);

		lmRenderer = new LpmsBRenderer(context);		
		setRenderer(lmRenderer);	
	}
}