package com.LpResearch.LpmsBNativeAndroidLibrary;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.io.IOException;
import java.io.InputStream;
import android.opengl.GLSurfaceView;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.opengl.GLUtils;
import android.opengl.Matrix;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;
import android.content.Context;
import android.opengl.GLSurfaceView;
import android.opengl.GLU;
import android.util.Log;

public class LpmsBRenderer implements GLSurfaceView.Renderer {
	private FloatBuffer vertexBuffer;
	private int numFaces = 6;
	private FloatBuffer texBuffer; 
	public float[] verticesBuffer;
	
	float[] texCoords = { // Texture coords for the above face (NEW)
		0.0f, 1.0f,
		1.0f, 1.0f,
		0.0f, 0.0f,
		1.0f, 0.0f
	};
   
	private int[] textureIDs = new int[numFaces];	
	
	private float[][] colors = {
		{ 1.0f, 0.5f, 0.0f, 1.0f },
		{ 1.0f, 0.0f, 1.0f, 1.0f },
		{ 0.0f, 1.0f, 0.0f, 1.0f },
		{ 0.0f, 0.0f, 1.0f, 1.0f },
		{ 1.0f, 0.0f, 0.0f, 1.0f }, 
		{ 1.0f, 1.0f, 0.0f, 1.0f }
	};	
	
	public float[] vertices2 = {
		-1.0f, -1.0f,  1.0f,
		1.0f, -1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f 
	};
	
	public float[] vertices = {
		-1.0f, -1.0f,  1.0f,
		1.0f, -1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,

		1.0f, -1.0f, -1.0f, 
		-1.0f, -1.0f, -1.0f,
		1.0f,  1.0f, -1.0f, 
		-1.0f,  1.0f, -1.0f, 

		-1.0f, -1.0f, -1.0f, 
		-1.0f, -1.0f,  1.0f, 
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f,  1.0f,

		1.0f, -1.0f,  1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f, -1.0f, 

		-1.0f,  1.0f,  1.0f, 
		1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f, -1.0f,
		1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f, -1.0f, 
		1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f, 
		1.0f, -1.0f,  1.0f
	};	
	
	public float backVertices[] = {
		-10.0f, 10.0f, -10.0f,
		-10.0f, -10.0f, -10.0f,
		10.0f, -10.0f, -10.0f,
		10.0f, 10.0f, -10.0f,
	};
	
	public float[] q;	
	public float[] vertices3;
	public float[] texVertices;	
	
	private static Context context;	
	
	LpmsBRenderer(Context appContext) {
		context = appContext;
		q = new float[4];
		
		q[0] = 1.0f;
		q[1] = 0;
		q[2] = 0;
		q[3] = 0;						
		
		vertices3 = new float[128];		
	}
  
    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
		// gl.glClearColor(0.6f, 0.6f, 1.0f, 1.0f);
		gl.glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		
		gl.glClearDepthf(1.0f);
		gl.glEnable(GL10.GL_DEPTH_TEST);
		gl.glDepthFunc(GL10.GL_LEQUAL);
		gl.glHint(GL10.GL_PERSPECTIVE_CORRECTION_HINT, GL10.GL_NICEST); 
		gl.glShadeModel(GL10.GL_SMOOTH);
		gl.glDisable(GL10.GL_DITHER); 
		gl.glEnable(GL10.GL_TEXTURE_2D);
		
		loadTexture(gl);	
		initShapes();		
    }
    
	public float[] rotM = {
		1.0f, 0, 0, 0,
		0, 1.0f, 0, 0,
		0, 0, 1.0f, 0, 
		0, 0, 0, 1.0f };
		
	public float[] scaleM = {
		-1.0f, 0, 0, 0,
		0, 1.0f, 0, 0,
		0, 0, 1.0f, 0, 
		0, 0, 0, 1.0f };
		
	/* public float[] quaternionToMatrix(float[] q)
	{
		float qw = q[0];
		float qx = q[1];
		float qy = q[2];
		float qz = q[3];	
		
		float[] M = new float[16];
		
		M[0] = qw*qw + qx*qx - qy*qy - qz*qz;
		M[1] = 2*qx*qy - 2*qw*qz;
		M[2] = 2*qx*qz + 2*qw*qy;

		M[3] = 2*qx*qy + 2*qw*qz;
		M[4] = qw*qw - qx*qx + qy*qy - qz*qz;
		M[5] = 2*qy*qz - 2*qw*qx;

		M[6] = 2*qx*qz - 2*qw*qy;
		M[7] = 2*qy*qz + 2*qw*qx;
		M[8] = qw*qw - qx*qx - qy*qy + qz*qz;
				
		return M;
	} */
	
	public float[] quaternionToMatrix(float[] q)
	{
		float tmp1;
		float tmp2;

		float[] M = new float[16];				
		
		float sqw = q[0] * q[0];
		float sqx = q[1] * q[1];
		float sqy = q[2] * q[2];
		float sqz = q[3] * q[3];
		
		float invs = 1 / (sqx + sqy + sqz + sqw);
		
		M[0] = ( sqx - sqy - sqz + sqw) * invs;
		M[4] = (-sqx + sqy - sqz + sqw) * invs;
		M[8] = (-sqx - sqy + sqz + sqw) * invs;
		
		tmp1 = q[1] * q[2];
		tmp2 = q[3] * q[0];
		
		M[3] = 2.0f * (tmp1 + tmp2) * invs;
		M[1] = 2.0f * (tmp1 - tmp2) * invs;
		
		tmp1 = q[1] * q[3];
		tmp2 = q[2] * q[0];
		
		M[6] = 2.0f * (tmp1 - tmp2) * invs;
		M[2] = 2.0f * (tmp1 + tmp2) * invs;
		
		tmp1 = q[2] * q[3];
		tmp2 = q[1] * q[0];
		
		M[7] = 2.0f * (tmp1 + tmp2) * invs;
		M[5] = 2.0f * (tmp1 - tmp2) * invs;
		
		return M;
	}
		
    public void onDrawFrame(GL10 gl) {
		gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
		gl.glEnableClientState(GL10.GL_TEXTURE_COORD_ARRAY);
		gl.glEnable(GL10.GL_CULL_FACE);		
			
		float[] M = quaternionToMatrix(q);
			
		for (int i=0; i < vertices.length; i+=3) {
			vertices3[i+0] = M[0]*vertices[i+0] + M[1]*vertices[i+1] + M[2]*vertices[i+2];
			vertices3[i+1] = M[3]*vertices[i+0] + M[4]*vertices[i+1] + M[5]*vertices[i+2];
			vertices3[i+2] = M[6]*vertices[i+0] + M[7]*vertices[i+1] + M[8]*vertices[i+2];
		}
		
		// Log.e("LpmsMonitor", "q: " + q[0] + " " + q[1] + " " + q[2] + " " + q[3]); 		
		
		ByteBuffer vbb = ByteBuffer.allocateDirect(vertices3.length * 4);
		vbb.order(ByteOrder.nativeOrder());
		vertexBuffer = vbb.asFloatBuffer();
		vertexBuffer.put(vertices3);
		vertexBuffer.position(0);
				
		gl.glClear(GL10.GL_COLOR_BUFFER_BIT | GL10.GL_DEPTH_BUFFER_BIT);

		gl.glLoadIdentity();
		gl.glTranslatef(0.0f, 0.0f, -6.0f);
		
		gl.glFrontFace(GL10.GL_CCW);
		gl.glCullFace(GL10.GL_BACK);
		
		gl.glVertexPointer(3, GL10.GL_FLOAT, 0, vertexBuffer);
		gl.glTexCoordPointer(2, GL10.GL_FLOAT, 0, texBuffer);			
		
		for (int i = 0; i < 6; i++) {		
			gl.glDrawArrays(GL10.GL_TRIANGLE_STRIP, i*4, 4);
		}
		
		gl.glDisableClientState(GL10.GL_TEXTURE_COORD_ARRAY);
		gl.glDisableClientState(GL10.GL_VERTEX_ARRAY);
		gl.glDisable(GL10.GL_CULL_FACE);
    }
    
    public void onSurfaceChanged(GL10 gl, int width, int height) {
		if (height == 0) height = 1; 
		float aspect = (float)width / height;

		gl.glViewport(0, 0, width, height);

		gl.glMatrixMode(GL10.GL_PROJECTION);
		gl.glLoadIdentity();
		
		GLU.gluPerspective(gl, 45, aspect, 0.1f, 100.f);

		gl.glMatrixMode(GL10.GL_MODELVIEW);
		gl.glLoadIdentity();
    }    	
	
	private void initShapes() {
		ByteBuffer vbb = ByteBuffer.allocateDirect(vertices.length * 4);
		vbb.order(ByteOrder.nativeOrder());
		vertexBuffer = vbb.asFloatBuffer();
		vertexBuffer.put(vertices);
		vertexBuffer.position(0);
		
		ByteBuffer tbb = ByteBuffer.allocateDirect(texCoords.length * 4 * numFaces);
		tbb.order(ByteOrder.nativeOrder());
		texBuffer = tbb.asFloatBuffer();
		for (int i=0; i<numFaces; i++) {
			texBuffer.put(texCoords);
		}
		texBuffer.position(0);
	}
	
	public void loadTexture(GL10 gl) {
		gl.glGenTextures(1, textureIDs, 0);

		gl.glBindTexture(GL10.GL_TEXTURE_2D, textureIDs[0]);

		gl.glTexParameterf(GL10.GL_TEXTURE_2D, GL10.GL_TEXTURE_MIN_FILTER, GL10.GL_NEAREST);
		gl.glTexParameterf(GL10.GL_TEXTURE_2D, GL10.GL_TEXTURE_MAG_FILTER, GL10.GL_LINEAR);
		
		gl.glTexParameterf(GL10.GL_TEXTURE_2D, GL10.GL_TEXTURE_WRAP_S, GL10.GL_CLAMP_TO_EDGE);
		gl.glTexParameterf(GL10.GL_TEXTURE_2D, GL10.GL_TEXTURE_WRAP_T, GL10.GL_CLAMP_TO_EDGE);

		InputStream istream = context.getResources().openRawResource(R.drawable.lpmslogo);
		Bitmap bitmap;
		
		Log.e("lpms", "Loading texture"); 
		
		try {
			bitmap = BitmapFactory.decodeStream(istream);
		} finally {
			try {
				istream.close();
			} catch(IOException e) {
				Log.e("lpms", "Stream problem"); 			
			}
		}

		GLUtils.texImage2D(GL10.GL_TEXTURE_2D, 0, bitmap, 0);
		bitmap.recycle();
		
		Log.e("lpms", "Loaded texture"); 
	}
}