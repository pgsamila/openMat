#ifndef VIDEO_WINDOW
#define VIDEO_WINDOW

#include <QGLWidget>
#include <QWidget>
#include <QTimer>
#include <QMouseEvent>
#include <QImage>
#include <QGLWidget>

#include <string>
#include <iostream>
#include <limits>

#include "opencv2/opencv.hpp"

#include <GL/glu.h>

class VideoWindow : public QGLWidget {
public:
	VideoWindow(QWidget *parent = 0);
	~VideoWindow(void);
	void updateVideo(void);
	void paintGL(void);
	bool openVideoFileForWriting(const char *fn);
	bool openVideoFileForReading(const char *fn);
	void writeVideoFrame(void);
	void stopRecording(void);
	void stopPlayback(void);
	void updatePlayback(void);
	void resetPlayback(void);
	
	cv::Mat currentImage;
	cv::VideoCapture* videoCaptureHandle;
	cv::VideoCapture* videoFileReader;
	bool videoPlaybackStarted;
	bool videoRecordingStarted;
	cv::VideoWriter* videoWriter;
};

#endif

