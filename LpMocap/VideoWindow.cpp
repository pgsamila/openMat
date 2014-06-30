#include "VideoWindow.h"

VideoWindow::VideoWindow(QWidget* parent) : QGLWidget(parent)
{
	videoCaptureHandle = new cv::VideoCapture(0);

	videoPlaybackStarted = false;
	videoRecordingStarted = false;	
	
	updateVideo();
}

VideoWindow::~VideoWindow(void)
{
	delete videoCaptureHandle;
}

void VideoWindow::updateVideo(void)
{	
    if (!videoCaptureHandle->isOpened()) return;
	if (videoPlaybackStarted) return;

	*videoCaptureHandle >> currentImage;
	
	if (videoRecordingStarted) {
		writeVideoFrame();
	}
		
	updateGL();
}

void VideoWindow::updatePlayback(void)
{
	if (videoPlaybackStarted == false) return;
	
	*videoFileReader >> currentImage;

	double nFrames = videoFileReader->get(CV_CAP_PROP_FRAME_COUNT);
	double currentFramePos = videoFileReader->get(CV_CAP_PROP_POS_FRAMES);

	if (nFrames <= currentFramePos) {
		videoFileReader->set(CV_CAP_PROP_POS_FRAMES, 0);
	}

	updateGL();
}

void VideoWindow::resetPlayback(void)
{
	videoFileReader->set(CV_CAP_PROP_POS_FRAMES, 0);
}

bool VideoWindow::openVideoFileForWriting(const char *fn)
{
	cv::Size imageSize = currentImage.size();	

	videoWriter = new cv::VideoWriter();
	if (videoWriter->open(fn, CV_FOURCC('M', 'J', 'P', 'G'), 30, imageSize) == false) {
		printf("[VideoWindow] Couldn't open video file %s\n", fn);
		videoRecordingStarted = false;
		delete videoWriter;
	} else {
		videoRecordingStarted = true;
		return true;
	}
	
	return false;
}

void VideoWindow::stopRecording(void)
{
	if (videoRecordingStarted == true) {
		if (videoWriter->isOpened()) delete videoWriter;
		
		videoRecordingStarted = false;
	}
}

bool VideoWindow::openVideoFileForReading(const char *fn)
{
	videoFileReader = new cv::VideoCapture(std::string(fn));
			
	if (videoFileReader->isOpened()) {
		videoPlaybackStarted = true;
		
		return true;
	} else {
		printf("[VideoWindow] Couldn't open video file %s\n", fn);
	}
	
	return false;
}

void VideoWindow::stopPlayback(void)
{
	videoPlaybackStarted = false;
	delete videoFileReader;
}
	
void VideoWindow::writeVideoFrame(void)
{
	if (videoRecordingStarted == true) {
		if (videoWriter->isOpened() == false) return;
			
		videoWriter->write(currentImage);
	} 
}

void VideoWindow::paintGL(void)
{
	double w;
	double h;
	double n_w, n_h;
	int x, y;
	
	w = this->width();
	h = this->height();

	cv::Size imageSize = currentImage.size();	
	
	double iw = imageSize.width;
	double ih = imageSize.height;

	if (w > (iw / ih * h)) {
		n_w = iw / ih * h;
		n_h = h;
	} else {
		n_w = w;
		n_h = ih / iw * w;
	}

	x = (w - n_w) / 2;
	y = (h - n_h) / 2;
	
	glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(0.5, 0.5, 0.5, 1.0);
	glDisable(GL_DEPTH_TEST);	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, imageSize.width, imageSize.height, 0);
	glMatrixMode(GL_MODELVIEW);	
	
	glEnable(GL_TEXTURE_2D);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	glTexImage2D(GL_TEXTURE_2D, 0, 4, imageSize.width, imageSize.height, 0, GL_BGR, GL_UNSIGNED_BYTE, (unsigned char *)currentImage.ptr());
	
	glBegin(GL_QUADS);
	glTexCoord2f(0, 1); glVertex2f(0, imageSize.height);
	glTexCoord2f(0, 0); glVertex2f(0, 0);
	glTexCoord2f(1, 0); glVertex2f(imageSize.width, 0);
	glTexCoord2f(1, 1); glVertex2f(imageSize.width, imageSize.height);
	glEnd();

	glDisable(GL_TEXTURE_2D);
	
	glViewport(x, y, (GLint)n_w, (GLint)n_h);		
	glFlush();
}