#ifndef INSOLE_VIEW
#define INSOLE_VIEW

#include <QSvgWidget>
#include <QRadialGradient>
#include <QPainter>

#define N_MARKER 4

class InsoleView : public QSvgWidget {
public:
	InsoleView(QWidget *parent = 0);
	void paintEvent(QPaintEvent *event);
	void drawMarker(QPainter *p, int i, float a);
	void updateMarker(int i, float v);
	
	int footLeftRight;
	float mValue[N_MARKER];
};

#endif