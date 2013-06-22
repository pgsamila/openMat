#ifndef VERTICAL_BAR_GRAPH
#define VERTICAL_BAR_GRAPH

#include <math.h>

#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QWidget>
#include <QTimer>
#include <QFontMetrics>

class VerticalBarGraph : public QWidget {
Q_OBJECT
public:
	float v;
	float mi;
	float ma;
	float lpV;
	int c;
	int cg;

public:
	VerticalBarGraph(int cg = 0, QWidget *p = 0) :
		cg(cg)
	{
		setMinimumHeight(200);
		setMinimumWidth(30);
		
		mi = 0;
		ma = 0.5f;
		v = 0;
		lpV = 0;
		c = 0;
		
		QTimer* timer = new QTimer(this);
		connect(timer, SIGNAL(timeout()), this, SLOT(timerUpdate()));
		timer->start(50);
	}
	
	void setRange(float mi, float ma)
	{
		this->mi = mi;
		this->ma = ma;
	}
	
	void setValue(float v)
	{
		if (v > ma) {
			this->v = ma;
		} else if (v < mi) {
			this->v = mi;
		} else {
			this->v = v;
		}
				
		if (this->v > lpV) {
			lpV = this->v;
		}
		
		update();
	}
	
	void setColor(int c)
	{
		this->c = c;
	}
	
	void draw(void)
	{
		QPainter painter(this);

		QFont f;
		f.setPixelSize(15);
		painter.setFont(f);
		
		QPen white(QColor(255, 255, 255));
		QPen black(QColor(80, 80, 80));
		QPen red(QColor(255, 150, 150));
		QPen blue(QColor(150, 150, 255));
		QBrush redB(QColor(255, 150, 150));
		QBrush blueB(QColor(150, 150, 255));
		QBrush whiteB(QColor(240, 240, 240));
		QPen green(QColor(150, 255, 150));
		QBrush greenB(QColor(150, 255, 150));
		
		painter.setPen(black);
		painter.setBrush(whiteB);	
		painter.drawRect(0, 0, width()-1, height()-1);		
		
		float d = ma - mi;

		float s = (height()-2) / d;
		if (cg == 1) s = (height()-3) / d;

		float gv = v * s;
		float agv = lpV * s;

		if (c == 0) {
			painter.setPen(blue);	
			painter.setBrush(blueB);
		} else {
			painter.setPen(green);	
			painter.setBrush(greenB);
		}
		
		if ((int) gv != 0) {
			if (cg == 0) {
				painter.drawRect(1, height()-1-gv, width()-3, gv);
			} else {
				if (gv >= 0) {
					painter.drawRect(1, height()/2-1-gv, width()-3, gv);
				} else {
					painter.drawRect(1, height()/2-1, width()-3, fabs(gv));
				}
			}
		}
			
		painter.setPen(red);
		painter.setBrush(redB);

		if (cg == 0) {
			painter.drawRect(1, height()-agv-3, width()-3, 2);
		} else {
			painter.drawRect(1, height()/2-agv-3, width()-3, 2);
		}
	}

public slots:
	void paintEvent(QPaintEvent* e) 
	{
		draw();
	}
	
	void timerUpdate(void)
	{
		lpV = lpV * 0.99 + v * 0.01;
	}
};

class RepDisplay : public QWidget {
Q_OBJECT
public:
	float v;
	int c;
	int f;
	int dt;

public:
	RepDisplay(int dt = 0, int f = 0, QWidget *p = 0) :
		f(f),
		dt(dt)
	{
		setMinimumHeight(30);
		setMinimumWidth(100);
		
		QTimer* timer = new QTimer(this);
		connect(timer, SIGNAL(timeout()), this, SLOT(timerUpdate()));
		timer->start(50);
		
		v = 0;
		c = 0;		
	}
	
	void setValue(float v)
	{
		this->v = v;

		update();
	}
	
	void setColor(int c)
	{
		this->c = c;
	}
	
	void draw(void)
	{
		QPainter painter(this);

		QFont f;
		f.setPixelSize(width() / 3);
		painter.setFont(f);
		QFontMetrics fm(f, painter.device());
		
		QPen white(QColor(255, 255, 255));
		QPen black(QColor(80, 80, 80));
		QPen red(QColor(255, 150, 150));
		QPen blue(QColor(150, 150, 255));
		QBrush redB(QColor(255, 150, 150));
		QBrush blueB(QColor(150, 150, 255));
		QBrush whiteB(QColor(240, 240, 240));
		QPen green(QColor(150, 255, 150));
		QBrush greenB(QColor(150, 255, 150));
		
		painter.setPen(black);
		painter.setBrush(whiteB);	
		painter.drawRect(0, 0, width()-1, height()-1);			
		
		painter.setPen(blue);
				
		if (dt == 0) {
			QString text = QString().number(v, 'f', 2);
			QPoint center = QPoint((width()-fm.boundingRect(text).width())/2, (height()+fm.boundingRect(text).height()/2)/2);		
			painter.drawText(center.x(), center.y(), text);
		} else {
			QString text = QString().number(v, 'f', 0);
			QPoint center = QPoint((width()-fm.width(text))/2, (height()+fm.height()/2)/2);		
			painter.drawText(center.x(), center.y(), text);
		}
	}

public slots:
	void paintEvent(QPaintEvent* e) 
	{
		draw();
	}
	
	void timerUpdate(void)
	{
	}
};

#endif
