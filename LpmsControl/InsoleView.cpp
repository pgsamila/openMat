#include "InsoleView.h"

InsoleView::InsoleView(QWidget *parent)/* : QSvgWidget(parent)*/ {
	QString fn = "./svg/foot_print_two_black_white_line_art.svg";
	load(fn);
	
	for (int i=0; i<N_MARKER; ++i) mValue[i] = 0;
}

void InsoleView::paintEvent(QPaintEvent *event) {
	QSvgWidget::paintEvent(event);	

	QPainter p(this);
	
	drawMarker(&p, 0, mValue[0]);
	drawMarker(&p, 1, mValue[1]);
	drawMarker(&p, 2, mValue[2]);
	drawMarker(&p, 3, mValue[3]);
}

void InsoleView::updateMarker(int i, float v)
{
	if (v < N_MARKER) mValue[i] = fabs(v);
	
	update();
}

void InsoleView::drawMarker(QPainter *p, int i, float a) 
{
	int d = 0;
	int px = 0;
	int py = 0;

	d = height () * 7/40;
	
	switch (i) {
	case 0:
		px = width() * 3/40 + d/2;
		py = height() * 6/20 + d/2;
	break;
	
	case 1:
		px = width() * 7/40 + d/2;
		py = height() * 6/20 + d/2;
	break;

	case 2:
		px = width() * 11/40 + d/2;
		py = height() * 6/20 + d/2;
	break;	
		
	case 3:
		px = width() * 10/40 + d/2;
		py = height() * 15/20 + d/2;
	break;
	}
	
	int v = (int) (a * 255.0f);
	
	QRadialGradient rg(px, py, d/2);
	rg.setColorAt(0, QColor(255, 0, 0, v));
	rg.setColorAt(1, QColor(255, 0, 0, 0));
	
	p->setBrush(QBrush(rg));	
	p->setPen(Qt::transparent);
	
	p->drawEllipse(px - d/2, py - d/2, d, d);
}
		
		
		
		
		
		
		