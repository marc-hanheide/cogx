#include "LaserScanView.h"
#include <math.h>

// ------------------------------------------------------
LaserScanView::LaserScanView(QWidget *parent)
    : QGraphicsView(parent)
{
	_scanScene=0;
}


// ------------------------------------------------------
LaserScanView::~LaserScanView()
{

}


// ------------------------------------------------------
void LaserScanView::addScan(double startAngle, double angleStep, double maxRange, QVector<double> range, double time)
{
	QGraphicsScene *old=_scanScene;
	_scanScene = new QGraphicsScene(this);

	double angle=startAngle;
	int prev=0;
	double prevAngle=startAngle;

	for(int i=0; i<range.size(); ++i)
	{
		if (range[i] < maxRange)
		{
			prev=i;
			prevAngle=angle;
			break;
		}
		angle+=angleStep;
	}


	for(int i=prev+1; i<range.size(); ++i)
	{
		angle+=angleStep;
		if (range[i] < maxRange)
		{
			double r1=range[prev];
			double r2=range[i];
			double a1=prevAngle;
			double a2=angle;
			double x1=r1*cos(a1);
			double y1=r1*sin(a1);
			double x2=r2*cos(a2);
			double y2=r2*sin(a2);

			prevAngle=angle;
			prev=i;

			_scanScene->addLine(x1, -y1, x2, -y2);
		}
	}

	setScene(_scanScene);
	fitInView(sceneRect(), Qt::KeepAspectRatio);
	if (old)
		delete old;
}

