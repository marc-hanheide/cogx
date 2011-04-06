#include "OdometryView.h"
#include "Tester.h"


// ------------------------------------------------------
OdometryView::OdometryView(QWidget *parent)
    : QGraphicsView(parent)
{
	_odomScene=0;
	_prevX=-10000000;
	_prevY=0;
	_prevTheta=0;
	_prevTime=0;
	_maxSpeed=0;
}

// ------------------------------------------------------
OdometryView::~OdometryView()
{

}

// ------------------------------------------------------
void OdometryView::addOdometry(double x, double y, double theta, double time)
{
	if (_prevX<-1000000)
	{
		_prevX=x;
		_prevY=y;
		_prevTheta=theta;
		_prevTime=time;
	}
	else
	{
		QGraphicsScene *old=_odomScene;
		_odomScene = new QGraphicsScene(this);

		double dt=time-_prevTime;
		double dx=x-_prevX;
		double dy=y-_prevY;
		double dr=sqrt(dx*dx+dy*dy);
		double dtheta=theta-_prevTheta;
		double speed=dr/dt;
		double rotspeed=dtheta/dt;
		_prevX=x;
		_prevY=y;
		_prevTheta=theta;
		_prevTime=time;

		if (speed>_maxSpeed)
			_maxSpeed=speed;

		double tmp=_maxSpeed/30;
		speed+=4*tmp;

		_odomScene->addLine(0, 0, speed*cos(3.1415926/2+rotspeed), -speed*sin(3.1415926/2+rotspeed));
		_odomScene->addEllipse(-tmp, -tmp, 2*tmp, 2*tmp);

		setScene(_odomScene);
		fitInView(-_maxSpeed, -_maxSpeed, 2*_maxSpeed, 2*_maxSpeed);
		if (old)
			delete old;
	}
}


