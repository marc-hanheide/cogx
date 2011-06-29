#ifndef ODOMETRYVIEW_H
#define ODOMETRYVIEW_H

#include <QtGui/QGraphicsView>

class OdometryView : public QGraphicsView
{
    Q_OBJECT

public:
    OdometryView(QWidget *parent);
    ~OdometryView();


public slots:

    void addOdometry(double x, double y, double theta, double time);


private:
    QGraphicsScene *_odomScene;

    double _prevX, _prevY, _prevTheta, _prevTime;
    double _maxSpeed;

};

#endif // OdometryView_H
