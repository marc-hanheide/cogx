#ifndef LASERSCANVIEW_H
#define LASERSCANVIEW_H

#include <QtGui/QGraphicsView>

class LaserScanView : public QGraphicsView
{
    Q_OBJECT

public:
    LaserScanView(QWidget *parent);
    ~LaserScanView();


public slots:

	void addScan(double startAngle, double angleStep, double maxRange, QVector<double> range, double time);


private:

    QGraphicsScene *_scanScene;

};

#endif // LaserScanView_H


