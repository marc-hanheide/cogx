#ifndef CATEGORICALWIDGET_H
#define CATEGORICALWIDGET_H

#include <QtGui/QWidget>
#include "ui_CategoricalWidget.h"
#include "CategoricalData.hpp"


namespace conceptual
{
	class Tester;
}


class CategoricalWidget : public QWidget
{
    Q_OBJECT

public:
    CategoricalWidget(QWidget *parent, conceptual::Tester *component);
    ~CategoricalWidget();


    void newVisualResults(CategoricalData::VisualResultsPtr vrPtr);
    void newLaserResults(CategoricalData::LaserResultsPtr lrPtr);
    void newLaserScan(CategoricalData::LaserScanPtr lsPtr);
    void newOdometry(CategoricalData::OdometryPtr oPtr);
    void newImage(CategoricalData::ImagePtr iPtr);


private slots:

	void newShapeResults(QStringList shapes, QList<double> values);
	void newSizeResults(QStringList sizes, QList<double> values);
	void newAppearanceResults(QStringList appearances, QList<double> values);

	void setImage(QImage img, int frame);


private:
    Ui::CategoricalWidgetClass ui;
    conceptual::Tester *_component;
    QVector<QRgb> _colorMap;

};

#endif // CATEGORICALWIDGET_H
