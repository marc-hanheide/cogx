#ifndef DEMOWIDGET_H
#define DEMOWIDGET_H

#include <QtGui/QWidget>
#include "ui_DemoWidget.h"
#include "CategoricalData.hpp"


namespace conceptual
{
	class Tester;
}


class DemoWidget : public QWidget
{
    Q_OBJECT

public:
    DemoWidget(QWidget *parent, conceptual::Tester *component);
    ~DemoWidget();

    void newVisualResults(CategoricalData::VisualResultsPtr vrPtr);
    void newLaserResults(CategoricalData::LaserResultsPtr lrPtr);
    void newLaserScan(CategoricalData::LaserScanPtr lsPtr);
    void newImage(CategoricalData::ImagePtr iPtr);
    void newWorldState(ConceptualData::WorldStatePtr wsPtr);


public slots:

	void enabledToggled(bool state);
	void newShapeResults(QStringList shapes, QList<double> values);
	void newSizeResults(QStringList sizes, QList<double> values);
	void newAppearanceResults(QStringList appearances, QList<double> values);
	void setImage(QImage img, int frame);
	void updateEvents(const QList<conceptual::ConceptualEvent> &events);
	void addGroundtruthButtonClicked();
	void addObjectButtonClicked();
	void locationChanged(int placeId);
	void updateStuff();


private:
    Ui::DemoWidgetClass ui;
    conceptual::Tester *_component;
	std::set<int> _roomIds;
    bool _enabled;
    QVector<QRgb> _colorMap;
	int _curPlaceId;
	int _curRoomId;
	const std::vector<std::string> *_roomCats;
	QList<conceptual::ConceptualEvent> _lastEvents;


	pthread_mutex_t _worldStateMutex;
	ConceptualData::WorldStatePtr _wsPtr;

};

#endif // DEMOWIDGET_H
