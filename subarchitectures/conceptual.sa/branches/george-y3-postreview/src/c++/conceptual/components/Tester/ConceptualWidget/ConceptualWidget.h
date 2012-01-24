/**
 * @author Andrzej Pronobis
 *
 * Main dialog.
 */

#ifndef CONCEPTUALWIDGET_H
#define CONCEPTUALWIDGET_H

// Conceptual.SA
#include "ui_ConceptualWidget.h"
#include "ConceptualData.hpp"
#include "ConceptualEvent.h"

// Qt&Std
#include <QtGui/QDialog>
#include <queue>

class QTimer;

namespace conceptual
{
	class Tester;
}

class ObjectPlacePropertyDialog;
class ObjectSearchResultDialog;
class HumanAssertionDialog;

class ConceptualWidget : public QWidget, public Ui::ConceptualWidgetClass
{
    Q_OBJECT

    friend class ObjectPlacePropertyDialog;
    friend class ObjectSearchResultDialog;
    friend class HumanAssertionDialog;


public:
    ConceptualWidget(QWidget *parent, conceptual::Tester *component);
    ~ConceptualWidget();

public:

    void newWorldState(ConceptualData::WorldStatePtr wsPtr);

signals:

	void newEventInfo(const QList<conceptual::ConceptualEvent> &events);
	void locationChanged(int placeId);


private slots:

	void categoriesButtonClicked();
	void objectsButtonClicked();
	void sendQueryButtonClicked();
	void refreshVarsButtonClicked();
	void saveEventsButtonClicked();
	void refreshWsButtonClicked();
	void showGraphButtonClicked();
	void visualizeButtonClicked();
	void varListCurrentTextChanged(const QString &curText);
	void factorListCurrentTextChanged(const QString &curText);
	void wsTimerTimeout();
	void posTimerTimeout();
	void addObjectPlacePropertyButtonClicked();
	void addObjectSearchResultButtonClicked();
	void addHumanAssertionButtonClicked();
	void addEvent(conceptual::ConceptualEvent event);
	void collectInfoCheckBoxToggled(bool);

private:

	int getRoomForPlace(ConceptualData::WorldStatePtr wsPtr, int placeId);
	void collectEventInfo(conceptual::ConceptualEvent event);
	void getPlacesForRoom(ConceptualData::WorldStatePtr wsPtr, int roomId, QList<int> &places);
	double getExistsProbability(SpatialProbabilities::ProbabilityDistribution &probDist);
	void saveEvents(QString fileName);
	double castTimeToSeconds(const cast::cdl::CASTTime &time);


private:
    conceptual::Tester *_component;

    QTimer *_wsTimer;
    QTimer *_posTimer;
    int _wsCount;

    std::queue<qint64> _wsUpdateTimes;

	pthread_mutex_t _worldStateMutex;
	ConceptualData::WorldStatePtr _wsPtr;

	int _prevPlace;
	long _eventNo;

	QList<conceptual::ConceptualEvent> _events;

};



#endif // ConceptualWidget_H
