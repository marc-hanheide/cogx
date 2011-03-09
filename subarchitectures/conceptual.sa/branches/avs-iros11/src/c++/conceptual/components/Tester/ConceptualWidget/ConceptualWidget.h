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
class RCVisualizer;

class ConceptualWidget : public QWidget, public Ui::ConceptualWidgetClass
{
    Q_OBJECT

    friend class ObjectPlacePropertyDialog;
    friend class ObjectSearchResultDialog;
    friend class RCVisualizer;

public:
    ConceptualWidget(QWidget *parent, conceptual::Tester *component);
    ~ConceptualWidget();

public:

    void newWorldState(ConceptualData::WorldStatePtr wsPtr);


signals:
	void addEventToHistorySignal(QString str);

private slots:

	void collectButtonToggled(bool state);
	void sendQueryButtonClicked();
	void refreshVarsButtonClicked();
	void refreshWsButtonClicked();
	void showGraphButtonClicked();
	void visualizeButtonClicked();
	void varListCurrentTextChanged(const QString &curText);
	void factorListCurrentTextChanged(const QString &curText);
	void wsTimerTimeout();
	void addObjectPlacePropertyButtonClicked();
	void addObjectSearchResultButtonClicked();
	void addEventToHistory(QString str);


private:

	int getRoomForPlace(ConceptualData::WorldStatePtr wsPtr, int placeId);


private:
    conceptual::Tester *_component;

    QTimer *_wsTimer;
    int _wsCount;

    std::queue<qint64> _wsUpdateTimes;

	pthread_mutex_t _worldStateMutex;
	pthread_mutex_t _eventsMutex;
	ConceptualData::WorldStatePtr _wsPtr;

	bool _collect;

	int _eventNo;
	struct EventInfo
	{
		int curRoomId;
		int curPlaceId;
		ConceptualData::EventInfo info;
		//std::map<std::string,double> curRoomCategories;
	};

	std::vector<EventInfo> _events;

};



#endif // ConceptualWidget_H
