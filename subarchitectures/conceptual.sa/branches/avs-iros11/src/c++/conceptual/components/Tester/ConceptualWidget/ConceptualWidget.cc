/**
 * @author Andrzej Pronobis
 *
 * Main dialog.
 */

#include "ConceptualWidget.h"
#include "Tester.h"
#include "GraphDialog.h"
#include "ConceptualData.hpp"
#include "SpatialProbabilities.hpp"
#include "ObjectPlacePropertyDialog.h"
#include "ObjectSearchResultDialog.h"
#include "RCVisualizer.h"

// Qt & std
#include <QDateTime>
#include <QTimer>
#include <queue>
#include <boost/lexical_cast.hpp>

using namespace conceptual;
using namespace std;
using namespace boost;
using namespace ConceptualData;

// -------------------------------------------------------
ConceptualWidget::ConceptualWidget(QWidget *parent, Tester *component)
    : QWidget(parent), _component(component), _eventNo(0)
{
	pthread_mutex_init(&_worldStateMutex, 0);
	pthread_mutex_init(&_eventsMutex, 0);

	setupUi(this);
	queryResultTreeWidget->setHeaderLabel("");
	queryResultTreeWidget->header()->setResizeMode(QHeaderView::ResizeToContents);

	_wsTimer = new QTimer(this);
	_wsCount = 0;

	_collect=false;

	// Signals and slots
	connect(this, SIGNAL(addEventToHistorySignal(QString)), this, SLOT(addEventToHistory(QString)));
	connect(collectButton, SIGNAL(toggled(bool)), this, SLOT(collectButtonToggled(bool)));
	connect(sendQueryButton, SIGNAL(clicked()), this, SLOT(sendQueryButtonClicked()));
	connect(refreshVarsButton, SIGNAL(clicked()), this, SLOT(refreshVarsButtonClicked()));
	connect(refreshWsButton, SIGNAL(clicked()), this, SLOT(refreshWsButtonClicked()));
	connect(showGraphButton, SIGNAL(clicked()), this, SLOT(showGraphButtonClicked()));
	connect(visualizeButton, SIGNAL(clicked()), this, SLOT(visualizeButtonClicked()));
	connect(variablesListWidget, SIGNAL(currentTextChanged(const QString &)), this, SLOT(varListCurrentTextChanged(const QString &)));
	connect(factorsListWidget, SIGNAL(currentTextChanged(const QString &)), this, SLOT(factorListCurrentTextChanged(const QString &)));
	connect(_wsTimer, SIGNAL(timeout()), this, SLOT(wsTimerTimeout()));
	connect(addObjectPlacePropertyButton, SIGNAL(clicked()), this, SLOT(addObjectPlacePropertyButtonClicked()));
	connect(addObjectSearchResultButton, SIGNAL(clicked()), this, SLOT(addObjectSearchResultButtonClicked()));

	_wsTimer->start(1000);
}


// -------------------------------------------------------
ConceptualWidget::~ConceptualWidget()
{
	pthread_mutex_destroy(&_worldStateMutex);
	pthread_mutex_destroy(&_eventsMutex);
}


// -------------------------------------------------------
void ConceptualWidget::newWorldState(ConceptualData::WorldStatePtr wsPtr)
{
	QString event = QString::number(_eventNo)+": ";
	switch(wsPtr->lastEvent.type)
	{
	case ConceptualData::EventRoomAdded:
		event+="RoomAdded (rid="+QString::number(wsPtr->lastEvent.roomId)+")";
		break;
	case ConceptualData::EventRoomDeleted:
		event+="RoomDeleted (rid="+QString::number(wsPtr->lastEvent.roomId)+")";
		break;
	case ConceptualData::EventRoomPlaceAdded:
		event+="RoomPlaceAdded (rid="+QString::number(wsPtr->lastEvent.roomId)+", pid="+QString::number(wsPtr->lastEvent.place1Id)+")";
		break;
	case ConceptualData::EventRoomPlaceDeleted:
		event+="RoomPlaceDeleted (rid="+QString::number(wsPtr->lastEvent.roomId)+", pid="+QString::number(wsPtr->lastEvent.place1Id)+")";
		break;
	case ConceptualData::EventPlaceStatusChanged:
		event+="PlaceStatusChanged (pid="+QString::number(wsPtr->lastEvent.place1Id)+")";
		break;
	case ConceptualData::EventGatewayPlacePropertyChanged:
		event+="GatewayPlacePropertyChanged (pid="+QString::number(wsPtr->lastEvent.place1Id)+")";
		break;
	case ConceptualData::EventObjectPlacePropertyAdded:
		if (wsPtr->lastEvent.place1Id>0)
			event+="ObjectPlacePropertyAdded (pid="+QString::number(wsPtr->lastEvent.place1Id)+
					", obj="+QString::fromStdString(wsPtr->lastEvent.propertyInfo)+")";
		else
			event+="ObjectPlacePropertyAdded (rid="+QString::number(wsPtr->lastEvent.roomId)+
					", obj="+QString::fromStdString(wsPtr->lastEvent.propertyInfo)+")";

		break;
	case ConceptualData::EventObjectPlacePropertyDeleted:
		if (wsPtr->lastEvent.place1Id>0)
			event+="ObjectPlacePropertyDeleted (pid="+QString::number(wsPtr->lastEvent.place1Id)+
					", obj="+QString::fromStdString(wsPtr->lastEvent.propertyInfo)+")";
		else
			event+="ObjectPlacePropertyAdded (rid="+QString::number(wsPtr->lastEvent.roomId)+
					", obj="+QString::fromStdString(wsPtr->lastEvent.propertyInfo)+")";
		break;
	case ConceptualData::EventObjectPlacePropertyChanged:
		if (wsPtr->lastEvent.place1Id>0)
			event+="ObjectPlacePropertyChanged (pid="+QString::number(wsPtr->lastEvent.place1Id)+
					", obj="+QString::fromStdString(wsPtr->lastEvent.propertyInfo)+")";
		else
			event+="ObjectPlacePropertyAdded (rid="+QString::number(wsPtr->lastEvent.roomId)+
					", obj="+QString::fromStdString(wsPtr->lastEvent.propertyInfo)+")";
		break;
	case ConceptualData::EventShapePlacePropertyChanged:
		event+="ShapePlacePropertyChanged (pid="+QString::number(wsPtr->lastEvent.place1Id)+")";
		break;
	case ConceptualData::EventAppearancePlacePropertyChanged:
		event+="AppearancePlacePropertyChanged (pid="+QString::number(wsPtr->lastEvent.place1Id)+")";
		break;
	case ConceptualData::EventRoomConnectivityChanged:
		event+="RoomConnectivityChanged (pid1="+QString::number(wsPtr->lastEvent.place1Id)+", pid2="+QString::number(wsPtr->lastEvent.place2Id)+")";
		break;
	default:
		event+="-- Nothing --";
		break;
	}

	pthread_mutex_lock(&_worldStateMutex);
	_wsCount++;
	_wsPtr = wsPtr;
	pthread_mutex_unlock(&_worldStateMutex);

	emit addEventToHistorySignal(event);

/*	if (_collect)
	{
		EventInfo event;
		// Get current place
		event.curPlaceId = _component->getCurrentPlace();
		// Get current room
		event.curRoomId = getRoomForPlace(wsPtr, event.curPlaceId);
		// Get categories for the current room
		ConceptualData::ProbabilityDistributions results =
				_component->sendQueryHandlerQuery("p(room"+lexical_cast<string>(event.curRoomId)+"_categorical)", false, false);
		if (results.size()>0)
		{
			SpatialProbabilities::ProbabilityDistribution result = results[0];
			for(unsigned int i=0; i<result.massFunction.size(); ++i)
			{
				double probability = result.massFunction[i].probability;
				string value =
						SpatialProbabilities::StringRandomVariableValuePtr::dynamicCast(
								result.massFunction[i].variableValues[0])->value;
				event.curRoomCategories[value]=probability;
			} //for
			pthread_mutex_lock(&_eventsMutex);
			_events.push_back(event);
			pthread_mutex_unlock(&_eventsMutex);
		} // if
	} // if*/

}


// -------------------------------------------------------
void ConceptualWidget::sendQueryButtonClicked()
{
	_component->log("Sending query " + queryComboBox->currentText().toStdString() + ".");

	queryResultTreeWidget->clear();

	ConceptualData::ProbabilityDistributions results =
			_component->sendQueryHandlerQuery(queryComboBox->currentText().toStdString(),
					imaginaryQueryRadioButton->isChecked(), factorQueryRadioButton->isChecked());

	if (results.size()>0)
	{
		// Setup header
		QStringList labels;
		for (unsigned int i=0; i<results[0].variableNameToPositionMap.size(); ++i)
			labels.append("Var "+QString::number(i));
		labels.append("p");
		queryResultTreeWidget->setHeaderLabels(labels);

		QList<QTreeWidgetItem *> items;
		for (unsigned int r=0; r<results.size(); ++r)
		{
			// Setup main item
			for (map<string, int>::iterator it = results[r].variableNameToPositionMap.begin();
					it!=results[r].variableNameToPositionMap.end(); ++it)
				labels[it->second] = QString::fromStdString(it->first);
			QTreeWidgetItem *parentItem = new QTreeWidgetItem((QTreeWidget*)0, labels);

			// Fill in values
			for(unsigned int i=0; i<results[r].massFunction.size(); ++i)
			{
				double probability = results[r].massFunction[i].probability;

				QStringList values;
				for(unsigned int j=0; j<results[r].massFunction[i].variableValues.size(); ++j)
				{
					QString valueStr;
					if (results[r].massFunction[i].variableValues[j]->ice_isA("::SpatialProbabilities::StringRandomVariableValue"))
					{
						string value =
								SpatialProbabilities::StringRandomVariableValuePtr::dynamicCast(
										results[r].massFunction[i].variableValues[j])->value;
						valueStr = QString::fromStdString(value);
					}
					if (results[r].massFunction[i].variableValues[j]->ice_isA("::SpatialProbabilities::IntRandomVariableValue"))
					{
						int value =
								SpatialProbabilities::IntRandomVariableValuePtr::dynamicCast(
										results[r].massFunction[i].variableValues[j])->value;
						valueStr = QString::number(value);
					}
					if (results[r].massFunction[i].variableValues[j]->ice_isA("::SpatialProbabilities::BoolRandomVariableValue"))
					{
						bool value =
								SpatialProbabilities::BoolRandomVariableValuePtr::dynamicCast(
										results[r].massFunction[i].variableValues[j])->value;
						valueStr = (value)?"true":"false";
					}
					values.append(valueStr);
				}
				values.append(QString::number(probability));
				new QTreeWidgetItem(parentItem, values);

			} //for
			items.append(parentItem);
		} // for
		queryResultTreeWidget->insertTopLevelItems(0, items);
		queryResultTreeWidget->expandAll();

		// Add to combo
		queryComboBox->insertItem(0, queryComboBox->currentText());
	}
	else
	{
		queryResultTreeWidget->setHeaderLabels(QStringList(" "));
		queryResultTreeWidget->insertTopLevelItem(0,
				new QTreeWidgetItem((QTreeWidget*)0, QStringList("Incorrect query!")) );
	}
}


// -------------------------------------------------------
void ConceptualWidget::refreshVarsButtonClicked()
{
	variablesListWidget->clear();
	factorsListWidget->clear();

	ConceptualData::VariableInfos vis = _component->getChainGraphVariables();

	for (map<int, ConceptualData::VariableInfo>::iterator i=vis.begin(); i!=vis.end(); ++i)
	{
		variablesListWidget->addItem(QString::fromStdString(i->second.name));
	}

	ConceptualData::FactorInfos fis = _component->getChainGraphFactors();

	for (map<int, ConceptualData::FactorInfo>::iterator i=fis.begin(); i!=fis.end(); ++i)
	{
		QString factorName = QString::fromStdString(i->second.name);
		if (factorsListWidget->findItems(factorName, Qt::MatchFixedString | Qt::MatchCaseSensitive).count()==0)
			factorsListWidget->addItem(factorName);
	}

}

// -------------------------------------------------------
void ConceptualWidget::varListCurrentTextChanged(const QString &curText)
{
	if (!curText.isEmpty())
	{
		standardQueryRadioButton->setChecked(true);
		queryComboBox->setEditText("p("+curText+")");
		sendQueryButtonClicked();
	}
}


// -------------------------------------------------------
void ConceptualWidget::factorListCurrentTextChanged(const QString &curText)
{
	if (!curText.isEmpty())
	{
		factorQueryRadioButton->setChecked(true);
		queryComboBox->setEditText(curText);
		sendQueryButtonClicked();
	}
}


// -------------------------------------------------------
void ConceptualWidget::refreshWsButtonClicked()
{
	if (!_wsPtr)
		return;


	pthread_mutex_lock(&_worldStateMutex);

	wsTreeWidget->clear();

	// Rooms
	QTreeWidgetItem *roomsItem = new QTreeWidgetItem(QStringList("Rooms"));
	roomsItem->setIcon(0, QIcon(":/icons/icons/room.png"));
	for (unsigned int r=0; r<_wsPtr->rooms.size(); ++r)
	{
		const ComaRoomInfo &cri = _wsPtr->rooms[r];
		QTreeWidgetItem *roomItem = new QTreeWidgetItem(roomsItem, QStringList("room-"+QString::number(cri.roomId)));
		roomItem->setIcon(0, QIcon(":/icons/icons/room.png"));
		QTreeWidgetItem *placesItem = new QTreeWidgetItem(roomItem, QStringList("Places"));
		placesItem->setIcon(0, QIcon(":/icons/icons/place.png"));
		for (unsigned int p=0; p<cri.places.size(); ++p)
		{
			PlaceInfo pi = cri.places[p];
			QTreeWidgetItem *placeItem = new QTreeWidgetItem(placesItem, QStringList("place-"+QString::number(pi.placeId)));
			placeItem->setIcon(0, QIcon(":/icons/icons/place.png"));

			if (pi.objectProperties.size())
			{
				QTreeWidgetItem *objectsItem = new QTreeWidgetItem(placeItem, QStringList("Object Place Properties"));
				objectsItem->setIcon(0, QIcon(":/icons/icons/object.png"));
				for (unsigned int o=0; o<pi.objectProperties.size(); ++o)
				{
					ObjectPlacePropertyInfo oppi = pi.objectProperties[o];
					QTreeWidgetItem *objectItem;
					if (oppi.relation == SpatialData::INROOM)
					{
						objectItem = new QTreeWidgetItem(objectsItem,
								QStringList(QString::fromStdString(oppi.category)+" in room"));
					}
					else if (oppi.relation == SpatialData::INOBJECT)
					{
						objectItem = new QTreeWidgetItem(objectsItem,
								QStringList(QString::fromStdString(oppi.category)+" in "+
										QString::fromStdString(oppi.supportObjectCategory)+"-"+
										QString::fromStdString(oppi.supportObjectId)));
					}
					else
					{
						objectItem = new QTreeWidgetItem(objectsItem,
								QStringList(QString::fromStdString(oppi.category)+" on "+
										QString::fromStdString(oppi.supportObjectCategory)+"-"+
										QString::fromStdString(oppi.supportObjectId)));
					}
					objectItem->setIcon(0, QIcon(":/icons/icons/object.png"));
					(new QTreeWidgetItem(objectItem, QStringList("Count: "+QString::number(oppi.count))))->
							setIcon(0, QIcon(":/icons/icons/flag-green.png"));
					(new QTreeWidgetItem(objectItem, QStringList("Beta: "+QString::fromStdString(lexical_cast<string>(oppi.beta)))))->
							setIcon(0, QIcon(":/icons/icons/flag-green.png"));
				}
			}
			if (pi.shapeProperties.size())
			{
				QTreeWidgetItem *shapeItem = new QTreeWidgetItem(placeItem, QStringList("Shape Place Property"));
				shapeItem->setIcon(0, QIcon(":/icons/icons/shape.png"));
			}
			if (pi.appearanceProperties.size())
			{
				QTreeWidgetItem *appearanceItem = new QTreeWidgetItem(placeItem, QStringList("Appearance Place Properties"));
				appearanceItem->setIcon(0, QIcon(":/icons/icons/appearance.png"));
			}
		}
		QTreeWidgetItem *placeholdersItem = new QTreeWidgetItem(roomItem, QStringList("Placeholders"));
		placeholdersItem->setIcon(0, QIcon(":/icons/icons/placeholder.png"));
		for (unsigned int p=0; p<cri.placeholders.size(); ++p)
		{
			PlaceholderInfo phi = cri.placeholders[p];
			QTreeWidgetItem *placeholderItem = new QTreeWidgetItem(placeholdersItem, QStringList("placeholder-"+QString::number(phi.placeholderId)));
			placeholderItem->setIcon(0, QIcon(":/icons/icons/placeholder.png"));
		}
		if (cri.objectProperties.size())
		{
			QTreeWidgetItem *objectsItem = new QTreeWidgetItem(roomItem, QStringList("Object Properties"));
			for (unsigned int o=0; o<cri.objectProperties.size(); ++o)
			{
				ObjectPlacePropertyInfo oppi = cri.objectProperties[o];
				QTreeWidgetItem *objectItem;
				if (oppi.relation == SpatialData::INROOM)
				{
					objectItem = new QTreeWidgetItem(objectsItem,
							QStringList(QString::fromStdString(oppi.category)+" in room"));
				}
				else if (oppi.relation == SpatialData::INOBJECT)
				{
					objectItem = new QTreeWidgetItem(objectsItem,
							QStringList(QString::fromStdString(oppi.category)+" in "+
									QString::fromStdString(oppi.supportObjectCategory)+"-"+
									QString::fromStdString(oppi.supportObjectId)));
				}
				else
				{
					objectItem = new QTreeWidgetItem(objectsItem,
							QStringList(QString::fromStdString(oppi.category)+" on "+
									QString::fromStdString(oppi.supportObjectCategory)+"-"+
									QString::fromStdString(oppi.supportObjectId)));
				}
				objectItem->setIcon(0, QIcon(":/icons/icons/object.png"));
				(new QTreeWidgetItem(objectItem, QStringList("Count: "+QString::number(oppi.count))))->
						setIcon(0, QIcon(":/icons/icons/flag-green.png"));
				(new QTreeWidgetItem(objectItem, QStringList("Beta: "+QString::number(oppi.beta))))->
						setIcon(0, QIcon(":/icons/icons/flag-green.png"));
			}
		}
	}
	wsTreeWidget->addTopLevelItem(roomsItem);

	// Room connectivity
	QTreeWidgetItem *connectivityItem = new QTreeWidgetItem(QStringList("Room Connectivity"));
	connectivityItem->setIcon(0, QIcon(":/icons/icons/connectivity.png"));
	for (unsigned int c=0; c<_wsPtr->roomConnections.size(); ++c)
	{
		RoomConnectivityInfo rci = _wsPtr->roomConnections[c];
		(new QTreeWidgetItem(connectivityItem, QStringList("room-"+QString::number(rci.room1Id)+" <-> room"+
				QString::number(rci.room2Id))))->
				setIcon(0, QIcon(":/icons/icons/connectivity.png"));
	}

	wsTreeWidget->addTopLevelItem(connectivityItem);

	// Unlock mutex
	pthread_mutex_unlock(&_worldStateMutex);

	wsTreeWidget->expandAll();
}


// -------------------------------------------------------
void ConceptualWidget::showGraphButtonClicked()
{
	ConceptualData::VariableInfos vis = _component->getChainGraphVariables();
	ConceptualData::FactorInfos fis = _component->getChainGraphFactors();

	GraphDialog *gd = new GraphDialog(this, _component);
	gd->show();
	gd->refresh(vis, fis);

}


// -------------------------------------------------------
void ConceptualWidget::addObjectPlacePropertyButtonClicked()
{
	ObjectPlacePropertyDialog *d = new ObjectPlacePropertyDialog(this, _component);
	d->show();
}


// -------------------------------------------------------
void ConceptualWidget::addObjectSearchResultButtonClicked()
{
	ObjectSearchResultDialog *d = new ObjectSearchResultDialog(this, _component);
	d->show();
}


// -------------------------------------------------------
void ConceptualWidget::wsTimerTimeout()
{
	pthread_mutex_lock(&_worldStateMutex);

	wsFreqLabel->setText(QString::number(_wsCount));
	_wsCount = 0;

	pthread_mutex_unlock(&_worldStateMutex);
}


// -------------------------------------------------------
void ConceptualWidget::visualizeButtonClicked()
{
	RCVisualizer *rcv = new RCVisualizer(this, _component);
	rcv->show();
}


// -------------------------------------------------------
void ConceptualWidget::collectButtonToggled(bool state)
{
	_collect=state;
}


// -------------------------------------------------------
int ConceptualWidget::getRoomForPlace(ConceptualData::WorldStatePtr wsPtr, int placeId)
{
	for (unsigned int i=0; i<wsPtr->rooms.size(); ++i)
	{
		ComaRoomInfo &cri = wsPtr->rooms[i];
		for(unsigned int j=0; j<cri.places.size(); j++)
		{
			PlaceInfo &pi = cri.places[j];
			if (pi.placeId == placeId)
				return cri.roomId;
		}
	}

	return -1;
}


// -------------------------------------------------------
void ConceptualWidget::addEventToHistory(QString str)
{
	eventHistoryListWidget->insertItem(0, str);
}


