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
    : QWidget(parent), _component(component), _prevPlace(-1), _eventNo(0)
{
	pthread_mutex_init(&_worldStateMutex, 0);
	pthread_mutex_init(&_eventsMutex, 0);

	setupUi(this);
	queryResultTreeWidget->setHeaderLabel("");
	queryResultTreeWidget->header()->setResizeMode(QHeaderView::ResizeToContents);

	_wsTimer = new QTimer(this);
	_wsCount = 0;
	_posTimer = new QTimer(this);

	_collect=false;

	// Signals and slots
	qRegisterMetaType<Event>("Event");
	qRegisterMetaType< QList<Event> >("QList<Event>");
	connect(sendQueryButton, SIGNAL(clicked()), this, SLOT(sendQueryButtonClicked()));
	connect(categoriesButton, SIGNAL(clicked()), this, SLOT(categoriesButtonClicked()));
	connect(objectsButton, SIGNAL(clicked()), this, SLOT(objectsButtonClicked()));
	connect(refreshVarsButton, SIGNAL(clicked()), this, SLOT(refreshVarsButtonClicked()));
	connect(refreshWsButton, SIGNAL(clicked()), this, SLOT(refreshWsButtonClicked()));
	connect(showGraphButton, SIGNAL(clicked()), this, SLOT(showGraphButtonClicked()));
	connect(visualizeButton, SIGNAL(clicked()), this, SLOT(visualizeButtonClicked()));
	connect(variablesListWidget, SIGNAL(currentTextChanged(const QString &)), this, SLOT(varListCurrentTextChanged(const QString &)));
	connect(factorsListWidget, SIGNAL(currentTextChanged(const QString &)), this, SLOT(factorListCurrentTextChanged(const QString &)));
	connect(_wsTimer, SIGNAL(timeout()), this, SLOT(wsTimerTimeout()));
	connect(_posTimer, SIGNAL(timeout()), this, SLOT(posTimerTimeout()));
	connect(addObjectPlacePropertyButton, SIGNAL(clicked()), this, SLOT(addObjectPlacePropertyButtonClicked()));
	connect(addObjectSearchResultButton, SIGNAL(clicked()), this, SLOT(addObjectSearchResultButtonClicked()));

	_wsTimer->start(1000);
	_posTimer->start(100);
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
	pthread_mutex_lock(&_worldStateMutex);
	_wsCount++;
	_wsPtr = wsPtr;

	// Get current place and room for the event
	int curPlaceId = _component->getCurrentPlace();
	int curRoomId = getRoomForPlace(wsPtr, curPlaceId);

	QList<Event> events;

	for (unsigned int i=0; i<wsPtr->lastEvents.size(); ++i)
	{
		Event event;
		event.info = wsPtr->lastEvents[i];
		event.curPlaceId = curPlaceId;
		event.curRoomId = curRoomId;
		events.append(event);
	}
	pthread_mutex_unlock(&_worldStateMutex);

	// Take care of the event
	QMetaObject::invokeMethod(this, "addEvent", Qt::QueuedConnection,
	                           Q_ARG(QList<Event>, events));

	// Auto refreshing
	if (autoRefreshQueryCheckBox->isChecked())
	{
		QMetaObject::invokeMethod(sendQueryButton, "click", Qt::QueuedConnection);
	}
	if (autoRefreshWsCheckBox->isChecked())
	{
		QMetaObject::invokeMethod(refreshWsButton, "click", Qt::QueuedConnection);
	}

}


// -------------------------------------------------------
void ConceptualWidget::sendQueryButtonClicked()
{
	if (queryComboBox->currentText().isEmpty())
		return;

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
	wsTreeWidget->clear();

	pthread_mutex_lock(&_worldStateMutex);
	if (!_wsPtr)
	{
		pthread_mutex_unlock(&_worldStateMutex);
		return;
	}

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
void ConceptualWidget::categoriesButtonClicked()
{
	standardQueryRadioButton->setChecked(true);
	queryComboBox->setEditText("p(room*_category)");
	sendQueryButtonClicked();
}


// -------------------------------------------------------
void ConceptualWidget::objectsButtonClicked()
{
	standardQueryRadioButton->setChecked(true);
	queryComboBox->setEditText("p(room*_object_*_unexplored)");
	sendQueryButtonClicked();
}


// -------------------------------------------------------
void ConceptualWidget::addEvent(QList<Event> events)
{
	for (int i=0; i<events.size(); ++i)
	{
		Event &event = events[i];

		// Increment the overall event number
		pthread_mutex_lock(&_eventsMutex);
		int eventNo=++_eventNo;
		pthread_mutex_unlock(&_eventsMutex);

		// Generate
		QString eventStr = QString::number(eventNo)+": ";
		switch(event.info.type)
		{
		case ConceptualData::EventRoomAdded:
			eventStr+="RoomAdded (rid="+QString::number(event.info.roomId)+")";
			break;
		case ConceptualData::EventRoomDeleted:
			eventStr+="RoomDeleted (rid="+QString::number(event.info.roomId)+")";
			break;
		case ConceptualData::EventRoomPlaceAdded:
			eventStr+="RoomPlaceAdded (rid="+QString::number(event.info.roomId)+", pid="+QString::number(event.info.place1Id)+")";
			break;
		case ConceptualData::EventRoomPlaceDeleted:
			eventStr+="RoomPlaceDeleted (rid="+QString::number(event.info.roomId)+", pid="+QString::number(event.info.place1Id)+")";
			break;
		case ConceptualData::EventPlaceStatusChanged:
			eventStr+="PlaceStatusChanged (pid="+QString::number(event.info.place1Id)+")";
			break;
		case ConceptualData::EventGatewayPlacePropertyChanged:
			eventStr+="GatewayPlacePropertyChanged (pid="+QString::number(event.info.place1Id)+")";
			break;
		case ConceptualData::EventObjectPlacePropertyAdded:
			if (event.info.place1Id>0)
				eventStr+="ObjectPlacePropertyAdded (pid="+QString::number(event.info.place1Id)+
						", obj="+QString::fromStdString(event.info.propertyInfo)+")";
			else
				eventStr+="ObjectPlacePropertyAdded (rid="+QString::number(event.info.roomId)+
						", obj="+QString::fromStdString(event.info.propertyInfo)+")";

			break;
		case ConceptualData::EventObjectPlacePropertyDeleted:
			if (event.info.place1Id>0)
				eventStr+="ObjectPlacePropertyDeleted (pid="+QString::number(event.info.place1Id)+
						", obj="+QString::fromStdString(event.info.propertyInfo)+")";
			else
				eventStr+="ObjectPlacePropertyAdded (rid="+QString::number(event.info.roomId)+
						", obj="+QString::fromStdString(event.info.propertyInfo)+")";
			break;
		case ConceptualData::EventObjectPlacePropertyChanged:
			if (event.info.place1Id>0)
				eventStr+="ObjectPlacePropertyChanged (pid="+QString::number(event.info.place1Id)+
						", obj="+QString::fromStdString(event.info.propertyInfo)+")";
			else
				eventStr+="ObjectPlacePropertyAdded (rid="+QString::number(event.info.roomId)+
						", obj="+QString::fromStdString(event.info.propertyInfo)+")";
			break;
		case ConceptualData::EventShapePlacePropertyAdded:
			eventStr+="ShapePlacePropertyAdded (pid="+QString::number(event.info.place1Id)+")";
			break;
		case ConceptualData::EventShapePlacePropertyDeleted:
			eventStr+="ShapePlacePropertyDeleted (pid="+QString::number(event.info.place1Id)+")";
			break;
		case ConceptualData::EventShapePlacePropertyChanged:
			eventStr+="ShapePlacePropertyChanged (pid="+QString::number(event.info.place1Id)+")";
			break;
		case ConceptualData::EventAppearancePlacePropertyAdded:
			eventStr+="AppearancePlacePropertyAdded (pid="+QString::number(event.info.place1Id)+")";
			break;
		case ConceptualData::EventAppearancePlacePropertyDeleted:
			eventStr+="AppearancePlacePropertyDeleted (pid="+QString::number(event.info.place1Id)+")";
			break;
		case ConceptualData::EventAppearancePlacePropertyChanged:
			eventStr+="AppearancePlacePropertyChanged (pid="+QString::number(event.info.place1Id)+")";
			break;
		case ConceptualData::EventRoomConnectivityChanged:
			eventStr+="RoomConnectivityChanged (pid1="+QString::number(event.info.place1Id)+", pid2="+QString::number(event.info.place2Id)+")";
			break;
		default: // Location event?
			eventStr+="Changed place (pid="+
					QString::number(event.curPlaceId)+", rid="+QString::number(event.curRoomId)+")";
			break;
		}

		eventHistoryListWidget->insertItem(0, eventStr);
	}

	if (collectInfoCheckBox->isChecked())
	{
		collectEventInfo(events);
	}

}


// -------------------------------------------------------
void ConceptualWidget::collectEventInfo(QList<Event> events)
{
	Event event = events[0];


	// Get categories for the current room
	ConceptualData::ProbabilityDistributions results =
			_component->sendQueryHandlerQuery("p(room"+lexical_cast<string>(event.curRoomId)+"_category)", false, false);
	const DefaultData::StringSeq &roomCats = _component->getRoomCategories();
	if (results.size()>0)
	{
		SpatialProbabilities::ProbabilityDistribution result = results[0];
		event.curRoomCategories.resize(result.massFunction.size());
		for(unsigned int i=0; i<result.massFunction.size(); ++i)
		{
			double probability = result.massFunction[i].probability;
			string value =
					SpatialProbabilities::StringRandomVariableValuePtr::dynamicCast(
							result.massFunction[i].variableValues[0])->value;
			int roomIndex=-1;
			for(unsigned int j=0; j<roomCats.size(); ++j)
			{
				if (roomCats[j]==value)
				{
					roomIndex=j;
					break;
				}
			}
			event.curRoomCategories[roomIndex]=probability;
		} //for
	} // if


	// Get shape and appearance for the current place
	results =
			_component->sendQueryHandlerQuery("p(place"+lexical_cast<string>(event.curPlaceId)+"_shape_property)", false, false);
	const DefaultData::StringSeq &shapes = _component->getShapes();
	if (results.size()>0)
	{
		SpatialProbabilities::ProbabilityDistribution result = results[0];
		event.curShapes.resize(result.massFunction.size());
		for(unsigned int i=0; i<result.massFunction.size(); ++i)
		{
			double probability = result.massFunction[i].probability;
			string value =
					SpatialProbabilities::StringRandomVariableValuePtr::dynamicCast(
							result.massFunction[i].variableValues[0])->value;
			int shapeIndex=-1;
			for(unsigned int j=0; j<shapes.size(); ++j)
			{
				if (shapes[j]==value)
				{
					shapeIndex=j;
					break;
				}
			}
			event.curShapes[shapeIndex]=probability;
		} //for
	} // if


	// Get shape and appearance for the current place
	results =
			_component->sendQueryHandlerQuery("p(place"+lexical_cast<string>(event.curPlaceId)+"_appearance_property)", false, false);
	const DefaultData::StringSeq &appearances = _component->getAppearances();
	if (results.size()>0)
	{
		SpatialProbabilities::ProbabilityDistribution result = results[0];
		event.curAppearances.resize(result.massFunction.size());
		for(unsigned int i=0; i<result.massFunction.size(); ++i)
		{
			double probability = result.massFunction[i].probability;
			string value =
					SpatialProbabilities::StringRandomVariableValuePtr::dynamicCast(
							result.massFunction[i].variableValues[0])->value;
			int appearanceIndex=-1;
			for(unsigned int j=0; j<appearances.size(); ++j)
			{
				if (appearances[j]==value)
				{
					appearanceIndex=j;
					break;
				}
			}
			event.curAppearances[appearanceIndex]=probability;
		} //for
	} // if

	if (event.curRoomId >=0)
	{
		pthread_mutex_lock(&_eventsMutex);
		_events.push_back(event);
		pthread_mutex_unlock(&_eventsMutex);

	}
}


// -------------------------------------------------------
void ConceptualWidget::posTimerTimeout()
{
	int curPlaceId = _component->getCurrentPlace();

	// Did we change place?
	if (_prevPlace!=curPlaceId)
	{
		_prevPlace = curPlaceId;

		// Get room Id
		pthread_mutex_lock(&_worldStateMutex);
		int curRoomId = -1;
		if (_wsPtr)
			curRoomId = getRoomForPlace(_wsPtr, curPlaceId);
		pthread_mutex_unlock(&_worldStateMutex);

		// Prepare event
		Event event;
		event.info.type = ConceptualData::EventNothig;
		// Get current place
		event.curPlaceId = curPlaceId;
		// Get current room
		event.curRoomId = curRoomId;

		QList<Event> events;
		events.append(event);

		addEvent(events);
	}
}
