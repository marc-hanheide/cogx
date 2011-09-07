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
#include "HumanAssertionDialog.h"
#include "EventVisualizer.h"

// Qt & std
#include <QDateTime>
#include <QTimer>
#include <QFileDialog>
#include <QDataStream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <queue>

using namespace conceptual;
using namespace std;
using namespace boost;
using namespace ConceptualData;

// -------------------------------------------------------
ConceptualWidget::ConceptualWidget(QWidget *parent, Tester *component)
    : QWidget(parent), _component(component), _prevPlace(-1), _eventNo(0)
{
	pthread_mutex_init(&_worldStateMutex, 0);

	setupUi(this);
	queryResultTreeWidget->setHeaderLabel("");
	queryResultTreeWidget->header()->setResizeMode(QHeaderView::ResizeToContents);

	_wsTimer = new QTimer(this);
	_wsCount = 0;
	_posTimer = new QTimer(this);

	// Signals and slots
	qRegisterMetaType<conceptual::ConceptualEvent>("conceptual::ConceptualEvent");
	connect(collectInfoCheckBox, SIGNAL(toggled(bool)), this, SLOT(collectInfoCheckBoxToggled(bool)));
	connect(sendQueryButton, SIGNAL(clicked()), this, SLOT(sendQueryButtonClicked()));
	connect(categoriesButton, SIGNAL(clicked()), this, SLOT(categoriesButtonClicked()));
	connect(objectsButton, SIGNAL(clicked()), this, SLOT(objectsButtonClicked()));
	connect(saveEventsButton, SIGNAL(clicked()), this, SLOT(saveEventsButtonClicked()));
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
	connect(addHumanAssertionButton, SIGNAL(clicked()), this, SLOT(addHumanAssertionButtonClicked()));

	_wsTimer->start(1000);
	_posTimer->start(100);
}


// -------------------------------------------------------
ConceptualWidget::~ConceptualWidget()
{
	pthread_mutex_destroy(&_worldStateMutex);
}


// -------------------------------------------------------
void ConceptualWidget::newWorldState(ConceptualData::WorldStatePtr wsPtr)
{
	pthread_mutex_lock(&_worldStateMutex);
	_wsCount++;
	_wsPtr = wsPtr;

	// Get current place and room for the event
	conceptual::ConceptualEvent event;
	event.time = castTimeToSeconds(_component->getCASTTime());
	event.curPlaceId = _component->getCurrentPlace();
	event.curRoomId = getRoomForPlace(wsPtr, event.curPlaceId);
	getPlacesForRoom(wsPtr, event.curRoomId, event.curRoomPlaces);
	event.setInfos(wsPtr->lastEvents);
	pthread_mutex_unlock(&_worldStateMutex);

	// Take care of the event
	QMetaObject::invokeMethod(this, "addEvent", Qt::QueuedConnection,
	                           Q_ARG(conceptual::ConceptualEvent, event));

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
			if (pi.sizeProperties.size())
			{
				QTreeWidgetItem *sizeItem = new QTreeWidgetItem(placeItem, QStringList("Size Place Property"));
				sizeItem->setIcon(0, QIcon(":/icons/icons/shape.png"));
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
			if (pi.humanAssertionProperties.size())
			{
				QTreeWidgetItem *humanAssertionsItem = new QTreeWidgetItem(placeItem, QStringList("Human Assertion Place Properties"));
				humanAssertionsItem->setIcon(0, QIcon(":/icons/icons/humanassertion.png"));
				for (unsigned int h=0; h<pi.humanAssertionProperties.size(); ++h)
				{
					HumanAssertionPlacePropertyInfo happi = pi.humanAssertionProperties[h];
					QTreeWidgetItem *haItem = new QTreeWidgetItem(humanAssertionsItem,
							QStringList(QString::fromStdString(happi.assertion)));
					haItem->setIcon(0, QIcon(":/icons/icons/humanassertion.png"));
				}
			}
		}
		QTreeWidgetItem *placeholdersItem = new QTreeWidgetItem(roomItem, QStringList("Placeholders"));
		placeholdersItem->setIcon(0, QIcon(":/icons/icons/placeholder.png"));
		for (unsigned int p=0; p<cri.placeholders.size(); ++p)
		{
			PlaceholderInfo phi = cri.placeholders[p];
			QTreeWidgetItem *placeholderItem = new QTreeWidgetItem(placeholdersItem, QStringList("placeholder-"+QString::number(phi.placeholderId)));
			placeholderItem->setIcon(0, QIcon(":/icons/icons/placeholder.png"));

			if (phi.gatewayProperties.size())
			{
				QTreeWidgetItem *gatewayItem = new QTreeWidgetItem(placeholderItem, QStringList("Gateway Placeholder Property"));
				gatewayItem->setIcon(0, QIcon(":/icons/icons/doorway.png"));
				(new QTreeWidgetItem(gatewayItem, QStringList("Probability: "+QString::number(phi.gatewayProperties[0].gatewayProbability,'f',2 ))))->
						setIcon(0, QIcon(":/icons/icons/flag-green.png"));
			}
			if (phi.associatedSpaceProperties.size())
			{
				QTreeWidgetItem *associatedSpaceItem = new QTreeWidgetItem(placeholderItem, QStringList("Associated Space Placeholder Property"));
				associatedSpaceItem->setIcon(0, QIcon(":/icons/icons/associatedspace.png"));
				(new QTreeWidgetItem(associatedSpaceItem, QStringList("Value: "+QString::fromStdString(
						lexical_cast<string>(phi.associatedSpaceProperties[0].associatedSpace)))))->
						setIcon(0, QIcon(":/icons/icons/flag-green.png"));
			}
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
void ConceptualWidget::addHumanAssertionButtonClicked()
{
	HumanAssertionDialog *d = new HumanAssertionDialog(this, _component);
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
	EventVisualizer *rcv = new EventVisualizer(this, _component->getRoomCategories(),
			_component->getShapes(), _component->getSizes(), _component->getAppearances(),
			_component->getVisualizedObjects());
	connect(this, SIGNAL(newEventInfo(const QList<conceptual::ConceptualEvent>&)), rcv, SLOT(generate(const QList<conceptual::ConceptualEvent>&)));
	rcv->show();

	rcv->generate(_events);
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
void ConceptualWidget::getPlacesForRoom(ConceptualData::WorldStatePtr wsPtr, int roomId, QList<int> &places)
{
	for (unsigned int i=0; i<wsPtr->rooms.size(); ++i)
	{
		if (wsPtr->rooms[i].roomId == roomId)
		{
			for (unsigned int j=0; j<wsPtr->rooms[i].places.size();++j)
			{
				places.push_back(wsPtr->rooms[i].places[j].placeId);
			}
			break;
		}
	}
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
void ConceptualWidget::addEvent(conceptual::ConceptualEvent event)
{
	for (int i=0; i<event.infos.size(); ++i)
	{
		// Increment the overall event number
		int eventNo=++_eventNo;

		// Generate
		QString eventStr = QString::number(eventNo)+": ";
		switch(event.infos[i].type)
		{
		case ConceptualData::EventRoomAdded:
			eventStr+="RoomAdded (rid="+QString::number(event.infos[i].roomId)+")";
			break;
		case ConceptualData::EventRoomDeleted:
			eventStr+="RoomDeleted (rid="+QString::number(event.infos[i].roomId)+")";
			break;
		case ConceptualData::EventRoomPlaceAdded:
			eventStr+="RoomPlaceAdded (rid="+QString::number(event.infos[i].roomId)+", pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventRoomPlaceDeleted:
			eventStr+="RoomPlaceDeleted (rid="+QString::number(event.infos[i].roomId)+", pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventPlaceholderAdded:
			eventStr+="PlaceholderAdded (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventPlaceholderDeleted:
			eventStr+="PlaceholderDeleted (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventGatewayPlacePropertyChanged:
			eventStr+="GatewayPlacePropertyChanged (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventObjectPlacePropertyAdded:
				eventStr+="ObjectPlacePropertyAdded (pid="+QString::number(event.infos[i].place1Id)+
						", obj="+QString::fromStdString(event.infos[i].propertyInfo)+")";
			break;
		case ConceptualData::EventObjectPlacePropertyDeleted:
				eventStr+="ObjectPlacePropertyDeleted (pid="+QString::number(event.infos[i].place1Id)+
						", obj="+QString::fromStdString(event.infos[i].propertyInfo)+")";
			break;
		case ConceptualData::EventObjectPlacePropertyChanged:
				eventStr+="ObjectPlacePropertyChanged (pid="+QString::number(event.infos[i].place1Id)+
						", obj="+QString::fromStdString(event.infos[i].propertyInfo)+")";
			break;
		case ConceptualData::EventObjectSearchResultAdded:
				eventStr+="ObjectSearchResultAdded (rid="+QString::number(event.infos[i].roomId)+
						", obj="+QString::fromStdString(event.infos[i].propertyInfo)+")";
			break;
		case ConceptualData::EventObjectSearchResultDeleted:
				eventStr+="ObjectSearchResultDeleted (rid="+QString::number(event.infos[i].roomId)+
						", obj="+QString::fromStdString(event.infos[i].propertyInfo)+")";
			break;
		case ConceptualData::EventObjectSearchResultChanged:
				eventStr+="ObjectSearchResultChanged (rid="+QString::number(event.infos[i].roomId)+
						", obj="+QString::fromStdString(event.infos[i].propertyInfo)+")";
			break;
		case ConceptualData::EventShapePlacePropertyAdded:
			eventStr+="ShapePlacePropertyAdded (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventShapePlacePropertyDeleted:
			eventStr+="ShapePlacePropertyDeleted (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventShapePlacePropertyChanged:
			eventStr+="ShapePlacePropertyChanged (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventSizePlacePropertyAdded:
			eventStr+="SizePlacePropertyAdded (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventSizePlacePropertyDeleted:
			eventStr+="SizePlacePropertyDeleted (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventSizePlacePropertyChanged:
			eventStr+="SizePlacePropertyChanged (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventAppearancePlacePropertyAdded:
			eventStr+="AppearancePlacePropertyAdded (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventAppearancePlacePropertyDeleted:
			eventStr+="AppearancePlacePropertyDeleted (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventAppearancePlacePropertyChanged:
			eventStr+="AppearancePlacePropertyChanged (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventHumanAssertionPlacePropertyAdded:
			eventStr+="HumanAssertionPlacePropertyAdded (pid="+QString::number(event.infos[i].place1Id)+
				", assertion="+QString::fromStdString(event.infos[i].propertyInfo)+")";
			break;
		case ConceptualData::EventHumanAssertionPlacePropertyDeleted:
			eventStr+="HumanAssertionPlacePropertyDeleted (pid="+QString::number(event.infos[i].place1Id)+
				", assertion="+QString::fromStdString(event.infos[i].propertyInfo)+")";
			break;
		case ConceptualData::EventHumanAssertionPlacePropertyChanged:
			eventStr+="HumanAssertionPlacePropertyChanged (pid="+QString::number(event.infos[i].place1Id)+
				", assertion="+QString::fromStdString(event.infos[i].propertyInfo)+")";
			break;
		case ConceptualData::EventGatewayPlaceholderPropertyAdded:
			eventStr+="GatewayPlaceholderPropertyAdded (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventGatewayPlaceholderPropertyDeleted:
			eventStr+="GatewayPlaceholderPropertyDeleted (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventGatewayPlaceholderPropertyChanged:
			eventStr+="GatewayPlaceholderPropertyChanged (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventAssociatedSpacePlaceholderPropertyAdded:
			eventStr+="AssociatedSpacePlaceholderPropertyAdded (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventAssociatedSpacePlaceholderPropertyDeleted:
			eventStr+="AssociatedSpacePlaceholderPropertyDeleted (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventAssociatedSpacePlaceholderPropertyChanged:
			eventStr+="AssociatedSpacePlaceholderPropertyChanged (pid="+QString::number(event.infos[i].place1Id)+")";
			break;
		case ConceptualData::EventRoomConnectivityChanged:
			eventStr+="RoomConnectivityChanged (pid1="+QString::number(event.infos[i].place1Id)+", pid2="+QString::number(event.infos[i].place2Id)+")";
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
		collectEventInfo(event);
	}

}


// -------------------------------------------------------
void ConceptualWidget::collectEventInfo(conceptual::ConceptualEvent event)
{
	if (event.curRoomId >=0)
	{
		// Get categories for the current room
		ConceptualData::ProbabilityDistributions results =
				_component->sendQueryHandlerQuery("p(room"+lexical_cast<string>(event.curRoomId)+"_category)", false, false);
		const DefaultData::StringSeq &roomCats = _component->getRoomCategories();
		if (results.size()>0)
		{
			SpatialProbabilities::ProbabilityDistribution result = results[0];
			event.curRoomCategories.clear();
			for(unsigned int i=0; i<result.massFunction.size(); ++i)
				event.curRoomCategories.append(0.0);
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

		// Get shape for the current room
		results =
				_component->sendQueryHandlerQuery("p(place*_shape_property)", false, false);

		const DefaultData::StringSeq &shapes = _component->getShapes();
		for (unsigned int i=0; i<shapes.size(); ++i)
			event.curShapes.push_back(1.0);
		for (unsigned int r=0; r<results.size(); ++r)
		{
			SpatialProbabilities::ProbabilityDistribution result = results[r];
			string varName = result.variableNameToPositionMap.begin()->first;
			erase_first(varName, "place");
			erase_last(varName, "_shape_property");
			int placeId = lexical_cast<int>(varName);

			bool found = false;
			for (int i=0; i<event.curRoomPlaces.size(); ++i)
			{
				if (event.curRoomPlaces[i]==placeId)
				{
					found=true;
					break;
				}
			}
			if (found)
			{
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
					event.curShapes[shapeIndex]*=probability;
				} //for
			}
		}
		// Normalize
		double sum = 0.0;
		for (unsigned int i=0; i<shapes.size(); ++i)
			sum+=event.curShapes[i];
		for (unsigned int i=0; i<shapes.size(); ++i)
			event.curShapes[i]/=sum;




		// Get size for the current room
		results =
				_component->sendQueryHandlerQuery("p(place*_size_property)", false, false);

		const DefaultData::StringSeq &sizes = _component->getSizes();
		for (unsigned int i=0; i<sizes.size(); ++i)
			event.curSizes.push_back(1.0);
		for (unsigned int r=0; r<results.size(); ++r)
		{
			SpatialProbabilities::ProbabilityDistribution result = results[r];
			string varName = result.variableNameToPositionMap.begin()->first;
			erase_first(varName, "place");
			erase_last(varName, "_size_property");
			int placeId = lexical_cast<int>(varName);

			bool found = false;
			for (int i=0; i<event.curRoomPlaces.size(); ++i)
			{
				if (event.curRoomPlaces[i]==placeId)
				{
					found=true;
					break;
				}
			}
			if (found)
			{
				for(unsigned int i=0; i<result.massFunction.size(); ++i)
				{
					double probability = result.massFunction[i].probability;
					string value =
							SpatialProbabilities::StringRandomVariableValuePtr::dynamicCast(
									result.massFunction[i].variableValues[0])->value;
					int sizeIndex=-1;
					for(unsigned int j=0; j<sizes.size(); ++j)
					{
						if (sizes[j]==value)
						{
							sizeIndex=j;
							break;
						}
					}
					event.curSizes[sizeIndex]*=probability;
				} //for
			}
		}
		// Normalize
		sum = 0.0;
		for (unsigned int i=0; i<sizes.size(); ++i)
			sum+=event.curSizes[i];
		for (unsigned int i=0; i<sizes.size(); ++i)
			event.curSizes[i]/=sum;


		// Get appearance for the current room
		results =
				_component->sendQueryHandlerQuery("p(place*_appearance_property)", false, false);

		const DefaultData::StringSeq &appearances = _component->getAppearances();
		for (unsigned int i=0; i<appearances.size(); ++i)
			event.curAppearances.push_back(1.0);
		for (unsigned int r=0; r<results.size(); ++r)
		{
			SpatialProbabilities::ProbabilityDistribution result = results[r];
			string varName = result.variableNameToPositionMap.begin()->first;
			erase_first(varName, "place");
			erase_last(varName, "_appearance_property");
			int placeId = lexical_cast<int>(varName);

			bool found = false;
			for (int i=0; i<event.curRoomPlaces.size(); ++i)
			{
				if (event.curRoomPlaces[i]==placeId)
				{
					found=true;
					break;
				}
			}
			if (found)
			{
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
					event.curAppearances[appearanceIndex]*=probability;
				} //for
			}
		}
		// Normalize
		sum = 0.0;
		for (unsigned int i=0; i<appearances.size(); ++i)
			sum+=event.curAppearances[i];
		for (unsigned int i=0; i<appearances.size(); ++i)
			event.curAppearances[i]/=sum;

		// Get object info for the current room
		vector<string> visualizedObjects = _component->getVisualizedObjects();
		for (unsigned int o=0; o<visualizedObjects.size(); ++o)
		{
			ConceptualData::ProbabilityDistributions resultsUnexplored =
					_component->sendQueryHandlerQuery("p(room"+lexical_cast<string>(event.curRoomId)+
							"_object_"+ visualizedObjects[o] +"_unexplored)", false, false);
			ConceptualData::ProbabilityDistributions resultsExplored =
					_component->sendQueryHandlerQuery("p(room"+lexical_cast<string>(event.curRoomId)+
							"_object_"+ visualizedObjects[o] +"_explored)", false, false);

			// Check if we have an explored and unexplored variable for this object
			double probability = 0.5;
			if ((!resultsExplored.empty()) && (getExistsProbability(resultsExplored[0]) > 0.99))
				probability = 1.0;
			else if (!resultsUnexplored.empty())
				probability = getExistsProbability(resultsUnexplored[0]);

			event.curObjects.push_back(probability);
		}
	}

	_events.push_back(event);

	emit newEventInfo(_events);

	// Save event history to disk
	if (_component->saveEvents())
	{
		double time = castTimeToSeconds(_component->getCASTTime());
		QString fileName = "events/"+QString::number(time, 'f', 6) + ".cevents";
		saveEvents(fileName);
	}
}


// -------------------------------------------------------
double ConceptualWidget::getExistsProbability(SpatialProbabilities::ProbabilityDistribution &probDist)
{
	for(unsigned int i=0; i<probDist.massFunction.size(); ++i)
	{
		if (SpatialProbabilities::StringRandomVariableValuePtr::dynamicCast(probDist.massFunction[i].variableValues[0])->
			value == ConceptualData::EXISTS)
			return probDist.massFunction[i].probability;
	}

	return 0.5;
}


// -------------------------------------------------------
void ConceptualWidget::posTimerTimeout()
{
	int curPlaceId = _component->getCurrentPlace();

	// Did we change place?
	if (_prevPlace!=curPlaceId)
	{
		_prevPlace = curPlaceId;

		conceptual::ConceptualEvent event;
		event.time = castTimeToSeconds(_component->getCASTTime());

		// Get room Id
		pthread_mutex_lock(&_worldStateMutex);
		event.curPlaceId = curPlaceId;
		event.curRoomId = -1;
		if (_wsPtr)
		{
			event.curRoomId = getRoomForPlace(_wsPtr, curPlaceId);
			getPlacesForRoom(_wsPtr, event.curRoomId, event.curRoomPlaces);
		}
		pthread_mutex_unlock(&_worldStateMutex);

		// Prepare event
		ConceptualData::EventInfo info;
		info.time = castTimeToSeconds(_component->getCASTTime());
		info.type = ConceptualData::EventNothig;
		event.infos.push_back(info);

		addEvent(event);
		emit locationChanged(curPlaceId);
	}
}


// -------------------------------------------------------
void ConceptualWidget::collectInfoCheckBoxToggled(bool state)
{
	if (state)
		_events.clear();
}


// -------------------------------------------------------
void ConceptualWidget::saveEventsButtonClicked()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save Conceptual.SA Events"),
	                            "", tr("Conceptual Events (*.cevents)"));
	saveEvents(fileName);
}


// -------------------------------------------------------
double ConceptualWidget::castTimeToSeconds(const cast::cdl::CASTTime &time)
{
  return static_cast<double>(time.s) + 1e-6 * static_cast<double>(time.us);
}


// -------------------------------------------------------
void ConceptualWidget::saveEvents(QString fileName)
{
	if (!fileName.isEmpty())
	{
		 QFile file(fileName);
		 if (file.open(QIODevice::WriteOnly))
		 {
			 QDataStream out(&file);
			 out << _component->getRoomCategories();
			 out << _component->getShapes();
			 out << _component->getSizes();
			 out << _component->getAppearances();
			 out << _component->getVisualizedObjects();
			 out << _events;
			 file.close();
		 }
	}
}
