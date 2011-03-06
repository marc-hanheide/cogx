/**
 * @author Andrzej Pronobis
 *
 * Main dialog.
 */

#include "MainDialog.h"
#include "Tester.h"
#include "ConceptualData.hpp"
#include "SpatialProbabilities.hpp"

// Qt & std
#include <QDateTime>
#include <QTimer>
#include <queue>

using namespace conceptual;
using namespace std;
using namespace ConceptualData;

// -------------------------------------------------------
MainDialog::MainDialog(Tester *component)
    : QDialog(0), _component(component)
{
	pthread_mutex_init(&_worldStateMutex, 0);

	setupUi(this);
	queryResultTreeWidget->setHeaderLabel("");
	queryResultTreeWidget->header()->setResizeMode(QHeaderView::ResizeToContents);

	_wsTimer = new QTimer(this);
	_wsCount = 0;
	_wsTimer->start(1000);

	// Signals and slots
	connect(sendQueryButton, SIGNAL(clicked()), this, SLOT(sendQueryButtonClicked()));
	connect(refreshVarsButton, SIGNAL(clicked()), this, SLOT(refreshVarsButtonClicked()));
	connect(refreshWsButton, SIGNAL(clicked()), this, SLOT(refreshWsButtonClicked()));
	connect(showGraphButton, SIGNAL(clicked()), this, SLOT(showGraphButtonClicked()));
	connect(variablesListWidget, SIGNAL(currentTextChanged(const QString &)), this, SLOT(varListCurrentTextChanged(const QString &)));
	connect(_wsTimer, SIGNAL(timeout()), this, SLOT(wsTimerTimeout()));
}


// -------------------------------------------------------
MainDialog::~MainDialog()
{
	pthread_mutex_destroy(&_worldStateMutex);
}


// -------------------------------------------------------
void MainDialog::newWorldState(ConceptualData::WorldStatePtr wsPtr)
{
	pthread_mutex_lock(&_worldStateMutex);
	_wsCount++;
	_wsPtr = wsPtr;
	pthread_mutex_unlock(&_worldStateMutex);
}


// -------------------------------------------------------
void MainDialog::sendQueryButtonClicked()
{
	_component->log("Sending query " + queryComboBox->currentText().toStdString() + ".");

	queryResultTreeWidget->clear();

	SpatialProbabilities::ProbabilityDistribution result =
			_component->sendQueryHandlerQuery(queryComboBox->currentText().toStdString(),
					!standardQueryRadioButton->isChecked());

	if (result.massFunction.size()>0)
	{
		// Setup header
		vector<int> varNos;
		QStringList labels;
		for (map<string, int>::iterator it = result.variableNameToPositionMap.begin();
				it!=result.variableNameToPositionMap.end(); ++it)
		{
			varNos.push_back(it->second);
			labels.append(QString::fromStdString(it->first));
		}
		labels.append("p");

		queryResultTreeWidget->setHeaderLabels(labels);

		// Fill in values
		QList<QTreeWidgetItem *> items;
		for(unsigned int i=0; i<result.massFunction.size(); ++i)
		{
			double probability = result.massFunction[i].probability;

			QStringList values;
			for(unsigned int j=0; j<varNos.size(); ++j)
			{
				QString valueStr;
				if (result.massFunction[i].variableValues[j]->ice_isA("::SpatialProbabilities::StringRandomVariableValue"))
				{
					string value =
							SpatialProbabilities::StringRandomVariableValuePtr::dynamicCast(
									result.massFunction[i].variableValues[j])->value;
					valueStr = QString::fromStdString(value);
				}
				if (result.massFunction[i].variableValues[j]->ice_isA("::SpatialProbabilities::IntRandomVariableValue"))
				{
					int value =
							SpatialProbabilities::IntRandomVariableValuePtr::dynamicCast(
									result.massFunction[i].variableValues[j])->value;
					valueStr = QString::number(value);
				}
				if (result.massFunction[i].variableValues[j]->ice_isA("::SpatialProbabilities::BoolRandomVariableValue"))
				{
					bool value =
							SpatialProbabilities::BoolRandomVariableValuePtr::dynamicCast(
									result.massFunction[i].variableValues[j])->value;
					valueStr = (value)?"true":"false";
				}
				values.append(valueStr);
			}
			values.append(QString::number(probability));
			items.append(new QTreeWidgetItem((QTreeWidget*)0, values));
		}

		queryResultTreeWidget->insertTopLevelItems(0, items);

		// Add to combo
		queryComboBox->insertItem(0, queryComboBox->currentText());
	}
	else
	{
		queryResultTreeWidget->setHeaderLabels(QStringList(" "));
		queryResultTreeWidget->insertTopLevelItem(0,
				new QTreeWidgetItem((QTreeWidget*)0, QStringList("Incorrect querry!")) );
	}
}


// -------------------------------------------------------
void MainDialog::refreshVarsButtonClicked()
{
	_component->log("Refreshing variable list");

	variablesListWidget->clear();
	ConceptualData::VariableInfos vis = _component->getChainGraphVariables();

	for (map<int, ConceptualData::VariableInfo>::iterator i=vis.begin(); i!=vis.end(); ++i)
	{
		variablesListWidget->addItem(QString::fromStdString(i->second.name));
	}
}

// -------------------------------------------------------
void MainDialog::varListCurrentTextChanged(const QString &curText)
{
	if (!curText.isEmpty())
	{
		queryComboBox->setEditText("p("+curText+")");
		sendQueryButtonClicked();
	}
}


// -------------------------------------------------------
void MainDialog::refreshWsButtonClicked()
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
					(new QTreeWidgetItem(objectItem, QStringList("Beta: "+QString::number(oppi.beta))))->
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
void MainDialog::showGraphButtonClicked()
{

}


// -------------------------------------------------------
void MainDialog::wsTimerTimeout()
{
	pthread_mutex_lock(&_worldStateMutex);

	wsFreqLabel->setText(QString::number(_wsCount));
	_wsCount = 0;

	pthread_mutex_unlock(&_worldStateMutex);
}
