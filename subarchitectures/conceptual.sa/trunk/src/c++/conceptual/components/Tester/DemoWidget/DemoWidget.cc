#include "DemoWidget.h"
#include "Tester.h"
#include "AddGroundtruthDialog.h"
#include "SpatialProperties.hpp"

using namespace std;

DemoWidget::DemoWidget(QWidget *parent, conceptual::Tester *component)
    : QWidget(parent), _component(component), _enabled(false), _curPlaceId(0), _curRoomId(0)
{
	ui.setupUi(this);
	connect(ui.enabledCheckBox, SIGNAL(toggled(bool)), this, SLOT(enabledToggled(bool)));
	connect(ui.addGroundTruthButton, SIGNAL(clicked()), this, SLOT(addGroundtruthButtonClicked()));
	connect(ui.addObjectButton, SIGNAL(clicked()), this, SLOT(addObjectButtonClicked()));

	qRegisterMetaType< QVector<double> >("QVector<double>");
	qRegisterMetaType< QList<double> >("QList<double>");

	// Color map for images
	for (int i=0; i<256; ++i)
		_colorMap.append(qRgb(i, i, i));

	_roomCats = &_component->getRoomCategories();
	ui.eventView->setDisplayedEntities(*_roomCats,
				_component->getShapes(), _component->getSizes(), _component->getAppearances(),
				_component->getVisualizedObjects());
	ui.legendView->setDisplayedEntities(*_roomCats,
				_component->getShapes(), _component->getSizes(), _component->getAppearances(),
				_component->getVisualizedObjects());

	const std::vector<std::string> &vos = _component->getVisualizedObjects();
	for (size_t i=0; i<vos.size(); ++i)
		ui.objectsListWidget->addItem(QString::fromStdString(vos[i]));

	pthread_mutex_init(&_worldStateMutex, 0);

}


DemoWidget::~DemoWidget()
{
	pthread_mutex_destroy(&_worldStateMutex);
}


void DemoWidget::enabledToggled(bool state)
{
	_enabled = state;
}


void DemoWidget::newVisualResults(CategoricalData::VisualResultsPtr vrPtr)
{
	if ((vrPtr->status == CategoricalData::DsValid) && (_enabled))
	{
		QStringList names;
		QList<double> values;
		for (size_t i=0; i<vrPtr->results.size(); ++i)
		{
			names.append(vrPtr->results[i].className.c_str());
			values.append(vrPtr->results[i].confidence);
		}
		QMetaObject::invokeMethod(this, "newAppearanceResults", Qt::QueuedConnection,
		                           Q_ARG(QStringList, names),
		                           Q_ARG(QList<double>, values));
	}
}


void DemoWidget::newLaserResults(CategoricalData::LaserResultsPtr lrPtr)
{
	if ((lrPtr->status == CategoricalData::DsValid) && (_enabled))
	{
		{
			QStringList names;
			QList<double> values;
			for (size_t i=0; i<lrPtr->results.size(); ++i)
			{
				names.append(lrPtr->results[i].className.c_str());
				values.append(lrPtr->results[i].confidence);
			}
			QMetaObject::invokeMethod(this, "newShapeResults", Qt::QueuedConnection,
					Q_ARG(QStringList, names),
					Q_ARG(QList<double>, values));
		}
		if (lrPtr->useSize)
		{
			QStringList names;
			QList<double> values;
			for (size_t i=0; i<lrPtr->sizeResults.size(); ++i)
			{
				names.append(lrPtr->sizeResults[i].className.c_str());
				values.append(lrPtr->sizeResults[i].confidence);
			}
			QMetaObject::invokeMethod(this, "newSizeResults", Qt::QueuedConnection,
					Q_ARG(QStringList, names),
					Q_ARG(QList<double>, values));
		}
	}
}


void DemoWidget::newLaserScan(CategoricalData::LaserScanPtr lsPtr)
{
	if ((lsPtr->status == CategoricalData::DsValid) && (_enabled))
	{
	    // Convert scan info
	    QVector<double> range(lsPtr->scanBuffer.ranges.size());
	    for(unsigned int i=0; i<lsPtr->scanBuffer.ranges.size(); ++i)
	      range[i]=lsPtr->scanBuffer.ranges[i];
	    double max=lsPtr->scanBuffer.maxRange;
	    double startAngle=lsPtr->scanBuffer.startAngle;
	    double angleStep=lsPtr->scanBuffer.angleStep;

		QMetaObject::invokeMethod(ui.laserScanView, "addScan", Qt::QueuedConnection,
		                           Q_ARG(double, startAngle),
		                           Q_ARG(double, angleStep),
		                           Q_ARG(double, max),
		                           Q_ARG(QVector<double>, range),
		                           Q_ARG(double, static_cast<double>(lsPtr->realTimeStamp.s) + static_cast<double>(lsPtr->realTimeStamp.us)/1000000.0));
	}
}


void DemoWidget::newImage(CategoricalData::ImagePtr iPtr)
{
	if ((iPtr->status == CategoricalData::DsValid) && (_enabled))
	{
		// Convert image to pixmap
		int w=iPtr->imageBuffer.width;
		int h=iPtr->imageBuffer.height;
		QImage tmpImage(w/2+1, h/2+1, QImage::Format_Indexed8);
		tmpImage.setColorTable(_colorMap);

		for (int i=0; i<h; i+=2)
			for (int j=0; j<w; j+=2)
				tmpImage.setPixel(j/2, i/2, (unsigned char)(iPtr->imageBuffer.data[i*w+j]) );

		QMetaObject::invokeMethod(this, "setImage", Qt::QueuedConnection,
		                           Q_ARG(QImage, tmpImage),
		                           Q_ARG(int, iPtr->frameNo));
	}
}


void DemoWidget::setImage(QImage img, int frame)
{
	ui.imageLabel->setPixmap(QPixmap::fromImage(img));
}


void DemoWidget::newShapeResults(QStringList shapes, QList<double> values)
{
	QString str = shapes[0];
	for(int i=1; i<shapes.count(); ++i)
	{
		if (values[i]<0.5)
		{
			str+=","+shapes[i];
		}
	}
	ui.curShapeLabel->setText(str);
}


void DemoWidget::newSizeResults(QStringList sizes, QList<double> values)
{
	QString str = sizes[0];
	for(int i=1; i<sizes.count(); ++i)
	{
		if (values[i]<0.5)
		{
			str+=","+sizes[i];
		}
	}
	ui.curSizeLabel->setText(str);
}


void DemoWidget::newAppearanceResults(QStringList appearances, QList<double> values)
{
	QString str = appearances[0];
	for(int i=1; i<appearances.count(); ++i)
	{
		if (values[i]<0.5)
		{
			str+=","+appearances[i];
		}
	}
	ui.curAppearanceLabel->setText(str);

}


void DemoWidget::locationChanged(int placeId)
{
	if (_enabled)
	{
		_curPlaceId = placeId;
		updateStuff();
	}
}


void DemoWidget::updateEvents(const QList<conceptual::ConceptualEvent> &events)
{
	if (_enabled)
	{
		_lastEvents = events;
		ui.eventView->updateEvents(events, true, 10, true, false, false,-1);
		ui.legendView->updateLegend();
	}
}



void DemoWidget::addGroundtruthButtonClicked()
{
	QStringList categories;
	for (unsigned int i=0 ; i<_roomCats->size(); ++i)
		categories.append(QString::fromStdString((*_roomCats)[i]));
	QList<int> roomIds;
	for (std::set<int>::iterator it = _roomIds.begin(); it!=_roomIds.end(); ++it)
		roomIds.append(*it);

	AddGroundtruthDialog *d = new AddGroundtruthDialog(this, roomIds, categories, _curRoomId);
	d->exec();
	if (d->result() == QDialog::Accepted)
	{
		int roomId = d->getRoomId();
		int categoryIndex = d->getCategoryIndex();
		ui.eventView->addGroundTruth(roomId, categoryIndex);
	}

	updateEvents(_lastEvents);
}


void DemoWidget::newWorldState(ConceptualData::WorldStatePtr wsPtr)
{
	if (_enabled)
	{
		pthread_mutex_lock(&_worldStateMutex);
		_wsPtr = wsPtr;
		for (size_t i=0; i<wsPtr->rooms.size(); ++i)
			_roomIds.insert(wsPtr->rooms[i].roomId);
		pthread_mutex_unlock(&_worldStateMutex);

		QMetaObject::invokeMethod(this, "updateStuff", Qt::QueuedConnection);
	}
}


void DemoWidget::updateStuff()
{
	if (!_wsPtr)
		return;

	pthread_mutex_lock(&_worldStateMutex);

	// Get properties for current place
	bool found=false;
	for (size_t r=0; (r<_wsPtr->rooms.size()) && (!found); ++r)
	{
		ConceptualData::ComaRoomInfo &room = _wsPtr->rooms[r];
		for (size_t p=0; p<room.places.size(); ++p)
		{
			ConceptualData::PlaceInfo &place = room.places[p];
			if (place.placeId == _curPlaceId)
			{
				_curRoomId = room.roomId;

				if (place.appearanceProperties.size())
				{
					ConceptualData::ValuePotentialPairs &d = place.appearanceProperties[0].distribution;
					double max=d[0].potential;
					string maxVal=d[0].value;
					for (size_t i=1; i<d.size(); ++i)
					{
						if (d[i].potential>max)
						{
							max=d[i].potential;
							maxVal = d[i].value;
						}
					}
					ui.plAppearanceLabel->setText(QString::fromStdString(maxVal));
				}
				if (place.shapeProperties.size())
				{
					ConceptualData::ValuePotentialPairs &d = place.shapeProperties[0].distribution;
					double max=d[0].potential;
					string maxVal=d[0].value;
					for (size_t i=1; i<d.size(); ++i)
					{
						if (d[i].potential>max)
						{
							max=d[i].potential;
							maxVal = d[i].value;
						}
					}
					ui.plShapeLabel->setText(QString::fromStdString(maxVal));
				}
				if (place.sizeProperties.size())
				{
					ConceptualData::ValuePotentialPairs &d = place.sizeProperties[0].distribution;
					double max=d[0].potential;
					string maxVal=d[0].value;
					for (size_t i=1; i<d.size(); ++i)
					{
						if (d[i].potential>max)
						{
							max=d[i].potential;
							maxVal = d[i].value;
						}
					}
					ui.plSizeLabel->setText(QString::fromStdString(maxVal));
				}
				// Objects in current room
				ui.objectsTreeWidget->clear();
				for (size_t o=0; o<room.objectProperties.size();o++)
				{
					ConceptualData::ObjectPlacePropertyInfo &object = room.objectProperties[o];
					QStringList s;
					s.append(QString::fromStdString(object.category));
					s.append(QString::number(object.count));
					s.append(QString::number(object.beta*100.0, 'f', 2));
					ui.objectsTreeWidget->addTopLevelItem(new QTreeWidgetItem(s));
				}
				found=true;
				break;
			}
		}
	}

	// Update current room
	ui.placeLabel->setText(QString::number(_curPlaceId));
	ui.roomLabel->setText(QString::number(_curRoomId));

	pthread_mutex_unlock(&_worldStateMutex);
}


void DemoWidget::addObjectButtonClicked()
{
	if (!ui.objectsListWidget->currentItem())
		return;

	string object = ui.objectsListWidget->currentItem()->text().toStdString();


	SpatialProperties::ObjectPlacePropertyPtr opp = new SpatialProperties::ObjectPlaceProperty();
	opp->inferred = false;
	opp->category = object;
	opp->placeId = _curPlaceId;
	opp->relation = SpatialData::INROOM;
	opp->supportObjectCategory = "";
	opp->supportObjectId = "";
	SpatialProperties::DiscreteProbabilityDistributionPtr pd = new SpatialProperties::DiscreteProbabilityDistribution();
	SpatialProperties::ValueProbabilityPair vap1;
		SpatialProperties::BinaryValuePtr bv1 = new SpatialProperties::BinaryValue();
		bv1->value = true;
		vap1.probability = 1.0;
		vap1.value = bv1;
	pd->data.push_back(vap1);
	SpatialProperties::ValueProbabilityPair vap2;
		SpatialProperties::BinaryValuePtr bv2 = new SpatialProperties::BinaryValue();
		bv2->value = false;
		vap2.probability = 0.0;
		vap2.value = bv2;
	pd->data.push_back(vap2);

	opp->distribution = pd;
		SpatialProperties::BinaryValuePtr bv3 = new SpatialProperties::BinaryValue();
		bv3->value = true;
	opp->mapValue = bv3;
	opp->mapValueReliable = true;

	_component->addToWorkingMemory<SpatialProperties::ObjectPlaceProperty>(
			_component->newDataID(), "spatial.sa", opp);



    // Check if we have such result already
	bool exists = false;
	vector< boost::shared_ptr< cast::CASTData< SpatialData::ObjectSearchResult > > > results;
	_component->getWorkingMemoryEntries<SpatialData::ObjectSearchResult> ("spatial.sa", 0 , results);
	for (unsigned int i=0; i<results.size(); ++i)
	{
		if ((results[i]->getData()->searchedObjectCategory == object) &&
			(results[i]->getData()->supportObjectId.empty()) &&
			(results[i]->getData()->supportObjectCategory.empty()) &&
			(results[i]->getData()->relation == SpatialData::INROOM) &&
			(results[i]->getData()->roomId == _curRoomId))
		{
			results[i]->getData()->beta = 1.0;
			_component->overwriteWorkingMemory<SpatialData::ObjectSearchResult>(results[i]->getID(), "spatial.sa", results[i]->getData());
			exists=true;
			break;
		}
	}
	if (!exists)
	{
		SpatialData::ObjectSearchResultPtr osr = new SpatialData::ObjectSearchResult();
		osr->beta = 1.0;
		osr->roomId = _curRoomId;
		osr->searchedObjectCategory = object;
		osr->supportObjectCategory = "";
		osr->supportObjectId = "";
		osr->relation = SpatialData::INROOM;

		_component->addToWorkingMemory<SpatialData::ObjectSearchResult>(
				_component->newDataID(), "spatial.sa", osr);
	}


}



