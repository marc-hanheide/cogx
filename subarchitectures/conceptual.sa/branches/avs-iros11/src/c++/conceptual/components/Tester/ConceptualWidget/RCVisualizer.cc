#include "RCVisualizer.h"
#include "ConceptualWidget.h"
#include "DefaultData.hpp"
#include "AddGroundtruthDialog.h"
#include "Tester.h"

#include <QFileDialog>
#include <QtSvg/QSvgGenerator>
#include <QGraphicsSimpleTextItem>


RCVisualizer::RCVisualizer(ConceptualWidget *parent, conceptual::Tester *component)
    : QDialog(parent), _parent(parent), _component(component)
{
	ui.setupUi(this);
	connect(ui.saveImageButton, SIGNAL(clicked()), this, SLOT(saveImageButtonClicked()));
	connect(ui.refreshButton, SIGNAL(clicked()), this, SLOT(generate()));
	connect(ui.addGroundtruthButton, SIGNAL(clicked()), this, SLOT(addGroundtruthButtonClicked()));

	generate();
}


RCVisualizer::~RCVisualizer()
{

}


void prepareLocalEvents()
{
}


void RCVisualizer::generate()
{
	// Settings
	const int categoriesDist = 5;
	const int rowHeight = 22;
	const int categorySeparator = 10;
	const int eventSeparator = 10;
	const int eventSeparatorCount = 10;
	const int horizSeparator = 50;
	const int horizSepWidth = 2;
	const double markDotSize=4;
	const double markPlusSize=6;
	QFont defaultFont("Ubuntu", 12);
	QFont smallFont("Ubuntu", 6);

	// Create scene
	QGraphicsScene *scene = new QGraphicsScene(this);
	pthread_mutex_lock(&_parent->_eventsMutex);

	// Initialization
	bool locationOnly = ui.locationEventsCheckBox->isChecked();
	bool placeIds = ui.placesCheckBox->isChecked();
	bool verticalLines = ui.verticalLinesCheckBox->isChecked();
	bool verticalIndicators = ui.verticalIndicatorsCheckBox->isChecked();
	int resultWidth= ui.widthSpinBox->value();
	QGraphicsSimpleTextItem *text;

	// Get data info
	const DefaultData::StringSeq &roomCats = _component->getRoomCategories();
	const DefaultData::StringSeq &shapes = _component->getShapes();
	const DefaultData::StringSeq &appearances = _component->getAppearances();
	unsigned int roomCatCount = roomCats.size();
	unsigned int shapeCount = shapes.size();
	unsigned int appearanceCount = appearances.size();
	unsigned int rowCount = roomCatCount+shapeCount+appearanceCount;
	unsigned int roomCatCountReal = 0;
	unsigned int shapeCountReal = 0;
	unsigned int appearanceCountReal = 0;


	// Draw results
	int curPlace=0;
	int curRoom=0;
	if (_parent->_events.size()>0)
	{
		curPlace=_parent->_events[0].curPlaceId;
		curRoom=_parent->_events[0].curRoomId;
	}
	unsigned long e=0;
	unsigned int lastPlaceChange=0;
	unsigned int lastRoomChange=0;
	std::vector<int> groundTruthRows;
	std::vector<ConceptualData::EventInfo> accumulatedInfos;
	for (unsigned long _e=0; _e<_parent->_events.size(); ++_e)
	{
		const ConceptualWidget::Event &event = _parent->_events[_e];
		for (unsigned int i=0; i<event.infos.size(); ++i)
		{
			if (locationOnly)
			{
				if (event.infos[i].type == ConceptualData::EventNothig)
					accumulatedInfos.push_back(event.infos[i]);
			}
			else
			{
				accumulatedInfos.push_back(event.infos[i]);
			}
		}
		if ((event.curRoomId<0) || (accumulatedInfos.empty()))
			continue;

		// Events
		roomCatCountReal = event.curRoomCategories.size();
		shapeCountReal = event.curShapes.size();
		appearanceCountReal = event.curAppearances.size();

		// Categories
		unsigned int row=0;
		for(unsigned int i=0; i<roomCatCountReal; ++i)
		{
			double prob = event.curRoomCategories[i];
			scene->addRect(e*resultWidth, row*rowHeight, resultWidth, rowHeight,
					(verticalLines)?QPen():QPen(Qt::NoPen), getBrushForProbability(prob));
			row++;
		}

		// Add groundtruth for this room
		if (_groundTruth.find(event.curRoomId) != _groundTruth.end())
			groundTruthRows.push_back(_groundTruth[event.curRoomId]);
		else
			groundTruthRows.push_back(-1);

		// Draw shapes
		row = roomCatCount;
		for(unsigned int i=0; i<shapeCountReal; ++i)
		{
			double prob = event.curShapes[i];
			scene->addRect(e*resultWidth, row*rowHeight, resultWidth, rowHeight,
					(verticalLines)?QPen():QPen(Qt::NoPen), getBrushForProbability(prob));
			row++;
		}
		// Draw appearances
		row = roomCatCount + shapeCount;
		for(unsigned int i=0; i<appearanceCountReal; ++i)
		{
			double prob = event.curAppearances[i];
			scene->addRect(e*resultWidth, row*rowHeight, resultWidth, rowHeight,
					(verticalLines)?QPen():QPen(Qt::NoPen), getBrushForProbability(prob));
			row++;
		}

		if ((e%eventSeparatorCount) == 0)
		{
			scene->addLine(e*resultWidth, -eventSeparator ,e*resultWidth, 0);
			QGraphicsSimpleTextItem *text = scene->addSimpleText(QString::number(e), defaultFont);
			text->setPos(e*resultWidth-text->boundingRect().width()/2, -eventSeparator-text->boundingRect().height());
			if (verticalIndicators)
				scene->addLine(e*resultWidth, 0 ,e*resultWidth, rowCount*rowHeight,
						QPen(QBrush(qRgb(150,150,150)),1, Qt::DotLine));
		}
		if (event.curPlaceId!=curPlace)
		{
			// Are we adding or changing place to already known?
			bool addingPlace = false;
			for (unsigned int i=0; i<accumulatedInfos.size(); ++i)
				if (accumulatedInfos[i].type==ConceptualData::EventRoomPlaceAdded)
					addingPlace = true;

			if (placeIds)
			{
				text = scene->addSimpleText(QString::number(curPlace), smallFont);
				text->setPos((e*resultWidth+lastPlaceChange)/2-text->boundingRect().width()/2, rowCount*rowHeight);
				lastPlaceChange=e*resultWidth;
			}

			if (event.curRoomId!=curRoom)
			{
				text = scene->addSimpleText(QString::number(curRoom), defaultFont);
				text->setPos((e*resultWidth+lastRoomChange)/2-text->boundingRect().width()/2, (rowCount+1)*rowHeight);
				lastRoomChange=e*resultWidth;
				if (verticalIndicators)
					scene->addLine(e*resultWidth, 0 ,e*resultWidth, (rowCount+2)*rowHeight,
							QPen(Qt::DashLine));

				if (addingPlace)
				{
					scene->addLine(e*resultWidth, (rowCount+1.5)*rowHeight-markPlusSize/2.0, e*resultWidth,(rowCount+1.5)*rowHeight+markPlusSize/2.0,
							QPen(QBrush("black"), 2, Qt::SolidLine));
					scene->addLine(e*resultWidth-markPlusSize/2.0, (rowCount+1.5)*rowHeight, e*resultWidth+markPlusSize/2.0,(rowCount+1.5)*rowHeight,
							QPen(QBrush("black"), 2, Qt::SolidLine));
				}
				else
				{
					scene->addEllipse(e*resultWidth-markDotSize/2.0, (rowCount+1.5)*rowHeight-markDotSize/2.0, markDotSize, markDotSize,
							QPen(), QBrush(qRgb(0,0,0)));
				}
			}
			if (addingPlace)
			{
				scene->addLine(e*resultWidth, (rowCount+0.5)*rowHeight-markPlusSize/2.0, e*resultWidth,(rowCount+0.5)*rowHeight+markPlusSize/2.0,
						QPen(QBrush("black"), 2, Qt::SolidLine));
				scene->addLine(e*resultWidth-markPlusSize/2.0, (rowCount+0.5)*rowHeight, e*resultWidth+markPlusSize/2.0,(rowCount+0.5)*rowHeight,
						QPen(QBrush("black"), 2, Qt::SolidLine));
			}
			else
			{
				scene->addEllipse(e*resultWidth-markDotSize/2.0, (rowCount+0.5)*rowHeight-markDotSize/2.0, markDotSize, markDotSize,
						QPen(), QBrush(qRgb(0,0,0)));
			}
		}

		curPlace=event.curPlaceId;
		curRoom=event.curRoomId;
		e++;
		accumulatedInfos.clear();
	}

	// Start
	for(unsigned int i=0; i<roomCatCount; ++i)
	{
		if (i==0)
			scene->addLine(-horizSeparator,i*rowHeight,resultWidth*e,i*rowHeight,
					QPen(QBrush("black"), horizSepWidth, Qt::SolidLine)); // Horizontal line
		else
			scene->addLine(-categorySeparator,i*rowHeight,resultWidth*e,i*rowHeight); // Horizontal line
		text = scene->addSimpleText(QString::fromStdString(roomCats[i]), defaultFont);
		text->setPos(-text->boundingRect().width()-categoriesDist, i*rowHeight);
	}
	scene->addLine(-horizSeparator,roomCatCount*rowHeight,resultWidth*e,roomCatCount*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	for(unsigned int i=0; i<shapeCount; ++i)
	{
		scene->addLine(-categorySeparator,(i+roomCatCount)*rowHeight,resultWidth*e,(i+roomCatCount)*rowHeight); // Horizontal line
		text = scene->addSimpleText(QString::fromStdString(shapes[i]), defaultFont);
		text->setPos(-text->boundingRect().width()-categoriesDist, (i+roomCatCount)*rowHeight);
	}
	scene->addLine(-horizSeparator,(roomCatCount+shapeCount)*rowHeight,resultWidth*e,(roomCatCount+shapeCount)*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	for(unsigned int i=0; i<appearanceCount; ++i)
	{
		scene->addLine(-categorySeparator,(i+roomCatCount+shapeCount)*rowHeight,
				resultWidth*e,(i+roomCatCount+shapeCount)*rowHeight); // Horizontal line
		text = scene->addSimpleText(QString::fromStdString(appearances[i]), defaultFont);
		text->setPos(-text->boundingRect().width()-categoriesDist, (i+roomCatCount+shapeCount)*rowHeight);
	}
	scene->addLine(-horizSeparator,rowCount*rowHeight,resultWidth*e,rowCount*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	text = scene->addSimpleText("Place Change", defaultFont);
	text->setPos(-text->boundingRect().width()-categoriesDist, rowCount*rowHeight);
	scene->addLine(-categorySeparator,(rowCount+1)*rowHeight,resultWidth*e,(rowCount+1)*rowHeight);
	text = scene->addSimpleText("Room Change", defaultFont);
	text->setPos(-text->boundingRect().width()-categoriesDist, (rowCount+1)*rowHeight);
	scene->addLine(-horizSeparator,(rowCount+2)*rowHeight,resultWidth*e,(rowCount+2)*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));

	// Starting and ending vertical line
	scene->addLine(0,-eventSeparator,0,rowHeight*(rowCount+2), QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	scene->addLine((e)*resultWidth,-eventSeparator,(e)*resultWidth,rowHeight*(rowCount+2),
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));

	// Groundtruth
	for (unsigned int i=0; i<groundTruthRows.size(); ++i)
	{
		if (groundTruthRows[i]>=0)
		{
			scene->addLine(i*resultWidth, groundTruthRows[i]*rowHeight, (i+1)*resultWidth, groundTruthRows[i]*rowHeight,
					QPen(QBrush(qRgb(0,0,150)),3, Qt::DashLine));
			scene->addLine(i*resultWidth, (groundTruthRows[i]+1)*rowHeight, (i+1)*resultWidth, (groundTruthRows[i]+1)*rowHeight,
					QPen(QBrush(qRgb(0,0,150)),3, Qt::DashLine));
		}
	}


	pthread_mutex_unlock(&_parent->_eventsMutex);
	ui.graphicsView->setScene(scene);
}



void RCVisualizer::saveImageButtonClicked()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
	                            "", tr("SVG Images (*.svg)"));
	if (!fileName.isEmpty())
	{
		QSvgGenerator generator;
		generator.setFileName(fileName);
		generator.setSize(QSize(200, 200));
		generator.setViewBox(QRect(0, 0, 200, 200));
		generator.setTitle("Room Category Visualization");
		QPainter painter;
		painter.begin(&generator);
		ui.graphicsView->scene()->render(&painter);
		painter.end();
	}
}


void RCVisualizer::addGroundtruthButtonClicked()
{
	QList<int> roomIds;
	pthread_mutex_lock(&_parent->_worldStateMutex);
	for (unsigned int i=0; i<_parent->_wsPtr->rooms.size(); ++i)
		roomIds.append(_parent->_wsPtr->rooms[i].roomId);
	int curPlaceId = _component->getCurrentPlace();
	int curRoomId = _parent->getRoomForPlace(_parent->_wsPtr, curPlaceId);
	pthread_mutex_unlock(&_parent->_worldStateMutex);
	DefaultData::StringSeq roomCategories = _component->getRoomCategories();
	QStringList categories;
	for (unsigned int i=0 ; i<roomCategories.size(); ++i)
		categories.append(QString::fromStdString(roomCategories[i]));

	AddGroundtruthDialog *d = new AddGroundtruthDialog(this, roomIds, categories, curRoomId);
	d->exec();
	if (d->result() == QDialog::Accepted)
	{
		int roomId = d->getRoomId();
		int categoryIndex = d->getCategoryIndex();
		_groundTruth[roomId] = categoryIndex;
	}
}


QBrush RCVisualizer::getBrushForProbability(double prob)
{
	prob=1-prob;
	double step1 = 0.5;
	double step2 = 0.75;

	double r = (prob<=step1)? prob/step1: 1.0;
	double g = ((prob>step1) && (prob<=step2))? (prob-step1)/(step2-step1): ((prob>step1)?1.0:0.0);
	double b = (prob>step2)? ((prob-step2)/(1.0-step1)): 0.0;

	return QBrush (qRgb(r*255.0, g*255.0, b*255.0));
}
