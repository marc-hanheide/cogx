#include "RCVisualizer.h"
#include "ConceptualWidget.h"
#include "DefaultData.hpp"
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

	generate();
}


RCVisualizer::~RCVisualizer()
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
	const int horizSeparator = 30;
	const int horizSepWidth = 2;
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

	// Get displayed event count
	unsigned long eventCount=0;
	if (locationOnly)
	{
		for (unsigned long e=0; e<_parent->_events.size(); ++e)
			if (_parent->_events[e].info.type == ConceptualData::EventNothig)
				eventCount++;
	}
	else
		eventCount = _parent->_events.size();

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
	for (unsigned long _e=0; _e<_parent->_events.size(); ++_e)
	{
		const ConceptualWidget::Event &event = _parent->_events[_e];
		if ((!locationOnly) || (event.info.type == ConceptualData::EventNothig))
		{
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

			// Draw shapes
			if ( (event.info.type == ConceptualData::EventShapePlacePropertyAdded) ||
				 (event.info.type == ConceptualData::EventShapePlacePropertyChanged) )
			{
				row = roomCatCount;
				for(unsigned int i=0; i<shapeCountReal; ++i)
				{
					double prob = event.curShapes[i];
					scene->addRect(e*resultWidth, row*rowHeight, resultWidth, rowHeight,
							(verticalLines)?QPen():QPen(Qt::NoPen), getBrushForProbability(prob));
					row++;
				}
			}

			// Draw appearances
			if ( (event.info.type == ConceptualData::EventAppearancePlacePropertyAdded) ||
				 (event.info.type == ConceptualData::EventAppearancePlacePropertyChanged) )
			{
				row = roomCatCount + shapeCount;
				for(unsigned int i=0; i<appearanceCountReal; ++i)
				{
					double prob = event.curAppearances[i];
					scene->addRect(e*resultWidth, row*rowHeight, resultWidth, rowHeight,
							(verticalLines)?QPen():QPen(Qt::NoPen), getBrushForProbability(prob));
					row++;
				}
			}

			if ((e%eventSeparatorCount) == 0)
			{
				scene->addLine(e*resultWidth, -eventSeparator ,e*resultWidth, 0);
				QGraphicsSimpleTextItem *text = scene->addSimpleText(QString::number(e), defaultFont);
				text->setPos(e*resultWidth-text->boundingRect().width()/2, -eventSeparator-text->boundingRect().height());
				if (verticalIndicators)
					scene->addLine(e*resultWidth, 0 ,e*resultWidth, rowCount*rowHeight, QPen(Qt::DotLine));

			}
			if (event.curPlaceId!=curPlace)
			{
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
					scene->addLine(e*resultWidth, rowCount*rowHeight,e*resultWidth,(rowCount+2)*rowHeight);
					lastRoomChange=e*resultWidth;
					if (verticalIndicators)
						scene->addLine(e*resultWidth, 0 ,e*resultWidth, rowCount*rowHeight, QPen(Qt::DashLine));
				}
				else
				{
					scene->addLine(e*resultWidth, rowCount*rowHeight,e*resultWidth,(rowCount+1)*rowHeight, QPen(Qt::DotLine));
				}
			}

			curPlace=event.curPlaceId;
			curRoom=event.curRoomId;
			e++;
		}
	}

	// Start
	for(unsigned int i=0; i<roomCatCount; ++i)
	{
		if (i==0)
			scene->addLine(-horizSeparator,i*rowHeight,resultWidth*eventCount,i*rowHeight,
					QPen(QBrush("black"), horizSepWidth, Qt::SolidLine)); // Horizontal line
		else
			scene->addLine(-categorySeparator,i*rowHeight,resultWidth*eventCount,i*rowHeight); // Horizontal line
		text = scene->addSimpleText(QString::fromStdString(roomCats[i]), defaultFont);
		text->setPos(-text->boundingRect().width()-categoriesDist, i*rowHeight);
	}
	scene->addLine(-horizSeparator,roomCatCount*rowHeight,resultWidth*eventCount,roomCatCount*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	for(unsigned int i=0; i<shapeCount; ++i)
	{
		scene->addLine(-categorySeparator,(i+roomCatCount)*rowHeight,resultWidth*eventCount,(i+roomCatCount)*rowHeight); // Horizontal line
		text = scene->addSimpleText(QString::fromStdString(shapes[i]), defaultFont);
		text->setPos(-text->boundingRect().width()-categoriesDist, (i+roomCatCount)*rowHeight);
	}
	scene->addLine(-horizSeparator,(roomCatCount+shapeCount)*rowHeight,resultWidth*eventCount,(roomCatCount+shapeCount)*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	for(unsigned int i=0; i<appearanceCount; ++i)
	{
		scene->addLine(-categorySeparator,(i+roomCatCount+shapeCount)*rowHeight,
				resultWidth*eventCount,(i+roomCatCount+shapeCount)*rowHeight); // Horizontal line
		text = scene->addSimpleText(QString::fromStdString(appearances[i]), defaultFont);
		text->setPos(-text->boundingRect().width()-categoriesDist, (i+roomCatCount+shapeCount)*rowHeight);
	}
	scene->addLine(-horizSeparator,rowCount*rowHeight,resultWidth*eventCount,rowCount*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	text = scene->addSimpleText("Place", defaultFont);
	text->setPos(-text->boundingRect().width()-categoriesDist, rowCount*rowHeight);
	scene->addLine(-categorySeparator,(rowCount+1)*rowHeight,resultWidth*eventCount,(rowCount+1)*rowHeight);
	text = scene->addSimpleText("Room", defaultFont);
	text->setPos(-text->boundingRect().width()-categoriesDist, (rowCount+1)*rowHeight);
	scene->addLine(-horizSeparator,(rowCount+2)*rowHeight,resultWidth*eventCount,(rowCount+2)*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));

	// Starting and ending vertical line
	scene->addLine(0,-eventSeparator,0,rowHeight*(rowCount+2), QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	scene->addLine((e)*resultWidth,-eventSeparator,(e)*resultWidth,rowHeight*(rowCount+2),
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));

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
