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
	int resultWidth= ui.widthSpinBox->value();
	const int categoriesDist = 5;
	const int rowHeight = 22;
	const int placeSeparator = 10;
	const int roomSeparator = 20;
	const int categorySeparator = 10;
	const int eventSeparator = 10;
	const int eventSeparatorCount = 10;

	// Create scene
	QGraphicsScene *scene = new QGraphicsScene(this);
	pthread_mutex_lock(&_parent->_eventsMutex);

	// Initialization
	bool locationOnly = ui.locationEventsCheckBox->isChecked();
	unsigned long eventCount=0;
	if (locationOnly)
	{
		for (unsigned long e=0; e<eventCount; ++e)
			if (_parent->_events[e].info.type == ConceptualData::EventNothig)
				eventCount++;
	}
	else
		eventCount = _parent->_events.size();
	const DefaultData::StringSeq &roomCats = _component->getRoomCategories();
	int roomCatCount = roomCats.size();
	QFont defaultFont("Ubuntu", 12);

	// Start
	for(unsigned int i=0; i<roomCats.size(); ++i)
	{
		QGraphicsSimpleTextItem *text = scene->addSimpleText(QString::fromStdString(roomCats[i]), defaultFont);
		text->setPos(-text->boundingRect().width()-categoriesDist, i*rowHeight);
		scene->addLine(-categorySeparator,i*rowHeight,resultWidth*eventCount,i*rowHeight);
	}
	scene->addLine(-categorySeparator,roomCatCount*rowHeight,resultWidth*eventCount,roomCatCount*rowHeight);

	// Starting vertical line
	scene->addLine(0,-eventSeparator,0,rowHeight*roomCatCount+roomSeparator);

	// Draw results
	int curPlace=-1;
	int curRoom=-1;
	for (unsigned long e=0; e<eventCount; ++e)
	{
		const ConceptualWidget::Event &event = _parent->_events[e];
		if ((!locationOnly) || (event.info.type == ConceptualData::EventNothig))
		{
			for(unsigned int i=0; i<event.curRoomCategories.size(); ++i)
			{
				double prob = event.curRoomCategories[i];
				scene->addRect(e*resultWidth, i*rowHeight, resultWidth, rowHeight, QPen(), getBrushForProbability(prob));
			}

			bool eventSep =  ((e+1)%eventSeparatorCount) == 0;

			if (event.curPlaceId!=curPlace)
			{
				if (event.curRoomId!=curRoom)
					scene->addLine((e+1)*resultWidth, eventSep?eventSeparator:0,(e+1)*resultWidth,roomCatCount*rowHeight+roomSeparator);
				else
					scene->addLine((e+1)*resultWidth, eventSep?eventSeparator:0 ,(e+1)*resultWidth,roomCatCount*rowHeight+placeSeparator);
			}
			else
				scene->addLine((e+1)*resultWidth, eventSep?eventSeparator:0 ,(e+1)*resultWidth,roomCatCount*rowHeight);
			curPlace=event.curPlaceId;
			curRoom=event.curRoomId;
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


QBrush RCVisualizer::getBrushForProbability(double prob)
{
	double step1 = 0.333333;
	double step2 = 0.666666;

	double r = (prob<=step1)? prob/step1: 1.0;
	double g = ((prob>step1) && (prob<=step2))? (prob-step1)/(step2-step1): ((prob>step1)?1.0:0.0);
	double b = (prob>step2)? ((prob-step2)/(1.0-step1)): 0.0;

	return QBrush (qRgb(r*255.0, g*255.0, b*255.0));
}
