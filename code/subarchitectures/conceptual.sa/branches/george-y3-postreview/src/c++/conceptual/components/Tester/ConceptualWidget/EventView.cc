#include "EventView.h"
#include <QGraphicsSimpleTextItem>


// ------------------------------------------------------
EventView::EventView(QWidget *parent)
    : QGraphicsView(parent),
      _roomCats(0), _shapes(0), _sizes(0), _appearances(0), _visualizedObjects(0),
      _lastUpdatedEvents(false), _lastE(0), _lastResultWidth(0)
{
	setRenderHints(QPainter::Antialiasing);

	_drawLegend=true;
	_drawEvents=true;

	categoriesDist = 5;
	rowHeight = 22;
	categorySeparator = 10;
	eventSeparator = 10;
	eventSeparatorCount = 10;
	horizSeparator = 50;
	horizSepWidth = 2;
	markDotSize=4;
	markPlusSize=6;
	defaultFont = QFont("Ubuntu", 12);
	smallFont = QFont("Ubuntu", 6);
	stdPen = QPen(Qt::black, 1, Qt::SolidLine);
	gtPen = QPen(Qt::black, 5, Qt::DotLine);
	gtPenBg = QPen(Qt::white, 5, Qt::SolidLine);
}


// ------------------------------------------------------
EventView::~EventView()
{

}


// ------------------------------------------------------
void EventView::updateAll(const QList<conceptual::ConceptualEvent> &events, bool placeIds,
		int resultWidth, bool verticalIndicators, bool verticalLines, bool locationOnly)
{
	QGraphicsScene *scene = new QGraphicsScene(this);
	_lastE = drawEvents(scene, events, placeIds,	resultWidth, verticalIndicators, verticalLines, locationOnly, -1);
	drawLegend(scene);
	setScene(scene);

	_lastResultWidth = resultWidth;
	_lastUpdatedEvents=true;
	fit();
}



// ------------------------------------------------------
void EventView::updateEvents(const QList<conceptual::ConceptualEvent> &events, bool placeIds,
		int resultWidth, bool verticalIndicators, bool verticalLines, bool locationOnly, double time)
{
	QGraphicsScene *scene = new QGraphicsScene(this);
	_lastE = drawEvents(scene, events, placeIds,	resultWidth, verticalIndicators, verticalLines, locationOnly, time);
	setScene(scene);

	_lastResultWidth = resultWidth;
	_lastUpdatedEvents=true;
	fit();
}


// ------------------------------------------------------
void EventView::updateLegend()
{
	QGraphicsScene *scene = new QGraphicsScene(this);
	drawLegend(scene);
	setScene(scene);

	_lastUpdatedEvents=false;
	fit();
}


// ------------------------------------------------------
void EventView::fit()
{
	if ( _roomCats&&_shapes&&_sizes&&_appearances&&_visualizedObjects)
	{
		QGraphicsSimpleTextItem text("100");
		text.setFont(defaultFont);
		int top = -eventSeparator-text.boundingRect().height();
		unsigned int rowCount = _visualizedObjects->size()+_roomCats->size()+_shapes->size()+_sizes->size()+_appearances->size();

		if (_lastUpdatedEvents)
			fitInView(_lastE*_lastResultWidth, top, 10, -top + rowHeight*(rowCount+6), Qt::KeepAspectRatio);
		else
			fitInView(0, top-10, 10, -top-10 + rowHeight*(rowCount+6)+10, Qt::KeepAspectRatio);
	}
}


// ------------------------------------------------------
int EventView::drawEvents(QGraphicsScene *scene, const QList<conceptual::ConceptualEvent> &events, bool placeIds,
		int resultWidth, bool verticalIndicators, bool verticalLines, bool locationOnly, double time)
{
	// Initialization
	scene->setBackgroundBrush(Qt::white);
	QGraphicsSimpleTextItem *text;

	// Get data info
	unsigned int objectCount = _visualizedObjects->size();
	unsigned int roomCatCount = _roomCats->size();
	unsigned int shapeCount = _shapes->size();
	unsigned int sizeCount = _sizes->size();
	unsigned int appearanceCount = _appearances->size();
	unsigned int rowCount = roomCatCount+shapeCount+sizeCount+appearanceCount+objectCount;
	unsigned int roomCatCountReal = 0;
	unsigned int shapeCountReal = 0;
	unsigned int sizeCountReal = 0;
	unsigned int appearanceCountReal = 0;
	unsigned int objectsCountReal = 0;


	// Draw results
	int curPlace=0;
	int curRoom=0;
	if (events.size()>0)
	{
		curPlace=events[0].curPlaceId;
		curRoom=events[0].curRoomId;
	}
	long e=0;
	unsigned int lastPlaceChange=0;
	unsigned int lastRoomChange=0;
	std::vector<int> groundTruthRows;
	std::vector<int> mapRows;
	std::vector<int> mapShapeRows;
	std::vector<int> mapSizeRows;
	std::vector<int> mapAppearanceRows;
	std::vector<ConceptualData::EventInfo> accumulatedInfos;
	for (long _e=0; _e<events.size() && ((time<0) || (events[_e].time<=time)); ++_e)
	{
		const conceptual::ConceptualEvent &event = events[_e];

		for (int i=0; i<event.infos.size(); ++i)
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
		sizeCountReal = event.curSizes.size();
		appearanceCountReal = event.curAppearances.size();
		objectsCountReal = event.curObjects.size();

		// Categories
		unsigned int row=0;
		unsigned int mapIndex = -1;
		double mapValue = -1;
		for(unsigned int i=0; i<roomCatCountReal; ++i)
		{
			double prob = event.curRoomCategories[i];
			if (prob>mapValue)
			{
				mapValue = prob;
				mapIndex = i;
			}
			scene->addRect(e*resultWidth, row*rowHeight, resultWidth, rowHeight,
					(verticalLines)?QPen():QPen(Qt::NoPen), getBrushForProbability(prob));
			row++;
		}

		// Add groundtruth for this room
		if (_groundTruth.find(event.curRoomId) != _groundTruth.end())
			groundTruthRows.push_back(_groundTruth[event.curRoomId]);
		else
			groundTruthRows.push_back(-1);

		// Add map for this room
		mapRows.push_back(mapIndex);

		// Draw shapes
		row = roomCatCount;
		mapIndex = -1;
		mapValue = -1;
		for(unsigned int i=0; i<shapeCountReal; ++i)
		{
			double prob = event.curShapes[i];
			if (prob>mapValue)
			{
				mapValue = prob;
				mapIndex = i;
			}
			scene->addRect(e*resultWidth, row*rowHeight, resultWidth, rowHeight,
					(verticalLines)?QPen():QPen(Qt::NoPen), getBrushForProbability(prob));
			row++;
		}
		// Add map
		mapShapeRows.push_back(mapIndex);


		// Draw sizes
		row = roomCatCount+ shapeCount;
		mapIndex = -1;
		mapValue = -1;
		for(unsigned int i=0; i<sizeCountReal; ++i)
		{
			double prob = event.curSizes[i];
			if (prob>mapValue)
			{
				mapValue = prob;
				mapIndex = i;
			}
			scene->addRect(e*resultWidth, row*rowHeight, resultWidth, rowHeight,
					(verticalLines)?QPen():QPen(Qt::NoPen), getBrushForProbability(prob));
			row++;
		}
		// Add map
		mapSizeRows.push_back(mapIndex);



		// Draw appearances
		row = roomCatCount + shapeCount + sizeCount;
		mapIndex = -1;
		mapValue = -1;
		for(unsigned int i=0; i<appearanceCountReal; ++i)
		{
			double prob = event.curAppearances[i];
			if (prob>mapValue)
			{
				mapValue = prob;
				mapIndex = i;
			}
			scene->addRect(e*resultWidth, row*rowHeight, resultWidth, rowHeight,
					(verticalLines)?QPen():QPen(Qt::NoPen), getBrushForProbability(prob));
			row++;
		}
		// Add map
		mapAppearanceRows.push_back(mapIndex);

		// Draw objects
		row = roomCatCount + shapeCount + appearanceCount + sizeCount;
		for(unsigned int i=0; i<objectsCountReal; ++i)
		{
			double prob = event.curObjects[i];
			scene->addRect(e*resultWidth, row*rowHeight, resultWidth, rowHeight,
					(verticalLines)?QPen():QPen(Qt::NoPen), getBrushForProbability(prob));
			row++;
		}

		if ((e%eventSeparatorCount) == 0)
		{
			scene->addLine(e*resultWidth, -eventSeparator ,e*resultWidth, 0, stdPen);
			QGraphicsSimpleTextItem *text = scene->addSimpleText(QString::number(e), defaultFont);
			text->setPos(e*resultWidth-text->boundingRect().width()/2, -eventSeparator-text->boundingRect().height());
			if (verticalIndicators)
				scene->addLine(e*resultWidth, 0 ,e*resultWidth, (rowCount+5)*rowHeight,
						QPen(QBrush(qRgb(150,150,150)),1, Qt::DotLine));
		}

		// Place/Room Events
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
				text->setPos((e*resultWidth+lastPlaceChange)/2-text->boundingRect().width()/2, (rowCount+4)*rowHeight);
				lastPlaceChange=e*resultWidth;
			}

			if (event.curRoomId!=curRoom)
			{
				text = scene->addSimpleText(QString::number(curRoom), defaultFont);
				text->setPos((e*resultWidth+lastRoomChange)/2-text->boundingRect().width()/2, (rowCount+5)*rowHeight);
				lastRoomChange=e*resultWidth;
				if (verticalIndicators)
					scene->addLine(e*resultWidth, 0 ,e*resultWidth, (rowCount+6)*rowHeight,
							QPen(QBrush(Qt::black), 1, Qt::DashLine));

				if (addingPlace)
				{
					scene->addLine(e*resultWidth, (rowCount+5.5)*rowHeight-markPlusSize/2.0, e*resultWidth,
							(rowCount+5.5)*rowHeight+markPlusSize/2.0,
							QPen(QBrush("black"), 2, Qt::SolidLine));
					scene->addLine(e*resultWidth-markPlusSize/2.0, (rowCount+5.5)*rowHeight, e*resultWidth+markPlusSize/2.0,
							(rowCount+5.5)*rowHeight,
							QPen(QBrush("black"), 2, Qt::SolidLine));
				}
				else
				{
					scene->addEllipse(e*resultWidth-markDotSize/2.0, (rowCount+5.5)*rowHeight-markDotSize/2.0, markDotSize, markDotSize,
							QPen(), QBrush(qRgb(0,0,0)));
				}
			}
			if (addingPlace)
			{
				scene->addLine(e*resultWidth, (rowCount+4.5)*rowHeight-markPlusSize/2.0, e*resultWidth,(rowCount+4.5)*rowHeight+markPlusSize/2.0,
						QPen(QBrush("black"), 2, Qt::SolidLine));
				scene->addLine(e*resultWidth-markPlusSize/2.0, (rowCount+4.5)*rowHeight, e*resultWidth+markPlusSize/2.0,(rowCount+4.5)*rowHeight,
						QPen(QBrush("black"), 2, Qt::SolidLine));
			}
			else
			{
				scene->addEllipse(e*resultWidth-markDotSize/2.0, (rowCount+4.5)*rowHeight-markDotSize/2.0, markDotSize, markDotSize,
						QPen(), QBrush(qRgb(0,0,0)));
			}
		}
		// Shape events
		bool shapeAdded = false;
		for (unsigned int i=0; i<accumulatedInfos.size(); ++i)
			if (accumulatedInfos[i].type==ConceptualData::EventShapePlacePropertyAdded)
				shapeAdded = true;
		if (shapeAdded)
		{
			scene->addLine(e*resultWidth, (rowCount+0.5)*rowHeight-markPlusSize/2.0, e*resultWidth,(rowCount+0.5)*rowHeight+markPlusSize/2.0,
					QPen(QBrush("black"), 2, Qt::SolidLine));
			scene->addLine(e*resultWidth-markPlusSize/2.0, (rowCount+0.5)*rowHeight, e*resultWidth+markPlusSize/2.0,(rowCount+0.5)*rowHeight,
					QPen(QBrush("black"), 2, Qt::SolidLine));
		}
		bool shapeChanged = false;
		for (unsigned int i=0; i<accumulatedInfos.size(); ++i)
			if (accumulatedInfos[i].type==ConceptualData::EventShapePlacePropertyChanged)
				shapeChanged = true;
		if (shapeChanged)
		{
			scene->addEllipse(e*resultWidth-markDotSize/2.0, (rowCount+0.5)*rowHeight-markDotSize/2.0, markDotSize, markDotSize,
					QPen(), QBrush(qRgb(0,0,0)));
		}
		// Size events
		bool sizeAdded = false;
		for (unsigned int i=0; i<accumulatedInfos.size(); ++i)
			if (accumulatedInfos[i].type==ConceptualData::EventSizePlacePropertyAdded)
				sizeAdded = true;
		if (sizeAdded)
		{
			scene->addLine(e*resultWidth, (rowCount+1.5)*rowHeight-markPlusSize/2.0, e*resultWidth,(rowCount+1.5)*rowHeight+markPlusSize/2.0,
					QPen(QBrush("black"), 2, Qt::SolidLine));
			scene->addLine(e*resultWidth-markPlusSize/2.0, (rowCount+1.5)*rowHeight, e*resultWidth+markPlusSize/2.0,(rowCount+1.5)*rowHeight,
					QPen(QBrush("black"), 2, Qt::SolidLine));
		}
		bool sizeChanged = false;
		for (unsigned int i=0; i<accumulatedInfos.size(); ++i)
			if (accumulatedInfos[i].type==ConceptualData::EventSizePlacePropertyChanged)
				sizeChanged = true;
		if (sizeChanged)
		{
			scene->addEllipse(e*resultWidth-markDotSize/2.0, (rowCount+1.5)*rowHeight-markDotSize/2.0, markDotSize, markDotSize,
					QPen(), QBrush(qRgb(0,0,0)));
		}
		// Appearance events
		bool appearanceAdded = false;
		for (unsigned int i=0; i<accumulatedInfos.size(); ++i)
			if (accumulatedInfos[i].type==ConceptualData::EventAppearancePlacePropertyAdded)
				appearanceAdded = true;
		if (appearanceAdded)
		{
			scene->addLine(e*resultWidth, (rowCount+2.5)*rowHeight-markPlusSize/2.0, e*resultWidth,(rowCount+2.5)*rowHeight+markPlusSize/2.0,
					QPen(QBrush("black"), 2, Qt::SolidLine));
			scene->addLine(e*resultWidth-markPlusSize/2.0, (rowCount+2.5)*rowHeight, e*resultWidth+markPlusSize/2.0,(rowCount+2.5)*rowHeight,
					QPen(QBrush("black"), 2, Qt::SolidLine));
		}
		bool appearanceChanged = false;
		for (unsigned int i=0; i<accumulatedInfos.size(); ++i)
			if (accumulatedInfos[i].type==ConceptualData::EventAppearancePlacePropertyChanged)
				appearanceChanged = true;
		if (appearanceChanged)
		{
			scene->addEllipse(e*resultWidth-markDotSize/2.0, (rowCount+2.5)*rowHeight-markDotSize/2.0, markDotSize, markDotSize,
					QPen(), QBrush(qRgb(0,0,0)));
		}
		// Object events
		bool objectAdded = false;
		for (unsigned int i=0; i<accumulatedInfos.size(); ++i)
			if (accumulatedInfos[i].type==ConceptualData::EventObjectSearchResultAdded)
				objectAdded = true;
		if (objectAdded)
		{
			scene->addLine(e*resultWidth, (rowCount+3.5)*rowHeight-markPlusSize/2.0, e*resultWidth,(rowCount+3.5)*rowHeight+markPlusSize/2.0,
					QPen(QBrush("black"), 2, Qt::SolidLine));
			scene->addLine(e*resultWidth-markPlusSize/2.0, (rowCount+3.5)*rowHeight, e*resultWidth+markPlusSize/2.0,(rowCount+3.5)*rowHeight,
					QPen(QBrush("black"), 2, Qt::SolidLine));
		}
		bool objectChanged = false;
		for (unsigned int i=0; i<accumulatedInfos.size(); ++i)
			if (accumulatedInfos[i].type==ConceptualData::EventObjectSearchResultChanged)
				objectChanged = true;
		if (objectChanged)
		{
			scene->addEllipse(e*resultWidth-markDotSize/2.0, (rowCount+3.5)*rowHeight-markDotSize/2.0, markDotSize, markDotSize,
					QPen(), QBrush(qRgb(0,0,0)));
		}

		curPlace=event.curPlaceId;
		curRoom=event.curRoomId;
		e++;
		accumulatedInfos.clear();
	}

	// Last place and room ids
	if (placeIds)
	{
		text = scene->addSimpleText(QString::number(curPlace), smallFont);
		text->setPos((e*resultWidth+lastPlaceChange)/2-text->boundingRect().width()/2, (rowCount+4)*rowHeight);
	}
	text = scene->addSimpleText(QString::number(curRoom), defaultFont);
	text->setPos((e*resultWidth+lastRoomChange)/2-text->boundingRect().width()/2, (rowCount+5)*rowHeight);

	// Horizontal lines
	for(unsigned int i=0; i<roomCatCount; ++i)
	{
		if (i==0)
			scene->addLine(0,i*rowHeight,resultWidth*e,i*rowHeight,
					QPen(QBrush("black"), horizSepWidth, Qt::SolidLine)); // Horizontal line
		else
			scene->addLine(0,i*rowHeight,resultWidth*e,i*rowHeight, stdPen); // Horizontal line
	}
	scene->addLine(0,roomCatCount*rowHeight,resultWidth*e,roomCatCount*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	for(unsigned int i=0; i<shapeCount; ++i)
		scene->addLine(0,(i+roomCatCount)*rowHeight,resultWidth*e,(i+roomCatCount)*rowHeight, stdPen); // Horizontal line
	scene->addLine(0,(roomCatCount+shapeCount)*rowHeight,resultWidth*e,(roomCatCount+shapeCount)*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	for(unsigned int i=0; i<sizeCount; ++i)
		scene->addLine(0,(i+roomCatCount+shapeCount)*rowHeight,resultWidth*e,(i+roomCatCount+shapeCount)*rowHeight, stdPen); // Horizontal line
	scene->addLine(0,(roomCatCount+shapeCount+sizeCount)*rowHeight,resultWidth*e,(roomCatCount+shapeCount+sizeCount)*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	for(unsigned int i=0; i<appearanceCount; ++i)
		scene->addLine(0,(i+roomCatCount+shapeCount+sizeCount)*rowHeight,
				resultWidth*e,(i+roomCatCount+shapeCount+sizeCount)*rowHeight, stdPen); // Horizontal line
	scene->addLine(0,(roomCatCount+shapeCount+sizeCount+appearanceCount)*rowHeight,resultWidth*e,
			(roomCatCount+shapeCount+sizeCount+appearanceCount)*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	for(unsigned int i=0; i<objectCount; ++i)
		scene->addLine(0,(i+roomCatCount+shapeCount+sizeCount+appearanceCount)*rowHeight,
				resultWidth*e,(i+roomCatCount+shapeCount+sizeCount+appearanceCount)*rowHeight, stdPen); // Horizontal line
	// Events
	scene->addLine(0,rowCount*rowHeight,resultWidth*e,rowCount*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	scene->addLine(0,(rowCount+1)*rowHeight,resultWidth*e,(rowCount+1)*rowHeight, stdPen);
	scene->addLine(0,(rowCount+2)*rowHeight,resultWidth*e,(rowCount+2)*rowHeight, stdPen);
	scene->addLine(0,(rowCount+3)*rowHeight,resultWidth*e,(rowCount+3)*rowHeight, stdPen);
	scene->addLine(0,(rowCount+4)*rowHeight,resultWidth*e,(rowCount+4)*rowHeight, stdPen);
	scene->addLine(0,(rowCount+5)*rowHeight,resultWidth*e,(rowCount+5)*rowHeight, stdPen);
	scene->addLine(0,(rowCount+6)*rowHeight,resultWidth*e,(rowCount+6)*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));

	// Starting and ending vertical line
	scene->addLine(0,-eventSeparator,0,rowHeight*(rowCount+6), QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	scene->addLine((e)*resultWidth,-eventSeparator,(e)*resultWidth,rowHeight*(rowCount+6),
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));

	// Groundtruth
	int lastG = -1;
	int lastI = 0;
	int lastLastI = 0;
	for (unsigned int i=0; i<groundTruthRows.size(); ++i)
	{
		if (( lastG != groundTruthRows[i] ) || (i==groundTruthRows.size()-1))
		{
			if (lastG>=0)
			{
				if (i==groundTruthRows.size()-1)
					i++;
				scene->addRect(lastLastI*resultWidth, lastG*rowHeight, (i-lastLastI)*resultWidth, rowHeight, gtPenBg);
				scene->addRect(lastLastI*resultWidth, lastG*rowHeight, (i-lastLastI)*resultWidth, rowHeight, gtPen);
			}
			lastLastI=i;
		}
		lastG = groundTruthRows[i];
		lastI = i;
	}

	// MAP
	for (unsigned int i=0; i<mapRows.size(); ++i)
	{
		scene->addEllipse(i*resultWidth+0.5*(resultWidth-5), mapRows[i]*rowHeight+0.5*(rowHeight-5), 5, 5, QPen(Qt::black, 1), QBrush(Qt::white));
	}
	for (unsigned int i=0; i<mapShapeRows.size(); ++i)
	{
		scene->addEllipse(i*resultWidth+0.5*(resultWidth-5), (roomCatCount+mapShapeRows[i])*rowHeight+0.5*(rowHeight-5), 5, 5, QPen(Qt::black, 1), QBrush(Qt::white));
	}
	for (unsigned int i=0; i<mapSizeRows.size(); ++i)
	{
		scene->addEllipse(i*resultWidth+0.5*(resultWidth-5), (roomCatCount+shapeCount+mapSizeRows[i])*rowHeight+0.5*(rowHeight-5), 5, 5, QPen(Qt::black, 1), QBrush(Qt::white));
	}
	for (unsigned int i=0; i<mapAppearanceRows.size(); ++i)
	{
		scene->addEllipse(i*resultWidth+0.5*(resultWidth-5), (roomCatCount+shapeCount+sizeCount+mapAppearanceRows[i])*rowHeight+0.5*(rowHeight-5), 5, 5, QPen(Qt::black, 1), QBrush(Qt::white));
	}

	return e;
}


// ------------------------------------------------------
void EventView::drawLegend(QGraphicsScene *scene)
{
	scene->setBackgroundBrush(Qt::white);

	unsigned int objectCount = _visualizedObjects->size();
	unsigned int roomCatCount = _roomCats->size();
	unsigned int shapeCount = _shapes->size();
	unsigned int sizeCount = _sizes->size();
	unsigned int appearanceCount = _appearances->size();
	unsigned int rowCount = roomCatCount+shapeCount+sizeCount+appearanceCount+objectCount;
	QGraphicsSimpleTextItem *text;

	// Start
	for(unsigned int i=0; i<roomCatCount; ++i)
	{
		if (i==0)
			scene->addLine(-horizSeparator,i*rowHeight, 0, i*rowHeight,
					QPen(QBrush("black"), horizSepWidth, Qt::SolidLine)); // Horizontal line
		else
			scene->addLine(-categorySeparator,i*rowHeight, 0,i*rowHeight, stdPen); // Horizontal line
		text = scene->addSimpleText(QString::fromStdString((*_roomCats)[i]), defaultFont);
		text->setPos(-text->boundingRect().width()-categoriesDist, i*rowHeight);
	}
	scene->addLine(-horizSeparator,roomCatCount*rowHeight, 0,roomCatCount*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	for(unsigned int i=0; i<shapeCount; ++i)
	{
		scene->addLine(-categorySeparator,(i+roomCatCount)*rowHeight,0,(i+roomCatCount)*rowHeight, stdPen); // Horizontal line
		text = scene->addSimpleText(QString::fromStdString((*_shapes)[i]), defaultFont);
		text->setPos(-text->boundingRect().width()-categoriesDist, (i+roomCatCount)*rowHeight);
	}

	scene->addLine(-horizSeparator,(roomCatCount+shapeCount)*rowHeight,0,(roomCatCount+shapeCount)*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	for(unsigned int i=0; i<sizeCount; ++i)
	{
		scene->addLine(-categorySeparator,(i+roomCatCount+shapeCount)*rowHeight,0,(i+roomCatCount+shapeCount)*rowHeight, stdPen); // Horizontal line
		text = scene->addSimpleText(QString::fromStdString((*_sizes)[i]), defaultFont);
		text->setPos(-text->boundingRect().width()-categoriesDist, (i+roomCatCount+shapeCount)*rowHeight);
	}

	scene->addLine(-horizSeparator,(roomCatCount+shapeCount+sizeCount)*rowHeight,0,(roomCatCount+shapeCount+sizeCount)*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	for(unsigned int i=0; i<appearanceCount; ++i)
	{
		scene->addLine(-categorySeparator,(i+roomCatCount+shapeCount+sizeCount)*rowHeight,
				0,(i+roomCatCount+shapeCount+sizeCount)*rowHeight, stdPen); // Horizontal line
		text = scene->addSimpleText(QString::fromStdString((*_appearances)[i]), defaultFont);
		text->setPos(-text->boundingRect().width()-categoriesDist, (i+roomCatCount+shapeCount+sizeCount)*rowHeight);
	}
	scene->addLine(-horizSeparator,(roomCatCount+shapeCount+sizeCount+appearanceCount)*rowHeight, 0,
			(roomCatCount+shapeCount+sizeCount+appearanceCount)*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	for(unsigned int i=0; i<objectCount; ++i)
	{
		scene->addLine(-categorySeparator,(i+roomCatCount+shapeCount+sizeCount+appearanceCount)*rowHeight,
				0,(i+roomCatCount+shapeCount+sizeCount+appearanceCount)*rowHeight, stdPen); // Horizontal line
		text = scene->addSimpleText(QString::fromStdString((*_visualizedObjects)[i]), defaultFont);
		text->setPos(-text->boundingRect().width()-categoriesDist, (i+roomCatCount+shapeCount+sizeCount+appearanceCount)*rowHeight);
	}
	// Events
	scene->addLine(-horizSeparator,rowCount*rowHeight,0,rowCount*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
	text = scene->addSimpleText("shape", defaultFont);
	text->setPos(-text->boundingRect().width()-categoriesDist, rowCount*rowHeight);
	scene->addLine(-categorySeparator,(rowCount+1)*rowHeight, 0,(rowCount+1)*rowHeight, stdPen);
	text = scene->addSimpleText("size", defaultFont);
	text->setPos(-text->boundingRect().width()-categoriesDist, (rowCount+1)*rowHeight);
	scene->addLine(-categorySeparator,(rowCount+2)*rowHeight, 0,(rowCount+2)*rowHeight, stdPen);
	text = scene->addSimpleText("appearance", defaultFont);
	text->setPos(-text->boundingRect().width()-categoriesDist, (rowCount+2)*rowHeight);
	scene->addLine(-categorySeparator,(rowCount+3)*rowHeight, 0,(rowCount+3)*rowHeight, stdPen);
	text = scene->addSimpleText("object", defaultFont);
	text->setPos(-text->boundingRect().width()-categoriesDist, (rowCount+3)*rowHeight);
	scene->addLine(-categorySeparator,(rowCount+4)*rowHeight, 0,(rowCount+4)*rowHeight, stdPen);
	text = scene->addSimpleText("place", defaultFont);
	text->setPos(-text->boundingRect().width()-categoriesDist, (rowCount+4)*rowHeight);
	scene->addLine(-categorySeparator,(rowCount+5)*rowHeight, 0,(rowCount+5)*rowHeight, stdPen);
	text = scene->addSimpleText("room", defaultFont);
	text->setPos(-text->boundingRect().width()-categoriesDist, (rowCount+5)*rowHeight);
	scene->addLine(-horizSeparator,(rowCount+6)*rowHeight, 0,(rowCount+6)*rowHeight,
			QPen(QBrush("black"), horizSepWidth, Qt::SolidLine));
}


// ------------------------------------------------------
QBrush EventView::getBrushForProbability(double prob)
{
	prob=1-prob;
	double step1 = 0.5;
	double step2 = 0.75;

	double r = (prob<=step1)? prob/step1: 1.0;
	double g = ((prob>step1) && (prob<=step2))? (prob-step1)/(step2-step1): ((prob>step1)?1.0:0.0);
	double b = (prob>step2)? ((prob-step2)/(1.0-step2)): 0.0;

	return QBrush (qRgb(r*255.0, g*255.0, b*255.0));
}


// ------------------------------------------------------
void EventView::addGroundTruth(int roomId, int categoryIndex)
{
	_groundTruth[roomId] = categoryIndex;

}

