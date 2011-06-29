#ifndef EVENTVIEW_H
#define EVENTVIEW_H

#include "ConceptualEvent.h"
#include <QtGui/QGraphicsView>
#include <vector>
#include <string>
#include <set>

class EventView : public QGraphicsView
{
    Q_OBJECT

public:
    EventView(QWidget *parent);
    ~EventView();

    void setDisplayedEntities(const std::vector<std::string> &roomCats, const std::vector<std::string> &shapes,
    		const std::vector<std::string> &sizes, const std::vector<std::string> &appearances,
    		const std::vector<std::string> &visualizedObjects)
    {
    	_roomCats=&roomCats;
		_shapes=&shapes;
		_sizes=&sizes;
		_appearances=&appearances;
		_visualizedObjects=&visualizedObjects;
    }

    void addGroundTruth(int roomId, int categoryIndex);

    void drawLegend(QGraphicsScene *scene);
    int drawEvents(QGraphicsScene *scene, const QList<conceptual::ConceptualEvent> &events, bool placeIds,
    		int resultWidth, bool verticalIndicators, bool verticalLines, bool locationOnly, double time);


public:

	void updateAll(const QList<conceptual::ConceptualEvent> &events, bool placeIds,
		int resultWidth, bool verticalIndicators, bool verticalLines, bool locationOnly);
	void updateEvents(const QList<conceptual::ConceptualEvent> &events, bool placeIds,
		int resultWidth, bool verticalIndicators, bool verticalLines, bool locationOnly, double time);
	void updateLegend();

	void fit();


protected:

	virtual void resizeEvent ( QResizeEvent * event )
	{
		QGraphicsView::resizeEvent(event);
		fit();
	}


private:

	QBrush getBrushForProbability(double prob);


private:

    /** Map roomId -> roomCateogory index. */
    std::map<int, int> _groundTruth;

	const std::vector<std::string> *_roomCats;
	const std::vector<std::string> *_shapes;
	const std::vector<std::string> *_sizes;
	const std::vector<std::string> *_appearances;
	const std::vector<std::string> *_visualizedObjects;

	bool _lastUpdatedEvents;
	int _lastE;
	int _lastResultWidth;


private:

	// Settings
	int categoriesDist;
	int rowHeight;
	int categorySeparator;
	int eventSeparator;
	int eventSeparatorCount;
	int horizSeparator;
	int horizSepWidth;
	double markDotSize;
	double markPlusSize;
	QFont defaultFont;
	QFont smallFont;
	QPen stdPen;
	QPen gtPen;
	QPen gtPenBg;
	bool _drawLegend;
	bool _drawEvents;

};

#endif // EVENTVIEW_H


