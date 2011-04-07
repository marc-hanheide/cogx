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


public slots:

	void update(const QList<conceptual::ConceptualEvent> &events, bool placeIds,
		int resultWidth, bool verticalIndicators, bool verticalLines, bool locationOnly);



private:

	QBrush getBrushForProbability(double prob);


private:

    /** Map roomId -> roomCateogory index. */
    std::map<int, int> _groundTruth;
	int _curPlaceId;
	int _curRoomId;
	std::set<int> _roomIds;

	const std::vector<std::string> *_roomCats;
	const std::vector<std::string> *_shapes;
	const std::vector<std::string> *_sizes;
	const std::vector<std::string> *_appearances;
	const std::vector<std::string> *_visualizedObjects;

};

#endif // EVENTVIEW_H


