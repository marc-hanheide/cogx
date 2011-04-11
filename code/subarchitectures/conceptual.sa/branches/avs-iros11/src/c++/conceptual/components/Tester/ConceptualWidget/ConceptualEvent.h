/*
 * ConceptualEvent.h
 *
 *  Created on: 13-03-2011
 *      Author: Andrzej Pronobis
 */

#ifndef CONCEPTUALEVENT_H_
#define CONCEPTUALEVENT_H_

#include <QList>
#include <ConceptualData.hpp>

class QDataStream;

namespace ConceptualData
{
	QDataStream &operator<<(QDataStream &, const EventInfo &);
	QDataStream &operator>>(QDataStream &, EventInfo &);
}

namespace std
{
	QDataStream &operator<<(QDataStream &, const vector<string> &);
	QDataStream &operator>>(QDataStream &, vector<string> &);
}


namespace conceptual
{

class ConceptualEvent
{
public:
	ConceptualEvent():
		curRoomId(-1), curPlaceId(-1)
	{}

	int curRoomId;
	int curPlaceId;
	QList<int> curRoomPlaces;
	QList<ConceptualData::EventInfo> infos;
	QList<double> curRoomCategories;
	QList<double> curShapes;
	QList<double> curSizes;
	QList<double> curAppearances;
	QList<double> curObjects;
	double time;

public:

	void setInfos(std::vector<ConceptualData::EventInfo> infos);


};


QDataStream &operator<<(QDataStream &, const ConceptualEvent &);
QDataStream &operator>>(QDataStream &, ConceptualEvent &);



}

#endif /* CONCEPTUALEVENT_H_ */
