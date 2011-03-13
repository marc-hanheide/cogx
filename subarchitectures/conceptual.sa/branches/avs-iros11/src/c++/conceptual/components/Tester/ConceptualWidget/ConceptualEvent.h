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
	QList<double> curAppearances;
	QList<double> curObjects;

public:

	void setInfos(std::vector<ConceptualData::EventInfo> infos);


};

}

#endif /* CONCEPTUALEVENT_H_ */
