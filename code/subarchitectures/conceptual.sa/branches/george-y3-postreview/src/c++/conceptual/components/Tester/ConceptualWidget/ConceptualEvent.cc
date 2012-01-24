/*
 * ConceptualEvent.cpp
 *
 *  Created on: 13-03-2011
 *      Author: czikus
 */

#include "ConceptualEvent.h"

#include <QDataStream>


namespace ConceptualData
{

QDataStream &operator<<(QDataStream &stream, const ConceptualData::EventInfo &eventInfo)
{
	stream << eventInfo.type;
	stream << eventInfo.time;
	stream << eventInfo.roomId;
	stream << eventInfo.place1Id;
	stream << eventInfo.place2Id;
	stream << QString::fromStdString(eventInfo.propertyInfo);

	return stream;
}


QDataStream &operator>>(QDataStream &stream, ConceptualData::EventInfo &eventInfo)
{
	int i;
	stream >> i;
	eventInfo.type = static_cast<ConceptualData::EventType>(i);
	stream >> eventInfo.time;
	stream >> eventInfo.roomId;
	stream >> eventInfo.place1Id;
	stream >> eventInfo.place2Id;
	QString str;
	stream >> str;
	eventInfo.propertyInfo = str.toStdString();

	return stream;
}


}



namespace std
{
	QDataStream &operator<<(QDataStream &stream, const vector<string> &vect)
	{
		stream << static_cast<int>(vect.size());
		for (unsigned int i=0; i<vect.size(); ++i)
		{
			stream << QString::fromStdString(vect[i]);
		}

		return stream;
	}

	QDataStream &operator>>(QDataStream &stream, vector<string> &vect)
	{
		unsigned int size;
		stream >> size;
		vect.resize(size);
		for (unsigned int i=0; i<size; ++i)
		{
			QString str;
			stream >> str;
			vect[i] = str.toStdString();
		}

		return stream;
	}
}



namespace conceptual
{


void ConceptualEvent::setInfos(std::vector<ConceptualData::EventInfo> infos)
{
	for (unsigned int i=0; i<infos.size(); ++i)
	{
		this->infos.append(infos[i]);
	}
}


QDataStream &operator<<(QDataStream &stream, const ConceptualEvent &event)
{
	stream << event.time;
	stream << event.curRoomId;
	stream << event.curPlaceId;
	stream << event.curRoomPlaces;
	stream << event.infos;
	stream << event.curRoomCategories;
	stream << event.curShapes;
	stream << event.curSizes;
	stream << event.curAppearances;
	stream << event.curObjects;

	return stream;
}


QDataStream &operator>>(QDataStream &stream, ConceptualEvent &event)
{
	stream >> event.time;
	stream >> event.curRoomId;
	stream >> event.curPlaceId;
	stream >> event.curRoomPlaces;
	stream >> event.infos;
	stream >> event.curRoomCategories;
	stream >> event.curShapes;
	stream >> event.curSizes;
	stream >> event.curAppearances;
	stream >> event.curObjects;

	return stream;
}


}
