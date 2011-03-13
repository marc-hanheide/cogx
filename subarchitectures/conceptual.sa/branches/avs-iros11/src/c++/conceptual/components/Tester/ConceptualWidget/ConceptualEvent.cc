/*
 * ConceptualEvent.cpp
 *
 *  Created on: 13-03-2011
 *      Author: czikus
 */

#include "ConceptualEvent.h"

namespace conceptual
{


void ConceptualEvent::setInfos(std::vector<ConceptualData::EventInfo> infos)
{
	for (unsigned int i=0; i<infos.size(); ++i)
	{
		this->infos.append(infos[i]);
	}
}



}
