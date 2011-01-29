// ==================================================================
// Place.SA - Place Classification Subarchitecture
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of Place.SA.
//
// Place.SA is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Place.SA is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Place.SA. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================

/**
 * NodePlaceInfoPullSender class.
 * \file NodePlaceInfoPullSender.cpp
 * \author Andrzej Pronobis
 * \date 2008-09-04
 */


#include "place/shared/NodePlaceInfoPullSender.h"
#include <sstream>

using namespace std;
using namespace categorical;

// ------------------------------------------------------
PlaceData::NodePlaceInfo NodePlaceInfoPullSender::pullNodePlaceInfo(int nodeId,
                                                                    PlaceData::NodeLabellerQueryType queryType,
                                                                    bool gateway)
{
  PlaceData::NodePlaceInfo nodePlaceInfo;
  nodePlaceInfo.nodeId=-1;

  // Safety check
  if (!isConnectedToNodeLabeller())
    return nodePlaceInfo;

  // Generate query
  stringstream s;
  s<<"node_id=";
  s<<nodeId;
  s<<" query_type=";
  s<<queryType;
  s<<" gateway=";
  s<<((gateway)?1:0);
  FrameworkQuery query("",s.str());

  // Pull
  FrameworkLocalData<PlaceData::NodePlaceInfo> *localData = 0;
  _nodePlaceInfoPuller->pull(query, localData);
  nodePlaceInfo = *(localData->data());
  delete localData;

  return nodePlaceInfo;
}

