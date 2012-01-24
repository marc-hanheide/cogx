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
 * \file NodePlaceInfoPullSender.h
 * \author Andrzej Pronobis
 * \date 2008-09-04
 */


#ifndef __PLACE_NODE_PLACE_INFO_PULL_SENDER__
#define __PLACE_NODE_PLACE_INFO_PULL_SENDER__

// Place.SA
#include <PlaceData.hpp>
// CAST/Balt
//#include <balt/core/Interfaces.hpp>

namespace categorical
{

class NodePlaceInfoPullSender : public PullSender<PlaceData::NodePlaceInfo>
{

public:

  NodePlaceInfoPullSender()
    { _nodePlaceInfoPuller = 0; }

  virtual void setPullConnector(const std::string&, PullConnectorOut<PlaceData::NodePlaceInfo> *out)
    { _nodePlaceInfoPuller = out; }

  bool isConnectedToNodeLabeller()
    { return (_nodePlaceInfoPuller!=0); }

  PlaceData::NodePlaceInfo pullNodePlaceInfo(int nodeId, 
                                             PlaceData::NodeLabellerQueryType queryType, 
                                             bool gateway);


private:

  /** Pull connector to NodeLabeller. */
  PullConnectorOut<PlaceData::NodePlaceInfo> *_nodePlaceInfoPuller;

};

}

#endif
