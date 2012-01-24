// ==================================================================
// libCRFH
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of libCRFH.
//
// libCRFH is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// libCRFH is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with libCRFH. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================

/**
* \file CScaleSpaceCache.cpp
* \author Andrzej Pronobis
*
* Contains implementation of the CScaleSpaceCache class.
*/

#include "CMatrix.h"
#include "CChannelCache.h"
#include "CFilterCache.h"
#include "CFilter.h"

#include "CScaleSpaceCache.h"


// -----------------------------------------
CScaleSpaceCache::~CScaleSpaceCache()
{
  for (int i=0; i<_scaleSpaceSamplesList.size(); ++i)
    delete _scaleSpaceSamplesList[i].matrix;
}


// -----------------------------------------
void CScaleSpaceCache::createScaleSpaceSample(ChannelType channelType, double scale)
{
  // Check if we don't have the sample in the cache
  for (int i=0; i<_scaleSpaceSamplesList.size(); ++i)
    if ((_scaleSpaceSamplesList[i].channelType == channelType) &&
        (_scaleSpaceSamplesList[i].scale == scale))
      return;

  // Create a new sample
  CScaleSpaceSampleInfo sssi;
  sssi.channelType = channelType;
  sssi.scale = scale;

  CGaussianFilterInfo gfi(scale);
  sssi.matrix = _filterCache->applyFilter( gfi, *(_channelCache->getChannel(channelType)) );

  // Append sample to the list
  _scaleSpaceSamplesList.append(sssi);
}



// -----------------------------------------
const CMatrix *CScaleSpaceCache::getScaleSpaceSample(ChannelType channelType, double scale) const
{
  for (int i=0; i<_scaleSpaceSamplesList.size(); ++i)
    if ((_scaleSpaceSamplesList[i].channelType == channelType) &&
        (_scaleSpaceSamplesList[i].scale == scale))
      return _scaleSpaceSamplesList[i].matrix;

  return 0;
}
