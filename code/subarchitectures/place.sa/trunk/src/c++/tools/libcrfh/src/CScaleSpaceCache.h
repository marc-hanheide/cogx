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
 * \file CScaleSpaceCache.h
 * \author Andrzej Pronobis
 *
 * Contains declaration of the CScaleSpaceCache class.
 */

#ifndef _CSCALESPACECACHE_H_
#define _CSCALESPACECACHE_H_

#include "CChannelCache.h"

class CFilterCache;

/**
* Struct storing information about a scale-space sample.
*/
struct CScaleSpaceSampleInfo
{
  ChannelType channelType;
  double scale;
  CMatrix *matrix;
};


/**
* Cache storing samples of the scale-space.
*/
class CScaleSpaceCache
{

public:

  /** Default constructor. */
  inline CScaleSpaceCache(const CChannelCache &channelCache,
                          const CFilterCache &filterCache) :
    _channelCache(&channelCache), _filterCache(&filterCache) {};

  /** Destructor. Deletes all the scale-space samples. */
  ~CScaleSpaceCache();


public:

  /** Creates a sample of the scale-space obtained from a given
      channel. If an identical sample already exists a new one will
      not be created. */
  void createScaleSpaceSample(ChannelType channelType, double scale);

  /** Returns a pointer to a matrix containing pixels of the scale-space sample. */
  const CMatrix *getScaleSpaceSample(ChannelType channelType, double scale) const;


private:

  /** Pointer to the channel cache. */
  const CChannelCache *_channelCache;

  /** Pointer to the filter cache. */
  const CFilterCache *_filterCache;

  /** List storing information about samples. */
  QList<CScaleSpaceSampleInfo> _scaleSpaceSamplesList;


};



#endif

