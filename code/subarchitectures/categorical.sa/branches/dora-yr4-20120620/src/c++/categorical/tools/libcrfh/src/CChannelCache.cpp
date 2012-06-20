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
* \file CChannelCache.cpp
* \author Andrzej Pronobis
*
* Contains implementation of the CChannelCache class.
*/

#include "CMatrix.h"
#include "CImage.h"

#include "CChannelCache.h"


// -----------------------------------------
CChannelCache::~CChannelCache()
{
  for (int i=0; i<_channelList.size(); ++i)
    delete _channelList[i];
}


// -----------------------------------------
void CChannelCache::createChannel(ChannelType channelType)
{
  // Check if we don't have the channel in the cache
  for (int i=0; i<_channelTypeList.size(); ++i)
    if (_channelTypeList[i] == channelType)
      return;

  // Create a new channel
  CMatrix *channel = _image->getL();

  if (channel)
  {
    _channelList.append(channel);
    _channelTypeList.append(channelType);
  }
}


// -----------------------------------------
const CMatrix *CChannelCache::getChannel(ChannelType channelType) const
{
  // Find the channel
  for (int i=0; i<_channelTypeList.size(); ++i)
    if (_channelTypeList[i] == channelType)
      return _channelList[i];

  aout<<"ERROR: No such channel!"<<endl;
  return 0;
}

