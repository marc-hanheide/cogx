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
* \file CChannelCache.h
* \author Andrzej Pronobis
*
* Contains declaration of the CChannelCache class.
*/

#ifndef _CCHANNELCACHE_H_
#define _CCHANNELCACHE_H_

#include <QtCore/QList>

class CMatrix;
class CImage;

enum ChannelType
{
  CT_UNKNOWN = 0,
  CT_L, CT_C1, CT_C2
};

/**
* Class storing a cache of channels.
*/
class CChannelCache
{

public:

  /** Default constructor. */
  inline CChannelCache(const CImage &image) : _image(&image) {};

  /** Destructor. Deletes all the channels. */
  ~CChannelCache();


public:

  /** Creates a new channel on the basis of the input image.
      If an identical channel already exists a new one will
      not be created. */
  void createChannel(ChannelType channelType);

  /** Returns a pointer to a matrix containing pixels of the channel. */
  const CMatrix *getChannel(ChannelType channelType) const;


private:

  /** Pointer to the input image. */
  const CImage *_image;

  /** List storing pointers to channels. */
  QList<CMatrix *> _channelList;

  /** List of types of channels in the _channelList. */
  QList<ChannelType> _channelTypeList;
};



#endif
