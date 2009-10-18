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
 * \file CDescriptorList.h
 * \author Andrzej Pronobis
 *
 * Contains declaration of the CDescriptorList class.
 */

#ifndef _CDESCRIPTORLIST_H_
#define _CDESCRIPTORLIST_H_

#include <QtCore/QString>
#include <QtCore/QList>

#include "CDescriptor.h"

class CMatrix;


class CDescriptorList : public QList<CDescriptor *>
{

public:

  /** Destructor. Deletes all the descriptors. */
  ~CDescriptorList();


public:

  /** Adds a new descriptor to the list. The descriptor is characterized
      by its type, scale, and number of bins. */
  void addDescriptor(DescriptorType descriptorType, double scale, int bins);

  /** Adds a new descriptor to the list. The descriptors is characterized
      by its name, scale, and number of bins. */
  void addDescriptor(QString name, double scale, int bins);


public:

  /** Creates filters requierd by all descriptors. */
  void createAllRequiredFilters(CFilterCache &filterCache) const;

  /** Creates samples of the scale-space requierd by all descriptors. */
  void createAllRequiredScales(CScaleSpaceCache &scaleSpaceCache) const;

  /** Creates channels required by all descriptors. */
  void createAllRequiredChannels(CChannelCache &channelCache) const;

  /** Applies all the descriptors in the list and returns a list of
      pointers to the output matrices. */
  QList<CMatrix *> applyAll(const CChannelCache &channelCache,
                            const CScaleSpaceCache &scaleSpaceCache,
                            const CFilterCache &filterCache) const;

};


#endif

