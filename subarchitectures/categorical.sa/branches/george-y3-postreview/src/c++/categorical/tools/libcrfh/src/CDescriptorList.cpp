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
* \file CDescriptorList.cpp
* \author Andrzej Pronobis
*
* Contains implementation of the CDescriptorList class.
*/

#include "CMatrix.h"

#include "CDescriptorList.h"


// -----------------------------------------
CDescriptorList::~CDescriptorList()
{
  for (int i=0; i<size(); ++i)
    delete at(i);
}


// -----------------------------------------
void CDescriptorList::addDescriptor(DescriptorType descriptorType, double scale, int bins)
{
  append(CDescriptor::createDescriptor(descriptorType, scale, bins));
}


// -----------------------------------------
void CDescriptorList::addDescriptor(QString name, double scale, int bins)
{
  append(CDescriptor::createDescriptor(name, scale, bins));
}


// -----------------------------------------
void CDescriptorList::createAllRequiredFilters(CFilterCache &filterCache) const
{
  for (int i=0; i<size(); ++i)
    at(i)->createRequiredFilters(filterCache);
}


// -----------------------------------------
void CDescriptorList::createAllRequiredScales(CScaleSpaceCache &scaleSpaceCache) const
{
  for (int i=0; i<size(); ++i)
    at(i)->createRequiredScales(scaleSpaceCache);
}


// -----------------------------------------
void CDescriptorList::createAllRequiredChannels(CChannelCache &channelCache) const
{
  for (int i=0; i<size(); ++i)
    at(i)->createRequiredChannels(channelCache);
}


// -----------------------------------------
QList<CMatrix *> CDescriptorList::applyAll(const CChannelCache &channelCache,
                                           const CScaleSpaceCache &scaleSpaceCache,
                                           const CFilterCache &filterCache) const
{
  QList<CMatrix *> list;

  for (int i=0; i<size(); ++i)
    list.append(at(i)->apply(channelCache, scaleSpaceCache, filterCache));

  return list;
}
