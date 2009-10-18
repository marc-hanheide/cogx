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
 * \file CSystem.cpp
 * \author Andrzej Pronobis
 *
 * Contains implementation of the CSystem class.
 */

#include <QtCore/QStringList>
#include <QtCore/QList>
#include "global.h"
#include "CChannelCache.h"
#include "CFilterCache.h"
#include "CScaleSpaceCache.h"
#include "CCrfh.h"
#include "CMatrix.h"

#include "CSystem.h"


// -----------------------------------------
CSystem::CSystem(QString sysDef)
{
  // Exemplar string:
  // Lxx(8,28)+Lxy(8,28)+Lyy(8,28)+Lxx(2,28)+Lxy(2,28)+Lyy(2,28)

  // Extract tokens
  QStringList tokens = sysDef.split('+');

  // Create descriptors
  for (int i=0; i<tokens.size(); ++i)
  {
    // Decode params
    int bracket1Pos = tokens[i].indexOf('(');
    int comaPos = tokens[i].indexOf(',');
    int bracket2Pos = tokens[i].indexOf(')');
    QString name = tokens[i].left(bracket1Pos);
    double scale = tokens[i].mid(bracket1Pos+1, comaPos-bracket1Pos-1).toDouble();
    int bins = tokens[i].mid(comaPos+1, bracket2Pos-comaPos-1).toInt();

    // Create descriptor
    _descriptorList.addDescriptor(name, scale, bins);
  }

  // Create filters
  _descriptorList.createAllRequiredFilters(_filterCache);

}


// -----------------------------------------
QList<CMatrix *> CSystem::computeDescriptorOutputs(const CImage &image) const
{
  // Create channel cache
  CChannelCache channelCache(image);
  _descriptorList.createAllRequiredChannels(channelCache);

  // Create scale-space cache
  CScaleSpaceCache scaleSpaceCache(channelCache, _filterCache);
  _descriptorList.createAllRequiredScales(scaleSpaceCache);

  // Apply the descriptors
  return _descriptorList.applyAll(channelCache, scaleSpaceCache, _filterCache);
}


// -----------------------------------------
CCrfh *CSystem::computeHistogram(const CImage &image, int skipBorderPixels) const
{
  QList<CMatrix *> outputs = computeDescriptorOutputs(image);
  CCrfh *crfh = new CCrfh(outputs, _descriptorList, skipBorderPixels);
  for(int i=0; i<outputs.size(); ++i)
    delete outputs[i];
  return crfh;
}

