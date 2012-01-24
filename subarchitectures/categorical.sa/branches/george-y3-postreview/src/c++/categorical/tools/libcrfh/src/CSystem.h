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
 * \file CSystem.h
 * \author Andrzej Pronobis
 *
 * Contains declaration of the CSystem class.
 */

#ifndef _CSYSTEM_H_
#define _CSYSTEM_H_

#include <QtCore/QList>
#include "CFilterCache.h"
#include "CDescriptorList.h"

class CImage;
class CCrfh;

/**
* Main class defining a system and managing the
* histogram extraction process.
*/
class CSystem
{

public:

  /** Constructor. Initializes the system (creates descriptors
      and filters). */
  CSystem(QString sysDef);


public:

  /** Computes outputs of all the descriptors. */
  QList<CMatrix *> computeDescriptorOutputs(const CImage &image) const;

  /** Computes the histogram for a given image. */
  CCrfh *computeHistogram(const CImage &image, int skipBorderPixels) const;


private:

  /** List of descriptors to be computed. */
  CDescriptorList _descriptorList;

  /** Filter cache. */
  CFilterCache _filterCache;
};



#endif

