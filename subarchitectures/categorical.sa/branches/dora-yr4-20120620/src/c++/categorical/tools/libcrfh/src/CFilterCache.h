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
* \file CFilterCache.h
* \author Andrzej Pronobis
*
* Contains declaration of the CFilterCache class.
*/

#ifndef _CFILTERCACHE_H_
#define _CFILTERCACHE_H_

#include <QtCore/QList>

class CFilter;
class CFilterInfo;
class CMatrix;


/**
* Class storing a cache of filters.
*/
class CFilterCache
{

public:

  /** Default constructor. */
  inline CFilterCache() {};

  /** Destructor. Deletes all the filters. */
  ~CFilterCache();

public:

  /** Creates a new filter. If an identical filter already exists
      a new one will not be created. */
  bool createFilter(const CFilterInfo &filterInfo);

  /** Applies a filter identified by the filterInfo to the given matrix. */
  CMatrix *applyFilter(const CFilterInfo &filterInfo,
                       const CMatrix &input, CMatrix *result = 0) const;

private:

  /** List storing pointers to filters. */
  QList<CFilter *> _filterList;

};


#endif

