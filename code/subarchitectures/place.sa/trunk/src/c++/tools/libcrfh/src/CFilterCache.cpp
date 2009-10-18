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
* \file CFilterCache.cpp
* \author Andrzej Pronobis
*
* Contains implementation of the CFilterCache class.
*/

#include "global.h"
#include "CFilter.h"


#include "CFilterCache.h"



// -----------------------------------------
CFilterCache::~CFilterCache()
{
  for (int i=0; i<_filterList.size(); ++i)
    delete _filterList[i];
}


// -----------------------------------------
bool CFilterCache::createFilter(const CFilterInfo &filterInfo)
{
  // Check if we don't have an identical filter in the cache
  for (int i=0; i<_filterList.size(); ++i)
    if (_filterList[i]->getFilterInfo() == filterInfo)
      return true;

  // Create a new filter
  CFilter *filter = CFilter::createFilter(filterInfo);
  if (filter)
  {
    _filterList.append( filter );
    return true;
  }
  else
    return false;
}


// -----------------------------------------
CMatrix *CFilterCache::applyFilter(const CFilterInfo &filterInfo,
                                   const CMatrix &input, CMatrix *result) const
{
  // Find the filter
  CFilter *filter = 0;
  for (int i=0; (i<_filterList.size()) && (!filter); ++i)
  {
    if (_filterList[i]->getFilterInfo() == filterInfo)
      filter=_filterList[i];
  }

  // Filter not found?
  if (!filter)
  {
    aout<<"ERROR: filter not found"<<endl;
    return result;
  }

  // Apply the filter
  result=filter->apply(input, result);

  return result;
}



