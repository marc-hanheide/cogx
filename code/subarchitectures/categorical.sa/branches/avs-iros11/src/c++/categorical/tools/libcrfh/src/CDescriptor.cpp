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
* \file CDescriptor.cpp
* \author Andrzej Pronobis
*
* Contains implementation of the CDescriptor class.
*/

#include <math.h>
#include <QtCore/QString>
#include "CFilter.h"
#include "CFilterCache.h"
#include "CChannelCache.h"
#include "CScaleSpaceCache.h"

#include "CDescriptor.h"




// -----------------------------------------
CDescriptor *CDescriptor::createDescriptor(DescriptorType descriptorType, double scale, int bins)
{
  switch(descriptorType)
  {
  case DT_L:
    return new CLDescriptor(scale, bins);
  case DT_Lx:
    return new CLxDescriptor(scale, bins);
  case DT_Lxx:
    return new CLxxDescriptor(scale, bins);
  case DT_Ly:
    return new CLyDescriptor(scale, bins);
  case DT_Lyy:
    return new CLyyDescriptor(scale, bins);
  case DT_Lxy:
    return new CLxyDescriptor(scale, bins);

  default:
    return 0;
  }
}


// -----------------------------------------
CDescriptor *CDescriptor::createDescriptor(QString descriptorName, double scale, int bins)
{
  if (descriptorName=="L")
    return new CLDescriptor(scale, bins);
  else if (descriptorName=="Lx")
    return new CLxDescriptor(scale, bins);
  else if (descriptorName=="Lxx")
    return new CLxxDescriptor(scale, bins);
  else if (descriptorName=="Ly")
    return new CLyDescriptor(scale, bins);
  else if (descriptorName=="Lyy")
    return new CLyyDescriptor(scale, bins);
  else if (descriptorName=="Lxy")
    return new CLxyDescriptor(scale, bins);
  else
    return 0;
}




// -----------------------------------------
void CLDescriptor::createRequiredFilters(CFilterCache &filterCache)
{
  CGaussianFilterInfo gfi(_scale);
  filterCache.createFilter(gfi);
}


// -----------------------------------------
void CLDescriptor::createRequiredChannels(CChannelCache &channelCache)
{
  channelCache.createChannel(CT_L);
}


// -----------------------------------------
void CLDescriptor::createRequiredScales(CScaleSpaceCache &scaleSpaceCache)
{
  scaleSpaceCache.createScaleSpaceSample(CT_L, _scale);
}


// -----------------------------------------
CMatrix *CLDescriptor::apply(const CChannelCache &channelCache,
                               const CScaleSpaceCache &scaleSpaceCache,
                               const CFilterCache &filterCache,
                               CMatrix *result)
{
  const CMatrix *scaleSpaceSample = scaleSpaceCache.getScaleSpaceSample(CT_L, _scale);
  result = scaleSpaceSample->getDeepCopy(result);
  return result;
}






// -----------------------------------------
void CLxxDescriptor::createRequiredFilters(CFilterCache &filterCache)
{
  CGaussianFilterInfo gfi(_scale);
  CCartesianFilterInfo cfi(2, 0);

  filterCache.createFilter(gfi);
  filterCache.createFilter(cfi);
}


// -----------------------------------------
void CLxxDescriptor::createRequiredChannels(CChannelCache &channelCache)
{
  channelCache.createChannel(CT_L);
}


// -----------------------------------------
void CLxxDescriptor::createRequiredScales(CScaleSpaceCache &scaleSpaceCache)
{
  scaleSpaceCache.createScaleSpaceSample(CT_L, _scale);
}


// -----------------------------------------
CMatrix *CLxxDescriptor::apply(const CChannelCache &channelCache,
                               const CScaleSpaceCache &scaleSpaceCache,
                               const CFilterCache &filterCache,
                               CMatrix *result)
{
  // Get scale space sample
  const CMatrix *scaleSpaceSample = scaleSpaceCache.getScaleSpaceSample(CT_L, _scale);

  // Filter with second-derivative filter
  CCartesianFilterInfo cfi(2, 0);
  result=filterCache.applyFilter(cfi, *scaleSpaceSample, result);

  // Normalize
  double factor = _scale;
  (*result)*=factor; // scale^(2/2)=sqrt(scale)^2 = scale

  // Return the result
  return result;
}





// -----------------------------------------
void CLxDescriptor::createRequiredFilters(CFilterCache &filterCache)
{
  CGaussianFilterInfo gfi(_scale);
  CCartesianFilterInfo cfi(1, 0);

  filterCache.createFilter(gfi);
  filterCache.createFilter(cfi);
}


// -----------------------------------------
void CLxDescriptor::createRequiredChannels(CChannelCache &channelCache)
{
  channelCache.createChannel(CT_L);
}


// -----------------------------------------
void CLxDescriptor::createRequiredScales(CScaleSpaceCache &scaleSpaceCache)
{
  scaleSpaceCache.createScaleSpaceSample(CT_L, _scale);
}


// -----------------------------------------
CMatrix *CLxDescriptor::apply(const CChannelCache &channelCache,
                               const CScaleSpaceCache &scaleSpaceCache,
                               const CFilterCache &filterCache,
                               CMatrix *result)
{
  // Get scale space sample
  const CMatrix *scaleSpaceSample = scaleSpaceCache.getScaleSpaceSample(CT_L, _scale);

  // Filter with second-derivative filter
  CCartesianFilterInfo cfi(1, 0);
  result=filterCache.applyFilter(cfi, *scaleSpaceSample, result);

  // Normalize
  double factor = sqrt(_scale);
  (*result)*=factor; // scale^(1/2)=sqrt(scale)

  // Return the result
  return result;
}





// -----------------------------------------
void CLyDescriptor::createRequiredFilters(CFilterCache &filterCache)
{
  CGaussianFilterInfo gfi(_scale);
  CCartesianFilterInfo cfi(0, 1);

  filterCache.createFilter(gfi);
  filterCache.createFilter(cfi);
}


// -----------------------------------------
void CLyDescriptor::createRequiredChannels(CChannelCache &channelCache)
{
  channelCache.createChannel(CT_L);
}


// -----------------------------------------
void CLyDescriptor::createRequiredScales(CScaleSpaceCache &scaleSpaceCache)
{
  scaleSpaceCache.createScaleSpaceSample(CT_L, _scale);
}


// -----------------------------------------
CMatrix *CLyDescriptor::apply(const CChannelCache &channelCache,
                               const CScaleSpaceCache &scaleSpaceCache,
                               const CFilterCache &filterCache,
                               CMatrix *result)
{
  // Get scale space sample
  const CMatrix *scaleSpaceSample = scaleSpaceCache.getScaleSpaceSample(CT_L, _scale);

  // Filter with second-derivative filter
  CCartesianFilterInfo cfi(0, 1);
  result=filterCache.applyFilter(cfi, *scaleSpaceSample, result);

  // Normalize
  double factor = sqrt(_scale);
  (*result)*=factor; // scale^(1/2)=sqrt(scale)

  // Return the result
  return result;
}





// -----------------------------------------
void CLyyDescriptor::createRequiredFilters(CFilterCache &filterCache)
{
  CGaussianFilterInfo gfi(_scale);
  CCartesianFilterInfo cfi(0, 2);

  filterCache.createFilter(gfi);
  filterCache.createFilter(cfi);
}


// -----------------------------------------
void CLyyDescriptor::createRequiredChannels(CChannelCache &channelCache)
{
  channelCache.createChannel(CT_L);
}


// -----------------------------------------
void CLyyDescriptor::createRequiredScales(CScaleSpaceCache &scaleSpaceCache)
{
  scaleSpaceCache.createScaleSpaceSample(CT_L, _scale);
}


// -----------------------------------------
CMatrix *CLyyDescriptor::apply(const CChannelCache &channelCache,
                               const CScaleSpaceCache &scaleSpaceCache,
                               const CFilterCache &filterCache,
                               CMatrix *result)
{
  // Get scale space sample
  const CMatrix *scaleSpaceSample = scaleSpaceCache.getScaleSpaceSample(CT_L, _scale);

  // Filter with second-derivative filter
  CCartesianFilterInfo cfi(0, 2);
  result=filterCache.applyFilter(cfi, *scaleSpaceSample, result);

  // Normalize
  double factor = _scale;
  (*result)*=factor; // scale^(1/2)=sqrt(scale)

  // Return the result
  return result;
}





// -----------------------------------------
void CLxyDescriptor::createRequiredFilters(CFilterCache &filterCache)
{
  CGaussianFilterInfo gfi(_scale);
  CCartesianFilterInfo cfi(1, 1);

  filterCache.createFilter(gfi);
  filterCache.createFilter(cfi);
}


// -----------------------------------------
void CLxyDescriptor::createRequiredChannels(CChannelCache &channelCache)
{
  channelCache.createChannel(CT_L);
}


// -----------------------------------------
void CLxyDescriptor::createRequiredScales(CScaleSpaceCache &scaleSpaceCache)
{
  scaleSpaceCache.createScaleSpaceSample(CT_L, _scale);
}


// -----------------------------------------
CMatrix *CLxyDescriptor::apply(const CChannelCache &channelCache,
                               const CScaleSpaceCache &scaleSpaceCache,
                               const CFilterCache &filterCache,
                               CMatrix *result)
{
  // Get scale space sample
  const CMatrix *scaleSpaceSample = scaleSpaceCache.getScaleSpaceSample(CT_L, _scale);

  // Filter with second-derivative filter
  CCartesianFilterInfo cfi(1, 1);
  result=filterCache.applyFilter(cfi, *scaleSpaceSample, result);

  // Normalize
  double factor = _scale;
  (*result)*=factor; // scale^(1/2)=sqrt(scale)

  // Return the result
  return result;
}
