// ==================================================================
// Place.SA - Place Classification Subarchitecture
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of Place.SA.
//
// Place.SA is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Place.SA is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Place.SA. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================

/**
 * OutputsCache class.
 * \file OutputsCache.h
 * \author Andrzej Pronobis
 * \date 2008-09-04
 */


#ifndef __PLACE_OUTPUTS_CACHE__
#define __PLACE_OUTPUTS_CACHE__

#include <CategoricalData.hpp>
#include <map>
#include <string>

namespace categorical
{

class OutputsCache
{
public:

  /** Constructor. */
  OutputsCache(double positionBinSize, double headingBinSize);

  /** Clears the cache. */
  void clear();

  /** Is the cache empty? */
  bool isEmpty() const
  { return _cache.empty(); }

  /** Adds outputs to the cache. */
  void addOutputs(double x, double y, double theta, const CategoricalData::ClassifierOutputs &outputs);

  /** Returns accumulated outputs. */
  void getAccumulatedOutputs(CategoricalData::ClassifierOutputs &outputs, unsigned int &binCount) const;

  /** Prints the contents of the cache. */
  void print() const;

  /** Accumulates and prints accumulated outputs. */
  void printAccumulatedOutputs() const;

  /** Accumulates and prints accumulated outputs, normalized. */
  void printNormalizedAccumulatedOutputs() const;


private:

  void accumulateOutputs(CategoricalData::ClassifierOutputs &accOutputs,
                         const CategoricalData::ClassifierOutputs &outputs) const;

  void accumulateUnnormalizedOutputs(CategoricalData::ClassifierOutputs &accOutputs,
                                     const CategoricalData::ClassifierOutputs &outputs,
                                     unsigned int binOutputsCount) const;

private:

  double _posBinSize;
  double _headBinSize;

  struct Bin
  {
    unsigned int count;
    CategoricalData::ClassifierOutputs accOutputs;
  };

  std::map<std::string, Bin> _cache;

};

}

#endif

