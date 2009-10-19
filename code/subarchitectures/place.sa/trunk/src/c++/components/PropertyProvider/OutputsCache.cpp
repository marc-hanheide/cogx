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
 * \file OutputsCache.cpp
 * \author Andrzej Pronobis
 * \date 2008-09-04
 */

#include "OutputsCache.h"
#include <cast/slice/CDL.hpp>
#include <boost/numeric/conversion/converter.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>

using namespace cast;
using namespace place;
using namespace std;
using namespace boost;


// ------------------------------------------------------
OutputsCache::OutputsCache(double positionBinSize, double headingBinSize) :
    _posBinSize(positionBinSize), _headBinSize(headingBinSize)
{
}


// ------------------------------------------------------
void OutputsCache::clear()
{
  _cache.clear();
}


// ------------------------------------------------------
void OutputsCache::addOutputs(double x, double y, double theta, const PlaceData::ClassifierOutputs &outputs)
{
  typedef boost::numeric::converter<int, double, boost::numeric::conversion_traits<int, double>,
                                    boost::numeric::def_overflow_handler, 
                                    boost::numeric::RoundEven<double> > Round2Int;
  // Get bin coordinates
  int bx, by, bt;
  bx = Round2Int::convert(x/_posBinSize);
  by = Round2Int::convert(y/_posBinSize);
  bt = Round2Int::convert(theta/_headBinSize);

  // Get bin name
  string binName = lexical_cast<string>(bx)+" "+lexical_cast<string>(by)+" "+lexical_cast<string>(bt);

  // Accumulate or add
  map<string, Bin>::iterator i = _cache.find(binName);
  if (i==_cache.end())
  { // Not found - add
    Bin b;
    b.count = 1;
    b.accOutputs = outputs;
    _cache[binName]=b;
  }
  else
  { // Found - accumulate
    Bin &b = _cache[binName];
    b.count++;
    accumulateOutputs(b.accOutputs, outputs);
  }

}


// ------------------------------------------------------
void OutputsCache::getAccumulatedOutputs(PlaceData::ClassifierOutputs &outputs, unsigned int &binCount) const
{
  if (isEmpty())
    throw CASTException(__HERE__, "Cannot accumulate when the cache is empty!");

  // Get iterator
  map<string,Bin>::const_iterator it;

  // Set outputs to first bin
  it=_cache.begin();
  binCount=1;
  outputs = (*it).second.accOutputs;
  for (unsigned int i=0; i<outputs.length(); ++i)
    outputs[i].value/=static_cast<double>((*it).second.count);

  // Go through all bins and accumulate
  for (it++ ; it!=_cache.end(); it++)
  {
    binCount++;
    accumulateUnnormalizedOutputs(outputs, (*it).second.accOutputs, (*it).second.count);
  }
}


// ------------------------------------------------------
void OutputsCache::accumulateOutputs(PlaceData::ClassifierOutputs &accOutputs, 
                                     const PlaceData::ClassifierOutputs &outputs) const
{
  // Check if the outputs match, if not, raise exception
  int accOutCount = accOutputs.length();
  int outCount = outputs.length();
  if (accOutCount != outCount)
    throw CASTException(__HERE__, "Outputs cannot be accumulated as they do not match (%d, %d)!", 
                        accOutCount, outCount);
  if (accOutCount == 0)
    throw CASTException(__HERE__, "No outputs in output vector!");

  // Now accumulate and check
  for (int i=0; i<accOutCount; ++i)
  {
    if (string(accOutputs[i].name) != string(outputs[i].name))
    {
      throw CASTException(__HERE__, "Outputs cannot be accumulated as they do not match (%s, %s)!", 
                          string(accOutputs[i].name).c_str(), string(outputs[i].name).c_str());
    }
    accOutputs[i].value+=outputs[i].value;
  }
}


// ------------------------------------------------------
void OutputsCache::accumulateUnnormalizedOutputs(PlaceData::ClassifierOutputs &accOutputs, 
                                                 const PlaceData::ClassifierOutputs &outputs, 
                                                 unsigned int binOutputsCount) const
{
  // Check if the outputs match, if not, raise exception
  int accOutCount = accOutputs.length();
  int outCount = outputs.length();
  if (accOutCount != outCount)
    throw CASTException(__HERE__, "Outputs cannot be accumulated as they do not match (%d, %d)!", 
                        accOutCount, outCount);
  if (accOutCount == 0)
    throw CASTException(__HERE__, "No outputs in output vector!");


  // Now accumulate and check
  double binOutputsCountDbl = static_cast<double>(binOutputsCount);
  for (int i=0; i<accOutCount; ++i)
  {
    if (string(accOutputs[i].name) != string(outputs[i].name))
    {
      throw CASTException(__HERE__, "Outputs cannot be accumulated as they do not match (%s, %s)!", 
                          string(accOutputs[i].name).c_str(), string(outputs[i].name).c_str());
    }
    accOutputs[i].value+=(outputs[i].value/binOutputsCountDbl);
  }
}


// ------------------------------------------------------
void OutputsCache::print() const
{
  map<string,Bin>::const_iterator it;
  cout<<endl;
  for (it=_cache.begin() ; it!=_cache.end(); it++)
  {
    cout<< (*it).first << " |";
    for (unsigned int i=0; i<(*it).second.accOutputs.length(); ++i)
      cout<<" "<<(*it).second.accOutputs[i].name<<"="<<(*it).second.accOutputs[i].value;
    cout<<endl;
  }
}


// ------------------------------------------------------
void OutputsCache::printAccumulatedOutputs() const
{
  PlaceData::ClassifierOutputs outputs;
  unsigned int binCount;

  // Get outputs
  getAccumulatedOutputs(outputs, binCount);

  cout<<endl;
  for (unsigned int i=0; i<outputs.length(); ++i)
    cout<<" "<<outputs[i].name<<"="<<outputs[i].value;
  cout<<endl;
}


// ------------------------------------------------------
void OutputsCache::printNormalizedAccumulatedOutputs() const
{
  PlaceData::ClassifierOutputs outputs;
  unsigned int binCount;

  // Get outputs
  getAccumulatedOutputs(outputs, binCount);
  double binCountDbl = static_cast<double>(binCount);

  cout<<endl;
  for (unsigned int i=0; i<outputs.length(); ++i)
    cout<<" "<<outputs[i].name<<"="<<outputs[i].value/binCountDbl;
  cout<<endl;
}




