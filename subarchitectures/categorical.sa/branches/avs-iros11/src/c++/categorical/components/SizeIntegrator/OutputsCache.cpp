/**
 * OutputsCache class.
 * \file OutputsCache.cpp
 * \author Andrzej Pronobis
 */

#include "OutputsCache.h"
#include <cast/architecture/ManagedComponent.hpp>
#include <boost/numeric/conversion/converter.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>

using namespace cast;
using namespace categorical;
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
void OutputsCache::addOutputs(double x, double y, double theta, const CategoricalData::ClassifierOutputs &outputs)
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
void OutputsCache::getAccumulatedOutputs(CategoricalData::ClassifierOutputs &outputs, unsigned int &binCount) const
{
  if (isEmpty())
    throw CASTException("Cannot accumulate when the cache is empty!");

  // Get iterator
  map<string,Bin>::const_iterator it;

  // Set outputs to first bin
  it=_cache.begin();
  binCount=1;
  outputs = (*it).second.accOutputs;
  for (unsigned int i=0; i<outputs.size(); ++i)
    outputs[i].value/=static_cast<double>((*it).second.count);

  // Go through all bins and accumulate
  for (it++ ; it!=_cache.end(); it++)
  {
    binCount++;
    accumulateUnnormalizedOutputs(outputs, (*it).second.accOutputs, (*it).second.count);
  }
}


// ------------------------------------------------------
void OutputsCache::accumulateOutputs(CategoricalData::ClassifierOutputs &accOutputs,
                                     const CategoricalData::ClassifierOutputs &outputs) const
{
  // Check if the outputs match, if not, raise exception
  int accOutCount = accOutputs.size();
  int outCount = outputs.size();
  if (accOutCount != outCount)
    throw CASTException(exceptionMessage(__HERE__, "Outputs cannot be accumulated as they do not match (%d, %d)!",
                        accOutCount, outCount));
  if (accOutCount == 0)
    throw CASTException(exceptionMessage(__HERE__, __HERE__, "No outputs in output vector!"));

  // Now accumulate and check
  for (int i=0; i<accOutCount; ++i)
  {
    if (string(accOutputs[i].name) != string(outputs[i].name))
    {
      throw CASTException(exceptionMessage(__HERE__, __HERE__, "Outputs cannot be accumulated as they do not match (%s, %s)!",
                          string(accOutputs[i].name).c_str(), string(outputs[i].name).c_str()));
    }
    accOutputs[i].value+=outputs[i].value;
  }
}


// ------------------------------------------------------
void OutputsCache::accumulateUnnormalizedOutputs(CategoricalData::ClassifierOutputs &accOutputs,
                                                 const CategoricalData::ClassifierOutputs &outputs,
                                                 unsigned int binOutputsCount) const
{
  // Check if the outputs match, if not, raise exception
  int accOutCount = accOutputs.size();
  int outCount = outputs.size();
  if (accOutCount != outCount)
    throw CASTException(exceptionMessage(__HERE__, __HERE__, "Outputs cannot be accumulated as they do not match (%d, %d)!",
                        accOutCount, outCount));
  if (accOutCount == 0)
    throw CASTException(exceptionMessage(__HERE__, __HERE__, "No outputs in output vector!"));


  // Now accumulate and check
  double binOutputsCountDbl = static_cast<double>(binOutputsCount);
  for (int i=0; i<accOutCount; ++i)
  {
    if (string(accOutputs[i].name) != string(outputs[i].name))
    {
      throw CASTException(exceptionMessage(__HERE__, __HERE__, "Outputs cannot be accumulated as they do not match (%s, %s)!",
                          string(accOutputs[i].name).c_str(), string(outputs[i].name).c_str()));
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
    for (unsigned int i=0; i<(*it).second.accOutputs.size(); ++i)
      cout<<" "<<(*it).second.accOutputs[i].name<<"="<<(*it).second.accOutputs[i].value;
    cout<<endl;
  }
}


// ------------------------------------------------------
void OutputsCache::printAccumulatedOutputs() const
{
  CategoricalData::ClassifierOutputs outputs;
  unsigned int binCount;

  // Get outputs
  getAccumulatedOutputs(outputs, binCount);

  cout<<endl;
  for (unsigned int i=0; i<outputs.size(); ++i)
    cout<<" "<<outputs[i].name<<"="<<outputs[i].value;
  cout<<endl;
}


// ------------------------------------------------------
void OutputsCache::printNormalizedAccumulatedOutputs() const
{
  CategoricalData::ClassifierOutputs outputs;
  unsigned int binCount;

  // Get outputs
  getAccumulatedOutputs(outputs, binCount);
  double binCountDbl = static_cast<double>(binCount);

  cout<<endl;
  for (unsigned int i=0; i<outputs.size(); ++i)
    cout<<" "<<outputs[i].name<<"="<<outputs[i].value/binCountDbl;
  cout<<endl;
}




