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
 * ConfidenceEstimator class.
 * \file ConfidenceEstimator.cpp
 * \author Andrzej Pronobis
 * \date 2008-09-04
 */

#include "ConfidenceEstimator.h"
#include "place/shared/LabelFile.h"
#include <cast/core/CASTException.hpp>
#include <boost/numeric/conversion/converter.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>

using namespace cast;
using namespace place;
using namespace std;
using namespace boost;

#define INF HUGE_VAL


// ------------------------------------------------------
void ConfidenceEstimator::getResultsForOutputs(PlaceData::ClassifierResults &results,
                                               const PlaceData::ClassifierOutputs &outputs,
                                               unsigned int outputsCount,
                                               PlaceData::SvmMulticlassAlg multiclassAlg, int hypAlg)
{
  // If the outputs are empty, return empty results
  if (outputs.length()==0)
  {
    results = PlaceData::ClassifierResults();
    return;
  }

  if (multiclassAlg==PlaceData::SMA_OAO)
  {
    if (hypAlg==1)
      getResultsForOaOAlg1(results, outputs, outputsCount);
    else
      throw CASTException(__HERE__, "Incorrect hypotheses estimation algorithm!");
  }
  else if (multiclassAlg==PlaceData::SMA_OAA)
  {
    if (hypAlg==1)
      getResultsForOaAAlg1(results, outputs, outputsCount);
    else if (hypAlg==3)
      getResultsForOaAAlg3(results, outputs, outputsCount);
    else
      throw CASTException(__HERE__, "Incorrect hypotheses estimation algorithm!");
  }
  else
    throw CASTException(__HERE__, "Incorrect multiclass algorithm!");
}


// ------------------------------------------------------
std::vector<ConfidenceEstimator::NormOut> ConfidenceEstimator::normalizeDecodeOutputsOaO(
    PlaceData::ClassifierOutputs outputs, unsigned int outputsCount)
{
  vector<NormOut> normOutputs(outputs.length());
  double outputsCountDbl = outputsCount;

  for (unsigned int i=0; i<outputs.length(); ++i)
  {
    if (string(outputs[i].name).size()!=3)
    {
      throw CASTException(__HERE__, "Incorrect output name!");
    }
    normOutputs[i].value = outputs[i].value/outputsCountDbl;
    normOutputs[i].class1No = lexical_cast<int>(string(1,string(outputs[i].name)[0]));
    normOutputs[i].class2No = lexical_cast<int>(string(1,string(outputs[i].name)[2]));
  }

  return normOutputs;
}


// ------------------------------------------------------
std::vector<ConfidenceEstimator::NormOut> ConfidenceEstimator::normalizeDecodeOutputsOaA(
    PlaceData::ClassifierOutputs outputs, unsigned int outputsCount)
{
  vector<NormOut> normOutputs(outputs.length());
  double outputsCountDbl = outputsCount;

  for (unsigned int i=0; i<outputs.length(); ++i)
  {
    normOutputs[i].value = outputs[i].value/outputsCountDbl;
    normOutputs[i].class1No = lexical_cast<int>(string(outputs[i].name));
    normOutputs[i].class2No = -1;
  }

  return normOutputs;
}


// ------------------------------------------------------
void ConfidenceEstimator::getResultsForOaOAlg1(PlaceData::ClassifierResults &results,
                                               const PlaceData::ClassifierOutputs &outputs,
                                               unsigned int outputsCount)
{
  // Normalize outputs
  std::vector<NormOut> normOut = normalizeDecodeOutputsOaO(outputs, outputsCount);


  // ---------------------------------
  // Get the winner class
  // ---------------------------------
  // Zero the votes and get class count
  map<int, int> votes;
  for (unsigned int i=0; i<normOut.size(); ++i)
  {
    votes[normOut[i].class1No]=0;
    votes[normOut[i].class2No]=0;
  }
  int classCount = votes.size();
  // Vote
  for (unsigned int i=0; i<normOut.size(); ++i)
  {
    if (normOut[i].value>0)
      votes[normOut[i].class1No]++;
    else
      votes[normOut[i].class2No]++;
  }
  // Find max
  int winnerClassNo=-1;
  int winnerClassVotes=-1;
  map<int,int>::iterator it;
  for (it=votes.begin(); it!=votes.end(); ++it)
  {
    if ((*it).second > winnerClassVotes)
    {
      winnerClassVotes=(*it).second;
      winnerClassNo=(*it).first;
    }
  }

  /*
  for (unsigned int i=0; i<normOut.size(); ++i)
  {
    cout<<normOut[i].class1No<<" "<<normOut[i].class2No<<" "<<normOut[i].value<<endl;
  }
  */

  // ---------------------------------
  // Get the confidences
  // ---------------------------------
  results = PlaceData::ClassifierResults();
  results.length(classCount);
  results[0].classNo=winnerClassNo;
  results[0].className=_labels->labelNoToName(winnerClassNo).c_str();

  for (int k=1; k<classCount; ++k)
  {
    double minOut = 10000000.0;
    int minClassNo = -1;
    int minOutIdx=-1;
    for (unsigned int i=0; i<normOut.size(); ++i)
    {
      if (normOut[i].class1No == winnerClassNo)
      {
        if (normOut[i].value<minOut)
        {
          minOut=normOut[i].value;
          minClassNo = normOut[i].class2No;
          minOutIdx = i;
        }
      }
      if (normOut[i].class2No == winnerClassNo)
      {
        if ((-normOut[i].value)<minOut)
        {
          minOut=-normOut[i].value;
          minClassNo = normOut[i].class1No;
          minOutIdx = i;
        }
      }
    } //for
    normOut[minOutIdx].class1No = -1;
    normOut[minOutIdx].class2No = -1;
    results[k].classNo=minClassNo;
    results[k].className=_labels->labelNoToName(minClassNo).c_str();
    results[k].confidence=minOut;
  }
  results[0].confidence=results[1].confidence;
}


// ------------------------------------------------------
void ConfidenceEstimator::getResultsForOaAAlg1(PlaceData::ClassifierResults &results,
                                               const PlaceData::ClassifierOutputs &outputs,
                                               unsigned int outputsCount)
{
  // Normalize outputs
  std::vector<NormOut> normOut = normalizeDecodeOutputsOaA(outputs, outputsCount);
  int classCount = normOut.size();

  // ---------------------------------
  // Get the winner class
  // ---------------------------------
  int winnerIdx=0;
  for (unsigned int i=1; i<normOut.size(); ++i)
  {
    if(normOut[i].value > normOut[winnerIdx].value)
      winnerIdx = i;
  }
  int winnerClassNo=normOut[winnerIdx].class1No;
  double winnerValue=normOut[winnerIdx].value;

  // Set the winner class
  results = PlaceData::ClassifierResults();
  results.length(classCount);
  results[0].classNo=winnerClassNo;
  results[0].className=_labels->labelNoToName(winnerClassNo).c_str();


  // ---------------------------------
  // Get the order of hypotheses and confidence
  // ---------------------------------
  normOut[winnerIdx].value=-INF;
  double maxValue=0;
  for(int i=1; i<classCount; ++i)
  {
    int maxIdx=0;
    for(int j=1; j<classCount; ++j)
      if (normOut[j].value>normOut[maxIdx].value)
        maxIdx=j;
    if (i==1)
      maxValue=normOut[maxIdx].value;
    results[i].classNo=normOut[maxIdx].class1No;
    results[i].className=_labels->labelNoToName(normOut[maxIdx].class1No).c_str();
    results[i].confidence=winnerValue-normOut[maxIdx].value;
    normOut[maxIdx].value=-INF;
  }

  // Set confidence of the decision
  results[0].confidence=winnerValue-maxValue;
}


// ------------------------------------------------------
void ConfidenceEstimator::getResultsForOaAAlg3(PlaceData::ClassifierResults &results,
                                               const PlaceData::ClassifierOutputs &outputs,
                                               unsigned int outputsCount)
{
  // Normalize outputs
  std::vector<NormOut> normOut = normalizeDecodeOutputsOaA(outputs, outputsCount);
  int classCount = normOut.size();

  // ---------------------------------
  // Get the winner class
  // ---------------------------------
  int winnerIdx=0;
  for (unsigned int i=1; i<normOut.size(); ++i)
  {
    if(normOut[i].value < normOut[winnerIdx].value)
      winnerIdx = i;
  }
  int winnerClassNo=normOut[winnerIdx].class1No;
  double winnerValue=normOut[winnerIdx].value;

  // Set the winner class
  results = PlaceData::ClassifierResults();
  results.length(classCount);
  results[0].classNo=winnerClassNo;
  results[0].className=_labels->labelNoToName(winnerClassNo).c_str();


  // ---------------------------------
  // Get the order of hypotheses and confidence
  // ---------------------------------
  normOut[winnerIdx].value=INF;
  double minValue=0;
  for(int i=1; i<classCount; ++i)
  {
    int minIdx=0;
    for(int j=1; j<classCount; ++j)
      if (normOut[j].value<normOut[minIdx].value)
        minIdx=j;
    if (i==1)
      minValue=normOut[minIdx].value;
    results[i].classNo=normOut[minIdx].class1No;
    results[i].className=_labels->labelNoToName(normOut[minIdx].class1No).c_str();
    results[i].confidence=winnerValue-normOut[minIdx].value;
    normOut[minIdx].value=INF;
  }

  // Set confidence of the decision
  results[0].confidence=minValue-winnerValue;

}


