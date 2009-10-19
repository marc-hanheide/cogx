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
 * \file ConfidenceEstimator.h
 * \author Andrzej Pronobis
 * \date 2008-09-04
 */


#ifndef __PLACE_CONFIDENCE_ESTIMATOR__
#define __PLACE_CONFIDENCE_ESTIMATOR__

#include <place/idl/PlaceData.hh>
#include <map>
#include <string>
#include <vector>

namespace place
{

class LabelFile;

class ConfidenceEstimator
{

public:

  ConfidenceEstimator(place::LabelFile *labels): _labels(labels)
    {}


public:
  void getResultsForOutputs(PlaceData::ClassifierResults &results,
                                   const PlaceData::ClassifierOutputs &outputs,
                                   unsigned int outputsCount,
                                   PlaceData::SvmMulticlassAlg multiclassAlg, 
                                   int hypAlg);



private:

  void getResultsForOaOAlg1(PlaceData::ClassifierResults &results,
                                  const PlaceData::ClassifierOutputs &outputs,
                                  unsigned int outputsCount);

  void getResultsForOaAAlg1(PlaceData::ClassifierResults &results,
                                  const PlaceData::ClassifierOutputs &outputs,
                                  unsigned int outputsCount);

  void getResultsForOaAAlg3(PlaceData::ClassifierResults &results,
                                  const PlaceData::ClassifierOutputs &outputs,
                                  unsigned int outputsCount);

private:

  struct NormOut
  {
    double value;
    int class1No;
    int class2No;
  };

  std::vector<NormOut> normalizeDecodeOutputsOaO(PlaceData::ClassifierOutputs outputs, unsigned int outputsCount);
  std::vector<NormOut> normalizeDecodeOutputsOaA(PlaceData::ClassifierOutputs outputs, unsigned int outputsCount);

  place::LabelFile *_labels;

};

}

#endif

