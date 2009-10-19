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
 * PlaceCueIntegrator class.
 * \file CueIntegrator.h
 * \author Andrzej Pronobis
 * \date 2008-09-02
 */

#ifndef __PLACE_CUE_INTEGRATOR__
#define __PLACE_CUE_INTEGRATOR__

#include "shared/LabelFile.h"
#include <cast/architecture/ManagedComponent.hpp>
#include <PlaceData.hpp>

class svm_model;


/**
 * Implements the CueIntegrator component
 */
class PlaceCueIntegrator: public cast::ManagedComponent
{
public: // Component management

  /** Constructor. */
  PlaceCueIntegrator();

  /** Destructor. */
  virtual ~PlaceCueIntegrator();

  /** Handles component configuration. */
  virtual void configure(const std::map<std::string,std::string> &config);

  /** Main thread of the component. */
  virtual void runComponent();

  /** Invoked on start. */
  virtual void start();

  /** Invoked on stop. */
  virtual void stop();


private:

  /** Reads the scales from file. */
  bool readScaleConfigFile();

  void createInvalidIntegratedResults(PlaceData::IntegratedResultsPtr intgRes);

  /** Adds empty results on the WM. */
  std::string addEmptyIntegratedResults();

  /** New LaserResults added to wm. */
  void newLaserResults(const cast::cdl::WorkingMemoryChange & change);

  /** New VisualResults added to wm. */
  void newVisualResults(const cast::cdl::WorkingMemoryChange & change);


private:

  /** Returns true if the component has collected all the data necessary for integration. */
  bool readyToIntegrate();

  /** Initializes the part of the recogntion system implemented in this component. */
  void initProcessing();

  /** Shuts down the recognition system. */
  void finishProcessing();

  /** Performs cue integration and produces integrated classification results. */
  void integrate(const PlaceData::VisualResultsPtr visualRes, const PlaceData::LaserResultsPtr laserRes,
                 PlaceData::IntegratedResultsPtr intgRes);


private:

  /** SVM Model. */
  svm_model* _svmModel;

  struct Scale
  {
    double min;
    double max;
  };
  std::vector<Scale> _scales;


private:

  std::string _integratedResultsId;

  /** Queue for received visual results. */
  std::list<PlaceData::VisualResultsPtr> _visualQueue;

  /** Queue for received laser results. */
  std::list<PlaceData::LaserResultsPtr> _laserQueue;

  pthread_cond_t _dataSignalCond;
  pthread_mutex_t _dataSignalMutex;


private:

  /** Name of the config file group. */
  const std::string _cfgGroup;

  /** Path to the model file*/
  std::string _modelFilePath;

  /** Path to the scale config file. */
  std::string _scaleFilePath;

  int _svmOaoAlg;
  int _svmOaaAlg;
  int _svmMeasure;

  double _confidenceThreshold;

  /** List of labels and names. */
  place::LabelFile _labels;
};


#endif

