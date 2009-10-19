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
 * PlaceLaserProcessor class.
 * \file LaserProcessor.h
 * \author Andrzej Pronobis
 * \date 2008-09-01
 */

#ifndef __PLACE_LASER_PROCESSOR__
#define __PLACE_LASER_PROCESSOR__

#include "shared/LabelFile.h"
#include "tools/sequentialadaboost/SequentialAdaBoostClassifier.h"
#include <cast/architecture/ManagedComponent.hpp>
#include <PlaceData.hpp>

class svm_model;

/**
 * Implements the LaserProcessor component
 */
class PlaceLaserProcessor: public cast::ManagedComponent
{
public: // Component management

  /** Constructor. */
  PlaceLaserProcessor();

  /** Destructor. */
  virtual ~PlaceLaserProcessor();

  /** Handles component configuration. */
  virtual void configure(const std::map<std::string,std::string> &config);

  /** Main thread of the component. */
  virtual void runComponent();

  /** Invoked on start. */
  virtual void start();

  /** Invoked on stop. */
  virtual void stop();


private:

  /** Reads the feature config file. */
  bool readLaserFeatureConfigFile();

  /** Reads the scales from file. */
  bool readScaleConfigFile();

  /** Creates an empty ack on the WM. */
  std::string addEmptyLaserProcessorCommandAck();

  void createInvalidLaserProcessorStatus(PlaceData::LaserProcessorStatusPtr lasStat);
  void createInvalidLaserResults(PlaceData::LaserResultsPtr lasRes);

  /** Adds an empty status on the WM. */
  std::string addEmptyLaserProcessorStatus();

  /** Adds empty results on the WM. */
  std::string addEmptyLaserResults();

  /** New LaserScan added to wm. */
  void newLaserScan(const cast::cdl::WorkingMemoryChange & change);

  /** New LaserProcessorCommand added to wm. */
  void newLaserProcessorCommand(const cast::cdl::WorkingMemoryChange & change);

  void sendDpUpdateCommand();


private:

  /** Initializes the part of the recogntion system implemented in this component. */
  void initProcessing();

  /** Shuts down the recognition system. */
  void finishProcessing();

  /** Performs processing of a single laser scan and produces classification results. */
  void processLaserScan(Laser::Scan2d &scan,
                        PlaceData::LaserProcessorStatusPtr laserProcessorStatus,
                        PlaceData::LaserResultsPtr laserResults);


private:

  /** SVM Model. */
  svm_model* _svmModel;

  /** List of features and their params. */
  oscar::dyntab_featuresInfo* _featureInfoList;

  oscar::RangeExampleGeneral *_rangeExample;

  SequentialAdaBoostClassifier classifier;

  struct Scale
  {
    double min;
    double max;
  };
  std::vector<Scale> _scales;


private:

  std::string _laserProcessorCommandAckId;
  std::string _laserResultsId;
  std::string _laserProcessorStatusId;

  /** Queue for received scans. */
  std::list<PlaceData::LaserScanPtr> _scanQueue;

  pthread_cond_t _dataSignalCond;
  pthread_mutex_t _dataSignalMutex;


private:

  /** Name of the config file group. */
  const std::string _cfgGroup;

  /** When true, the update will be sent after data read.*/
  bool _updateOnRead;

  /** Path to the feature config file. */
  std::string _featureFilePath;

  /** Path to the scale config file. */
  std::string _scaleFilePath;

  /** Path to the model file*/
  std::string _modelFilePath;

  int _svmOaoAlg;
  int _svmOaaAlg;
  int _svmMeasure;

  double _confidenceThreshold;

  /** List of labels and names. */
  place::LabelFile _labels;
};


#endif

