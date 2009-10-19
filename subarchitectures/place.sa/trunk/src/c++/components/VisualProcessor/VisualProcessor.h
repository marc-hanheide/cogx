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
 * PlaceVisualProcessor class.
 * \file VisualProcessor.h
 * \author Andrzej Pronobis
 * \date 2008-08-16
 */

#ifndef __PLACE_VISUAL_PROCESSOR__
#define __PLACE_VISUAL_PROCESSOR__

#include "shared/LabelFile.h"
#include <cast/architecture/ManagedComponent.hpp>
#include <PlaceData.hpp>

class CSystem;
class CImage;
class svm_model;


/**
 * Implements the VisualProcessor component
 */
class PlaceVisualProcessor: public cast::ManagedComponent
{
public: // Component management

  /** Constructor. */
  PlaceVisualProcessor();

  /** Destructor. */
  virtual ~PlaceVisualProcessor();

  /** Handles component configuration. */
  virtual void configure(const std::map<std::string,std::string> &config);

  /** Main thread of the component. */
  virtual void runComponent();

  /** Invoked on start. */
  virtual void start();

  /** Invoked on stop. */
  virtual void stop();


private:

  /** Creates an empty ack on the WM. */
  std::string addEmptyVisualProcessorCommandAck();

  void createInvalidVisualProcessorStatus(PlaceData::VisualProcessorStatusPtr visStat);
  void createInvalidVisualResults(PlaceData::VisualResultsPtr visRes);

  /** Adds an empty status on the WM. */
  std::string addEmptyVisualProcessorStatus();

  /** Adds empty results on the WM. */
  std::string addEmptyVisualResults();

  /** New Image added to wm. */
  void newImage(const cast::cdl::WorkingMemoryChange & change);

  /** New VisualProcessorCommand added to wm. */
  void newVisualProcessorCommand(const cast::cdl::WorkingMemoryChange & change);

  void sendDpUpdateCommand();


private:

  /** Initializes the part of the recognition system implemented in this component. */
  void initProcessing();

  /** Shuts down the recognition system. */
  void finishProcessing();

  /** Performs processing of a single image and produces classification results. */
  void processImage(const CImage &image,
                    PlaceData::VisualProcessorStatusPtr visualProcessorStatus,
                    PlaceData::VisualResultsPtr visualResults);


private:

  /** CRFH System. */
  CSystem *_crfhSystem;

  /** SVM Model. */
  svm_model* _svmModel;


private:

  std::string _visualProcessorCommandAckId;
  std::string _visualResultsId;
  std::string _visualProcessorStatusId;

  struct CopiedImage
  {
    CImage *image;
    bool valid;
    long frameNo;
    cast::cdl::CASTTime realTimeStamp;
    cast::cdl::CASTTime wmTimeStamp;
  };

  /** Queue for received images. */
  std::list<CopiedImage> _imageQueue;

  pthread_cond_t _dataSignalCond;
  pthread_mutex_t _dataSignalMutex;


private:

  /** Name of the config file group. */
  const std::string _cfgGroup;

  /** Specification of descriptors used. */
  std::string _crfhDescriptors;

  /** Small value filtering? */
  bool _crfhFiltering;

  /** When true, the update will be sent after data read.*/
  bool _updateOnRead;

  /** Path to the model file*/
  std::string _modelFilePath;

  /** Small value filtering threshold. */
  double _crfhFiltThreshold;

  /** No. of border pixels skipped. */
  double _crfhSkipBorderPixels;

  int _svmOaoAlg;
  int _svmOaaAlg;
  int _svmMeasure;

  double _confidenceThreshold;

  /** List of labels and names. */
  place::LabelFile _labels;

  int _delay;

};


#endif

