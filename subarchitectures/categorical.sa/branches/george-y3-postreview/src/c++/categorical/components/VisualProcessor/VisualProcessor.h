/**
 * CategoricalVisualProcessor class.
 * \file VisualProcessor.h
 * \author Andrzej Pronobis
 */

#ifndef __PLACE_VISUAL_PROCESSOR__
#define __PLACE_VISUAL_PROCESSOR__

#include "shared/LabelFile.h"
#include <cast/architecture/ManagedComponent.hpp>
#include <CategoricalData.hpp>

class CSystem;
class CImage;
class svm_model;


/**
 * Implements the VisualProcessor component
 */
class CategoricalVisualProcessor: public cast::ManagedComponent
{
public: // Component management

  /** Constructor. */
  CategoricalVisualProcessor();

  /** Destructor. */
  virtual ~CategoricalVisualProcessor();

  /** Handles component configuration. */
  virtual void configure(const std::map<std::string,std::string> &config);

  /** Main thread of the component. */
  virtual void runComponent();

  /** Invoked on start. */
  virtual void start();

  /** Invoked on stop. */
  virtual void stop();


private:

  void createInvalidVisualProcessorStatus(CategoricalData::VisualProcessorStatusPtr visStat);
  void createInvalidVisualResults(CategoricalData::VisualResultsPtr visRes);

  /** Adds an empty status on the WM. */
  std::string addEmptyVisualProcessorStatus();

  /** Adds empty results on the WM. */
  std::string addEmptyVisualResults();

  /** New Image added to wm. */
  void newImage(const cast::cdl::WorkingMemoryChange & change);

  void sendDpUpdateCommand();

private:

  /** Initializes the part of the recognition system implemented in this component. */
  void initProcessing();

  /** Shuts down the recognition system. */
  void finishProcessing();

  /** Performs processing of a single image and produces classification results. */
  void processImage(const CImage &image,
                    CategoricalData::VisualProcessorStatusPtr visualProcessorStatus,
                    CategoricalData::VisualResultsPtr visualResults);

private:

  /** CRFH System. */
  CSystem *_crfhSystem;

  /** SVM Model. */
  svm_model* _svmModel;


private:

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
  categorical::LabelFile _labels;

  int _delay;

};


#endif

