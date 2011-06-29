/**
 * CategoricalLaserProcessor class.
 * \file LaserProcessor.h
 * \author Andrzej Pronobis
 */

#ifndef __PLACE_LASER_PROCESSOR__
#define __PLACE_LASER_PROCESSOR__

#include "shared/LabelFile.h"
#include "tools/sequentialadaboost/SequentialAdaBoostClassifier.h"
#include <cast/architecture/ManagedComponent.hpp>
#include <CategoricalData.hpp>

class svm_model;

/**
 * Implements the LaserProcessor component
 */
class CategoricalLaserProcessor: public cast::ManagedComponent
{
public: // Component management

  /** Constructor. */
  CategoricalLaserProcessor();

  /** Destructor. */
  virtual ~CategoricalLaserProcessor();

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

  void createInvalidLaserProcessorStatus(CategoricalData::LaserProcessorStatusPtr lasStat);
  void createInvalidLaserResults(CategoricalData::LaserResultsPtr lasRes);

  /** Adds an empty status on the WM. */
  std::string addEmptyLaserProcessorStatus();

  /** Adds empty results on the WM. */
  std::string addEmptyLaserResults();

  /** New LaserScan added to wm. */
  void newLaserScan(const cast::cdl::WorkingMemoryChange & change);

  void sendDpUpdateCommand();

  
private:

  /** Initializes the part of the recogntion system implemented in this component. */
  void initProcessing();

  /** Shuts down the recognition system. */
  void finishProcessing();

  /** Performs processing of a single laser scan and produces classification results. */
  void processLaserScan(Laser::Scan2d &scan,
                        CategoricalData::LaserProcessorStatusPtr laserProcessorStatus,
                        CategoricalData::LaserResultsPtr laserResults, bool &wasError);


private:

  /** SVM Model. */
  svm_model* _svmModel;
  /** SVM Model. */
  svm_model* _sizeSvmModel;

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
  std::list<CategoricalData::LaserScanPtr> _scanQueue;

  pthread_cond_t _dataSignalCond;
  pthread_mutex_t _dataSignalMutex;


private:

  /** Name of the config file group. */
  const std::string _cfgGroup;

  /** Path to the feature config file. */
  std::string _featureFilePath;

  /** Path to the scale config file. */
  std::string _scaleFilePath;

  /** Path to the model file*/
  std::string _modelFilePath;
  std::string _sizeModelFilePath;

  int _svmOaoAlg;
  int _svmOaaAlg;
  int _svmMeasure;

  int _delay;

  double _confidenceThreshold;

  /** List of labels and names. */
  categorical::LabelFile _labels;
  categorical::LabelFile _sizeLabels;

  bool _useVision;
  bool _useSize;
};


#endif

