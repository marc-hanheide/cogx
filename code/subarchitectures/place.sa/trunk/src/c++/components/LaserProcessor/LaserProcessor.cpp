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
 * \file LaserProcessor.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-31
 */

// Place.SA
#include "LaserProcessor.h"
#include "shared/ConfigFile.h"
#include "tools/libsvm-mod/src/svm.h"
#include "tools/libsvm-mod/src/predict.h"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>
// Boost
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
// Std
#include <exception>

using namespace std;
using namespace cast;
using namespace place;
using boost::bad_lexical_cast;
using boost::lexical_cast;
using namespace boost;

// ------------------------------------------------------
extern "C"
{
  cast::interfaces::CASTComponentPtr newComponent()
  {
    return new PlaceLaserProcessor();
  }
}



// ------------------------------------------------------
PlaceLaserProcessor::PlaceLaserProcessor(): _cfgGroup("LaserProcessor")
{
  _scanQueue.clear();
  _updateOnRead=false;

  pthread_cond_init(&_dataSignalCond, 0);
  pthread_mutex_init(&_dataSignalMutex, 0);

  _svmModel=0;

  _featureInfoList = new dyntab_featuresInfo(2);
  _rangeExample = new RangeExampleGeneral();
}


// ------------------------------------------------------
PlaceLaserProcessor::~PlaceLaserProcessor()
{
  pthread_cond_destroy(&_dataSignalCond);
  pthread_mutex_destroy(&_dataSignalMutex);

  delete _featureInfoList;
  delete _rangeExample;
}


// ------------------------------------------------------
void PlaceLaserProcessor::configure(const map<string,string> &config)
{
  // Read cmd line options
  map<string, string>::const_iterator it = config.find("--config");
  string configFile;
  if (it!=config.end())
    configFile = it->second;
  string labelFile;

  // Read config file
  ConfigFile cf;
  if (configFile.empty())
    println("No configuration file provided. Using defaults instead.");
  else
  {
    if (!cf.read(configFile))
      throw(CASTException(exceptionMessage(__HERE__, "Unable to load configuration file '%s'!", configFile.c_str())));
  }

  // Get configuration from the file
  try
  {
    labelFile=cf.getStrValue("Common", "LabelFile", "");

    _featureFilePath=cf.getStrValue(_cfgGroup, "FeatureConfigFile", "");
    _scaleFilePath=cf.getStrValue(_cfgGroup, "ScaleConfigFile", "");

    _modelFilePath=cf.getStrValue(_cfgGroup, "ModelFile", "");
    _svmOaoAlg=cf.getIntValue(_cfgGroup, "SvmOaoAlg", 1);
    _svmOaaAlg=cf.getIntValue(_cfgGroup, "SvmOaaAlg", 1);
    _svmMeasure=cf.getIntValue(_cfgGroup, "SvmMeasure", 1);
    _confidenceThreshold=cf.getDoubleValue(_cfgGroup, "ConfidenceThreshold", 0.1);
  }
  catch(bad_lexical_cast &)
  {
    throw(CASTException(exceptionMessage(__HERE__, "Incorrect item value in the config file '%s'!", configFile.c_str())));
  }
  if (_modelFilePath.empty())
    throw(CASTException(exceptionMessage(__HERE__, "No model file given!")));

  // Read label file
  if (labelFile.empty())
  {
    throw(CASTException(exceptionMessage(__HERE__, "No label file provided! Use the LabelFile option in the config file!")));
  }
  else
  {
    if (!_labels.read(labelFile))
      throw(CASTException(exceptionMessage(__HERE__, "Unable to load the label file '%s'!", labelFile.c_str())));
  }

  // Read feature config file
  if (_featureFilePath.empty())
  {
    throw(CASTException(exceptionMessage(__HERE__, "No laser feature config file provided! Use the FeatureConfigFile option in the config file!")));
  }
  else
  {
    if (!readLaserFeatureConfigFile())
      throw(CASTException(exceptionMessage(__HERE__, "Unable to load the laser feature config file '%s'!", _featureFilePath.c_str())));
  }

  // Check amount of features
  if (_featureInfoList->num() == 0)
    throw(CASTException(exceptionMessage(__HERE__, "No features defined in the config file!")));

  // Read scale config file
  if (_scaleFilePath.empty())
  {
    throw(CASTException(exceptionMessage(__HERE__, "No scale config file provided! Use the ScaleConfigFile option in the config file!")));
  }
  else
  {
    if (!readScaleConfigFile())
      throw(CASTException(exceptionMessage(__HERE__, "Unable to load the scale config file '%s'!", _scaleFilePath.c_str())));
  }


  // Print the configuration
  log("Configuration:");
  log("-> Model file: %s", _modelFilePath.c_str());
  log("-> SVM OaO algorithm: %d", _svmOaoAlg);
  log("-> SVM OaA algorithm: %d", _svmOaaAlg);
  log("-> SVM distance measure: %d", _svmMeasure);
  log("-> Confidence threshold: %f", _confidenceThreshold);
}


// ------------------------------------------------------
bool PlaceLaserProcessor::readScaleConfigFile()
{
  // Open the file
  ifstream f(_scaleFilePath.c_str());
  if (!f.is_open())
  {
    return false;
  }

  // Read the header
  std::string line;
  getline(f, line);
  if (line!="x")
  {
    f.close();
    return false;
  }
  getline(f, line);
  if (line!="-1 1")
  {
    f.close();
    return false;
  }

  // Create scales vector
  _scales = vector<Scale>(_featureInfoList->num());
  for (unsigned int i=0; i<_scales.size(); ++i)
  {
    _scales[i].min=0.0;
    _scales[i].max=0.0;
  }

  // Read the scales
  try
  {
    while(!f.eof())
    {
      getline(f, line);
      if (line.empty())
        continue;

      // Split line
      vector<string> tokens;
      split(tokens, line, is_any_of(" \t") );
      if (tokens.size()!=3)
      {
        cerr<<"Invalid line format!"<<endl;
        throw 1;
      }
      int featNo = lexical_cast<int>(tokens[0]);
      double min = lexical_cast<double>(tokens[1]);
      double max = lexical_cast<double>(tokens[2]);

      // Check the number
      if (featNo>_featureInfoList->num())
      {
        cerr<<"Invalid feature number!"<<endl;
        throw 1;
      }

      _scales[featNo-1].min=min;
      _scales[featNo-1].max=max;
    }
  }
  catch(...)
  {
    f.close();
    return false;
  }

  f.close();
  return true;
}


// ------------------------------------------------------
bool PlaceLaserProcessor::readLaserFeatureConfigFile()
{
  // Open the file
  FILE* f = fopen(_featureFilePath.c_str(), "r");
  if ( f == NULL )
    return false;

  // Read
  int ret = 0;
  int op = 0;
  while ( !feof(f) ) 
  {
    FeatureInfo *fi = new FeatureInfo;

    ret = fscanf(f, "%s %d", fi->name, &op);
    if ( ret != 2 ) 
      return false;

    // Read parameter
    if ( op == 1 ) 
    {
      fi->hasParam = true;
      if (fscanf(f, " %lf", &fi->param)!=1)
        return false;
      log("Using feature %s with param %lf", fi->name, fi->param);
    }
    else
      log("Using feature %s", fi->name);

    // Add to list
    _featureInfoList->add(fi);
  }

  log("Total number of features: %d", _featureInfoList->num());

  return true;
}


// ------------------------------------------------------
void PlaceLaserProcessor::start()
{
  // Add change filters
  addChangeFilter(createLocalTypeFilter<PlaceData::LaserScan>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceLaserProcessor>(this,
          &PlaceLaserProcessor::newLaserScan));

  addChangeFilter(createLocalTypeFilter<PlaceData::LaserProcessorCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<PlaceLaserProcessor>(this,
          &PlaceLaserProcessor::newLaserProcessorCommand));

  // Add empty objects first
  _laserProcessorCommandAckId = addEmptyLaserProcessorCommandAck();
  _laserProcessorStatusId = addEmptyLaserProcessorStatus();
  _laserResultsId = addEmptyLaserResults();

  // Initialize the system
  initProcessing();

  debug("Started!");
}


// ------------------------------------------------------
void PlaceLaserProcessor::stop()
{
  // Shut down the system
  finishProcessing();

  debug("Stop!");
}


// ------------------------------------------------------
std::string PlaceLaserProcessor::addEmptyLaserProcessorCommandAck()
{
  PlaceData::LaserProcessorCommandAckPtr cmdAck = new PlaceData::LaserProcessorCommandAck();
  PlaceData::LaserProcessorCommandPtr cmd = new PlaceData::LaserProcessorCommand();
  cmd->cmd=PlaceData::LpCmdInvalid;
  cmdAck->cmd=cmd;
  cmdAck->src="";

  string dataId = newDataID();
  addToWorkingMemory(dataId, cmdAck);
  debug("Empty LaserProcessorCommandAck placed in the WM.");

  return dataId;
}

// ------------------------------------------------------
void PlaceLaserProcessor::createInvalidLaserResults(PlaceData::LaserResultsPtr lasRes)
{
  lasRes->status = PlaceData::DsInvalid;
  lasRes->frameNo = -1;
  lasRes->scanRealTimeStamp.s=0;
  lasRes->scanRealTimeStamp.us=0;
  lasRes->scanWmTimeStamp.s=0;
  lasRes->scanWmTimeStamp.us=0;
  lasRes->multiclassAlg=PlaceData::SmaOaO;
  lasRes->hypFindAlg=-1;
  lasRes->confidenceThreshold = 0.0;
  lasRes->confident = false;
}


// ------------------------------------------------------
std::string PlaceLaserProcessor::addEmptyLaserResults()
{
  PlaceData::LaserResultsPtr lasRes = new PlaceData::LaserResults();

  createInvalidLaserResults(lasRes);

  string dataId = newDataID();
  addToWorkingMemory(dataId, lasRes);
  debug("Empty LaserResults placed in the WM.");

  return dataId;
}


// ------------------------------------------------------
void PlaceLaserProcessor::createInvalidLaserProcessorStatus(PlaceData::LaserProcessorStatusPtr lasStat)
{
  lasStat->status = PlaceData::DsInvalid;
  lasStat->frameNo = -1;
  lasStat->scanRealTimeStamp.s=0;
  lasStat->scanRealTimeStamp.us=0;
  lasStat->scanWmTimeStamp.s=0;
  lasStat->scanWmTimeStamp.us=0;
  lasStat->processingStartTimeStamp.s=0;
  lasStat->processingStartTimeStamp.us=0;
  lasStat->extractionEndTimeStamp.s=0;
  lasStat->extractionEndTimeStamp.us=0;
  lasStat->classificationEndTimeStamp.s=0;
  lasStat->classificationEndTimeStamp.us=0;
}


// ------------------------------------------------------
std::string PlaceLaserProcessor::addEmptyLaserProcessorStatus()
{
  PlaceData::LaserProcessorStatusPtr lasStat = new PlaceData::LaserProcessorStatus();

  createInvalidLaserProcessorStatus(lasStat);

  string dataId = newDataID();
  addToWorkingMemory(dataId, lasStat);
  debug("Empty LaserProcessorStatus placed in the WM.");

  return dataId;
}


// ------------------------------------------------------
void PlaceLaserProcessor::newLaserProcessorCommand(const cast::cdl::WorkingMemoryChange & change)
{
  // Get the command and remove from WM
  shared_ptr<CASTData<PlaceData::LaserProcessorCommand> > commandCast =
    getWorkingMemoryEntry<PlaceData::LaserProcessorCommand>(change.address);
  if (!commandCast)
    return;
  PlaceData::LaserProcessorCommandPtr command =
      new PlaceData::LaserProcessorCommand(*commandCast->getData());
  deleteFromWorkingMemory(change.address);

  // Send the ack
  PlaceData::LaserProcessorCommandAckPtr commandAck = new PlaceData::LaserProcessorCommandAck;
  commandAck->cmd = command;
  commandAck->src = change.src;
  overwriteWorkingMemory(_laserProcessorCommandAckId, commandAck);

  // Execute command
  if (command->cmd==PlaceData::LpCmdUpdateOnReadStart)
  {
    debug("Received command LpCmdUpdateOnReadStart.");
    _updateOnRead=true;
  }
  else if (command->cmd==PlaceData::LpCmdUpdateStop)
  {
    debug("Received command LpCmdUpdateStop.");
    _updateOnRead=false;
  }
  else
  {
    throw(CASTException(exceptionMessage(__HERE__, "Unknown command!")));
  }
}


// ------------------------------------------------------
void PlaceLaserProcessor::newLaserScan(const cast::cdl::WorkingMemoryChange & change)
{
  pthread_mutex_lock(&_dataSignalMutex);

  // Grab data
  sched_yield();

  shared_ptr<CASTData<PlaceData::LaserScan> > scanCast =
    getWorkingMemoryEntry<PlaceData::LaserScan>(change.address);
  if (!scanCast)
    throw(CASTException(exceptionMessage(__HERE__, "Cannot get scan from WM!")));
  const PlaceData::LaserScanPtr scan = scanCast->getData();

  // Addd to queue
  _scanQueue.push_back(new PlaceData::LaserScan(*scan));

  sched_yield();

  // Signal
  pthread_cond_signal(&_dataSignalCond);
  pthread_mutex_unlock(&_dataSignalMutex);
}


// ------------------------------------------------------
void PlaceLaserProcessor::sendDpUpdateCommand()
{
  PlaceData::DataProviderCommandPtr dataProviderCommand = new PlaceData::DataProviderCommand;
  dataProviderCommand->cmd = PlaceData::DpCmdUpdate;
  string dataId = newDataID();
  addToWorkingMemory(dataId, dataProviderCommand);
  debug("Sent DP_CMD_UPDATE command.");
}


// ------------------------------------------------------
void PlaceLaserProcessor::runComponent()
{
  debug("Running...");

  // Run component
  while(isRunning())
  {
    // Get current time and add 1 sec
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 1;

    // Wait if necessary
    pthread_mutex_lock(&_dataSignalMutex);
    if (_scanQueue.empty())
      pthread_cond_timedwait(&_dataSignalCond, &_dataSignalMutex, &ts);

    // Handle signal if signal arrived
    if ((!isRunning()) || (_scanQueue.empty()))
      pthread_mutex_unlock(&_dataSignalMutex);
    else
    {
      PlaceData::LaserScanPtr scan = _scanQueue.front();
      _scanQueue.pop_front();
      pthread_mutex_unlock(&_dataSignalMutex);

      // Send update if asked to do so
      if (_updateOnRead)
      {
        sendDpUpdateCommand();
      }

      sched_yield();

      // Create the output data
      PlaceData::LaserResultsPtr laserResults = new PlaceData::LaserResults;
      PlaceData::LaserProcessorStatusPtr laserProcessorStatus = new PlaceData::LaserProcessorStatus;

      if (scan->status==PlaceData::DsValid)
      {
        // Run all the feature extraction and classification
        processLaserScan(scan->scanBuffer, laserProcessorStatus, laserResults);
        // Set remaining fields
        laserResults->status=PlaceData::DsValid;
        laserResults->frameNo=scan->frameNo;
        laserResults->scanRealTimeStamp=scan->realTimeStamp;
        laserResults->scanWmTimeStamp=scan->wmTimeStamp;
        laserProcessorStatus->status=PlaceData::DsValid;
        laserProcessorStatus->frameNo=scan->frameNo;
        laserProcessorStatus->scanRealTimeStamp=scan->realTimeStamp;
        laserProcessorStatus->scanWmTimeStamp=scan->wmTimeStamp;
      }
      else
      {
        // Create empty, invalid outputs
        createInvalidLaserResults(laserResults);
        createInvalidLaserProcessorStatus(laserProcessorStatus);
        // Set the frameNo
        laserResults->frameNo=scan->frameNo;
        laserProcessorStatus->frameNo=scan->frameNo;
        // Wait a short while not to overwrite the results before someone reads them
        usleep(50000);
      }

      sched_yield();

      // Overwrite the outputs on the WM
      overwriteWorkingMemory(_laserResultsId, laserResults);
      overwriteWorkingMemory(_laserProcessorStatusId, laserProcessorStatus);

      sched_yield();

    } // if
  } // while

  debug("Completed!");
}


void PlaceLaserProcessor::finishProcessing()
{
  if (_svmModel)
    svm_destroy_model(_svmModel);
}


void PlaceLaserProcessor::initProcessing()
{
  // Load model
  println("Loading model...");
  if( (_svmModel = svm_load_model(_modelFilePath.c_str())) == 0 )
  {
    throw(CASTException(exceptionMessage(__HERE__, "Unable to load model file '%s'!", _modelFilePath.c_str() )));
  }

  println("Done!");
}


void PlaceLaserProcessor::processLaserScan(Laser::Scan2d &scan,
                                           PlaceData::LaserProcessorStatusPtr laserProcessorStatus,
                                           PlaceData::LaserResultsPtr laserResults)
{

  debug("Processing scan...");

  // Get processing start timestamp
  laserProcessorStatus->processingStartTimeStamp = getCASTTime();

  // Extract features
  _rangeExample->clean();
  if (_rangeExample->setRanges(scan.ranges.size(), &(scan.ranges[0]))<0)
    throw(CASTException(exceptionMessage(__HERE__, "Couldn't set ranges!")));

  classifier.calcSelectedFeatures(_rangeExample, _featureInfoList);
  int featureCount = _featureInfoList->num();

  // Get feature extraction timestamp
  laserProcessorStatus->extractionEndTimeStamp = getCASTTime();

  // Convert the result to the libsvm format AND SCALE!
  svm_node *libSvmFeatures = reinterpret_cast<svm_node*>(malloc(sizeof(svm_node)*(featureCount+1)));
  for (int i=0; i<featureCount; ++i)
  {
    FeatureInfo *fi = _featureInfoList->getElement(i);
    libSvmFeatures[i].index = i+1;
    if (_scales[i].min==_scales[i].max)
      libSvmFeatures[i].value = 0; //fi->result;
    else
      libSvmFeatures[i].value = ((fi->result-_scales[i].min)*2.0)/(_scales[i].max-_scales[i].min) - 1.0;
  }
  libSvmFeatures[featureCount].index=-1;
  libSvmFeatures[featureCount].value=0.0;

  // Yield
  sched_yield();

  // Setup prediction
  predict_parameter predParam;
  predParam.oaaAlg=_svmOaaAlg;
  predParam.oaoAlg=_svmOaoAlg;
  predParam.measure=_svmMeasure;
  int nrClass = _svmModel->nr_class;
  int nrHyp = _svmModel->nr_hyp;
  double v;
  int *classes = (int*)malloc(sizeof(int)*nrClass);
  double *confidence = (double*)malloc(sizeof(double)*nrClass);
  double *outputs = (double*)malloc(sizeof(double)*nrHyp);
  int *outputLabels1 = (int*)malloc(sizeof(int)*nrHyp);
  int *outputLabels2 = (int*)malloc(sizeof(int)*nrHyp);
  int outputCount=0;

  svm_predict(&_svmModel, &libSvmFeatures,
              0 /*f_values*/, &predParam, 1 /*target*/,
              &v, classes, confidence, outputs,
              outputLabels1, outputLabels2, &outputCount );

  // Get classification timestamp
  laserProcessorStatus->classificationEndTimeStamp = getCASTTime();

  debug("Laser scan classified as %d - %s", classes[0], _labels.labelNoToName(classes[0]).c_str());

  // Copy outcomes to IDL structs
  laserResults->results.resize(nrClass);
  laserResults->outputs.resize(nrHyp);
  for (int i=0; i<nrClass; ++i)
  {
    laserResults->results[i].classNo=classes[i];
    laserResults->results[i].className=_labels.labelNoToName(classes[i]).c_str();
    laserResults->results[i].confidence=confidence[i];
  }
  for (int i=0; i<nrHyp; ++i)
  {
    string outName;
    if (outputLabels2[i]<0)
      outName=lexical_cast<string>(outputLabels1[i]);
    else
      outName=lexical_cast<string>(outputLabels1[i])+"_"+lexical_cast<string>(outputLabels2[i]);
    laserResults->outputs[i].name=outName.c_str();
    laserResults->outputs[i].value=outputs[i];
  }

  // Confidence
  laserResults->confidenceThreshold = _confidenceThreshold;
  if (confidence[0]<_confidenceThreshold)
    laserResults->confident=false;
  else
    laserResults->confident=true;


  // Set the algorithms
  if (_svmModel->param.svm_type==ONE_AGAINST_ALL)
  { // OAA
    laserResults->hypFindAlg=_svmOaaAlg;
    laserResults->multiclassAlg=PlaceData::SmaOaA;
  }
  else
  { // OAO
    laserResults->hypFindAlg=_svmOaoAlg;
    laserResults->multiclassAlg=PlaceData::SmaOaO;
  }

  // Clean up
  delete classes;
  delete confidence;
  delete outputs;
  delete outputLabels1;
  delete outputLabels2;
  free(libSvmFeatures);
}

