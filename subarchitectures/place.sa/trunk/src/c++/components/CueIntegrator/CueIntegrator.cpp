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
 * \file CueIntegrator.cpp
 * \author Andrzej Pronobis
 * \date 2008-09-02
 */

// Place.SA
#include "CueIntegrator.h"
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
#include <iostream>
#include <fstream>

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
    return new PlaceCueIntegrator();
  }
}


// ------------------------------------------------------
PlaceCueIntegrator::PlaceCueIntegrator(): _cfgGroup("CueIntegrator")
{
  _laserQueue.clear();
  _visualQueue.clear();

  pthread_cond_init(&_dataSignalCond, 0);
  pthread_mutex_init(&_dataSignalMutex, 0);

  _svmModel=0;
}


// ------------------------------------------------------
PlaceCueIntegrator::~PlaceCueIntegrator()
{
  pthread_cond_destroy(&_dataSignalCond);
  pthread_mutex_destroy(&_dataSignalMutex);
}


// ------------------------------------------------------
void PlaceCueIntegrator::configure(const map<string,string> &config)
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
bool PlaceCueIntegrator::readScaleConfigFile()
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
  _scales = vector<Scale>(100);
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
void PlaceCueIntegrator::start()
{
  // Add change filters
  addChangeFilter(createLocalTypeFilter<PlaceData::LaserResults>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceCueIntegrator>(this,
          &PlaceCueIntegrator::newLaserResults));
  addChangeFilter(createLocalTypeFilter<PlaceData::VisualResults>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceCueIntegrator>(this,
          &PlaceCueIntegrator::newVisualResults));

  // Add empty objects first
  _integratedResultsId = addEmptyIntegratedResults();

  // Initialize the system
  initProcessing();

  log("Started!");
}


// ------------------------------------------------------
void PlaceCueIntegrator::stop()
{
  // Shut down the system
  finishProcessing();

  log("Stop!");
}


// ------------------------------------------------------
void PlaceCueIntegrator::createInvalidIntegratedResults(PlaceData::IntegratedResultsPtr intgRes)
{
  intgRes->status = PlaceData::DsInvalid;
  intgRes->frameNo = -1;
  intgRes->scanRealTimeStamp.s=0;
  intgRes->scanRealTimeStamp.us=0;
  intgRes->scanWmTimeStamp.s=0;
  intgRes->scanWmTimeStamp.us=0;
  intgRes->imageRealTimeStamp.s=0;
  intgRes->imageRealTimeStamp.us=0;
  intgRes->imageWmTimeStamp.s=0;
  intgRes->imageWmTimeStamp.us=0;
  intgRes->usedVision=false;
  intgRes->usedLaser=false;
  intgRes->confident=false;
  intgRes->hypFindAlg=-1;
}


// ------------------------------------------------------
std::string PlaceCueIntegrator::addEmptyIntegratedResults()
{
  PlaceData::IntegratedResultsPtr intgRes = new PlaceData::IntegratedResults();

  createInvalidIntegratedResults(intgRes);

  string dataId = newDataID();
  addToWorkingMemory(dataId, intgRes);
  log("Empty IntegratedResults placed in the WM.");

  return dataId;
}


// ------------------------------------------------------
void PlaceCueIntegrator::newLaserResults(const cast::cdl::WorkingMemoryChange & change)
{
  pthread_mutex_lock(&_dataSignalMutex);

  // Grab data
  sched_yield();
  shared_ptr<CASTData<PlaceData::LaserResults> > resCast =
      getWorkingMemoryEntry<PlaceData::LaserResults>(change.address);
  if (!resCast)
    throw(CASTException(exceptionMessage(__HERE__, "Cannot get LaserResults from WM!")));
  PlaceData::LaserResultsPtr res=resCast->getData();

  log("Received new LaserResults.");

  // Add to queue
  _laserQueue.push_back(new PlaceData::LaserResults(*res));

  sched_yield();

  // Signal
  pthread_cond_signal(&_dataSignalCond);
  pthread_mutex_unlock(&_dataSignalMutex);
}


// ------------------------------------------------------
void PlaceCueIntegrator::newVisualResults(const cast::cdl::WorkingMemoryChange & change)
{
  pthread_mutex_lock(&_dataSignalMutex);

  // Grab data
  sched_yield();
  shared_ptr<CASTData<PlaceData::VisualResults> > resCast =
      getWorkingMemoryEntry<PlaceData::VisualResults>(change.address);
  if (!resCast)
    throw(CASTException(exceptionMessage(__HERE__, "Cannot get VisualResults from WM!")));
  PlaceData::VisualResultsPtr res=resCast->getData();

  log("Received new VisualResults.");

  // Addd to queue
  _visualQueue.push_back(new PlaceData::VisualResults(*res));

  sched_yield();

  // Signal
  pthread_cond_signal(&_dataSignalCond);
  pthread_mutex_unlock(&_dataSignalMutex);
}


// ------------------------------------------------------
bool PlaceCueIntegrator::readyToIntegrate()
{
  if ( (_visualQueue.empty()) || (_laserQueue.empty()) )
    return false;

  if (_visualQueue.front()->frameNo != _laserQueue.front()->frameNo)
  {
    log("Frame numbers of laser and visual results do not match (yet) %d %d.", _visualQueue.front()->frameNo, _laserQueue.front()->frameNo);
    return false;
  }

  return true;
}


// ------------------------------------------------------
void PlaceCueIntegrator::runComponent()
{
  log("Running...");

  // Run component
  while(isRunning())
  {
    // Get current time and add 1 sec
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 1;

    // Wait if necessary
    pthread_mutex_lock(&_dataSignalMutex);
    if (!readyToIntegrate())
      pthread_cond_timedwait(&_dataSignalCond, &_dataSignalMutex, &ts);

    // Handle signal if signal arrived
    if ((!isRunning()) || (!readyToIntegrate()))
      pthread_mutex_unlock(&_dataSignalMutex);
    else
    {
      PlaceData::LaserResultsPtr laserRes = _laserQueue.front();
      PlaceData::VisualResultsPtr visualRes = _visualQueue.front();
      _laserQueue.pop_front();
      _visualQueue.pop_front();
      pthread_mutex_unlock(&_dataSignalMutex);

      sched_yield();

      // Create the output data
      PlaceData::IntegratedResultsPtr intgRes = new PlaceData::IntegratedResults;

      // Integrate
      integrate(visualRes, laserRes, intgRes);

      // Fill in the remaining fields
      intgRes->frameNo=laserRes->frameNo;
      intgRes->scanRealTimeStamp=laserRes->scanRealTimeStamp;
      intgRes->scanWmTimeStamp=laserRes->scanWmTimeStamp;
      intgRes->imageRealTimeStamp=visualRes->imageRealTimeStamp;
      intgRes->imageWmTimeStamp=visualRes->imageWmTimeStamp;

      sched_yield();

      // Overwrite the outputs on the WM
      overwriteWorkingMemory(_integratedResultsId, intgRes);

      sched_yield();

    } // if
  } // while

  log("Completed!");
}


void PlaceCueIntegrator::finishProcessing()
{
  if (_svmModel)
    svm_destroy_model(_svmModel);
}


void PlaceCueIntegrator::initProcessing()
{
  // Load model
  println("Loading model...");
  if( (_svmModel = svm_load_model(_modelFilePath.c_str())) == 0 )
  {
    throw(CASTException(exceptionMessage(__HERE__, "Unable to load model file '%s'!", _modelFilePath.c_str())));
  }

  println("Done!");
}


void PlaceCueIntegrator::integrate(const PlaceData::VisualResultsPtr visualRes,
                                   const PlaceData::LaserResultsPtr laserRes,
                                   PlaceData::IntegratedResultsPtr intgRes)
{
  // Check if we have any valid data
  if ( (visualRes->status==PlaceData::DsInvalid) && (laserRes->status==PlaceData::DsInvalid) )
  {
    createInvalidIntegratedResults(intgRes);
    return;
  }

  // Whatever we do now, it will be valid
  intgRes->status = PlaceData::DsValid;

  // Check if we do integration at all or we just return classification for one modality
  if ((visualRes->status==PlaceData::DsValid) && (laserRes->status==PlaceData::DsInvalid))
  { // No laser, take from vision
    intgRes->usedVision=true;
    intgRes->usedLaser=false;
    intgRes->outputs = visualRes->outputs;
    intgRes->hypFindAlg=visualRes->hypFindAlg;
    intgRes->multiclassAlg=visualRes->multiclassAlg;
    intgRes->results = visualRes->results;
    intgRes->confidenceThreshold = visualRes->confidenceThreshold;
    intgRes->confident = visualRes->confident;

    return;
  }
  if ((laserRes->status==PlaceData::DsValid) && (visualRes->status==PlaceData::DsInvalid))
  { // No vision, take from laser
    intgRes->usedVision=false;
    intgRes->usedLaser=true;
    intgRes->outputs = laserRes->outputs;
    intgRes->hypFindAlg=laserRes->hypFindAlg;
    intgRes->multiclassAlg=laserRes->multiclassAlg;
    intgRes->results = laserRes->results;
    intgRes->confidenceThreshold = laserRes->confidenceThreshold;
    intgRes->confident = laserRes->confident;

    return;
  }

  log("Integrating...");

  // Ok, we do the real integration
  intgRes->usedVision=true;
  intgRes->usedLaser=true;

  // Concatenate outputs from laser and vision in one libsvm vector
  int visOutCount = visualRes->outputs.size();
  int lasOutCount = laserRes->outputs.size();
  int concatOutCount = visOutCount+lasOutCount;
  svm_node *libSvmFeatures = 
      reinterpret_cast<svm_node*>(malloc(sizeof(svm_node)*(concatOutCount+1)));
  for (int i=0; i<visOutCount; ++i)
  {
    libSvmFeatures[i].index = i+1;
    libSvmFeatures[i].value = ((visualRes->outputs[i].value-_scales[i].min)*2.0)/(_scales[i].max-_scales[i].min) - 1.0;
  }
  for (int i=0; i<lasOutCount; ++i)
  {
    libSvmFeatures[i+visOutCount].index = i+visOutCount+1;
    libSvmFeatures[i+visOutCount].value =
        ((laserRes->outputs[i].value-_scales[i+visOutCount].min)*2.0)/
        (_scales[i+visOutCount].max-_scales[i+visOutCount].min) - 1.0;
  }

  libSvmFeatures[concatOutCount].index=-1;
  libSvmFeatures[concatOutCount].value=0.0;

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

  debug("Vision+laser integrated: %d - %s", classes[0], _labels.labelNoToName(classes[0]).c_str());

  // Copy outcomes to IDL structs
  intgRes->results.resize(nrClass);
  intgRes->outputs.resize(nrHyp);
  for (int i=0; i<nrClass; ++i)
  {
    intgRes->results[i].classNo=classes[i];
    intgRes->results[i].className=_labels.labelNoToName(classes[i]).c_str();
    intgRes->results[i].confidence=confidence[i];
  }
  for (int i=0; i<nrHyp; ++i)
  {
    string outName;
    if (outputLabels2[i]<0)
      outName=lexical_cast<string>(outputLabels1[i]);
    else
      outName=lexical_cast<string>(outputLabels1[i])+"_"+lexical_cast<string>(outputLabels2[i]);
    intgRes->outputs[i].name=outName.c_str();
    intgRes->outputs[i].value=outputs[i];
  }

  // Confidence
  intgRes->confidenceThreshold = _confidenceThreshold;
  if (confidence[0]<_confidenceThreshold)
    intgRes->confident=false;
  else
    intgRes->confident=true;

  // Set the algorithms
  if (_svmModel->param.svm_type==ONE_AGAINST_ALL)
  { // OAA
    intgRes->hypFindAlg=_svmOaaAlg;
    intgRes->multiclassAlg=PlaceData::SmaOaA;
  }
  else
  { // OAO
    intgRes->hypFindAlg=_svmOaoAlg;
    intgRes->multiclassAlg=PlaceData::SmaOaO;
  }

  // Clean up
  delete classes;
  delete confidence;
  delete outputs;
  delete outputLabels1;
  delete outputLabels2;
  free(libSvmFeatures);
}

