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
 * \file VisualProcessor.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-16
 */

// Place.SA
#include "VisualProcessor.h"
#include "shared/ConfigFile.h"
#include "tools/libsvm-mod/src/svm.h"
#include "tools/libsvm-mod/src/predict.h"
#include "tools/libcrfh/src/libCRFH.h"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>
// Boost
#include <boost/lexical_cast.hpp>
// Std

using namespace std;
using namespace cast;
using namespace place;
using boost::bad_lexical_cast;
using boost::lexical_cast;
using boost::shared_ptr;

// ------------------------------------------------------
extern "C"
{
  cast::interfaces::CASTComponentPtr newComponent()
  {
    return new PlaceVisualProcessor();
  }
}



// ------------------------------------------------------
PlaceVisualProcessor::PlaceVisualProcessor(): _cfgGroup("VisualProcessor")
{
  _imageQueue.clear();
  _updateOnRead=false;

  pthread_cond_init(&_dataSignalCond, 0);
  pthread_mutex_init(&_dataSignalMutex, 0);

  _svmModel=0;
  _crfhSystem=0;
}


// ------------------------------------------------------
PlaceVisualProcessor::~PlaceVisualProcessor()
{
  pthread_cond_destroy(&_dataSignalCond);
  pthread_mutex_destroy(&_dataSignalMutex);
}


// ------------------------------------------------------
void PlaceVisualProcessor::configure(const map<string,string> &config)
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

    _crfhDescriptors=cf.getStrValue(_cfgGroup, "CrfhDescriptors", "Lxx(4,28)+Lxy(4,28)+Lyy(4,28)+Lxx(64,28)+Lxy(64,28)+Lyy(64,28)");
    _crfhFiltering=cf.getBoolValue(_cfgGroup, "CrfhSmallValueFiltering", false);
    _crfhFiltThreshold=cf.getDoubleValue(_cfgGroup, "CrfhFilteringThreshold", 0.0001);
    _crfhSkipBorderPixels=cf.getIntValue(_cfgGroup, "CrfhSkipBorderPixels", 15);
    _modelFilePath=cf.getStrValue(_cfgGroup, "ModelFile", "");
    _svmOaoAlg=cf.getIntValue(_cfgGroup, "SvmOaoAlg", 1);
    _svmOaaAlg=cf.getIntValue(_cfgGroup, "SvmOaaAlg", 1);
    _svmMeasure=cf.getIntValue(_cfgGroup, "SvmMeasure", 1);
    _confidenceThreshold=cf.getDoubleValue(_cfgGroup, "ConfidenceThreshold", 0.1);
    _delay=cf.getIntValue(_cfgGroup, "Delay", 0);
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

  // Print the configuration
  log("Configuration:");
  log("-> CRFH descriptors: %s", _crfhDescriptors.c_str());
  log("-> CRFH small value filtering: %s", (_crfhFiltering)?"on":"off");
  log("-> CRFH filtering threshold: %f", _crfhFiltThreshold);
  log("-> CRFH skip border pixels: %d", _crfhSkipBorderPixels);
  log("-> Model file: %s", _modelFilePath.c_str());
  log("-> SVM OaO algorithm: %d", _svmOaoAlg);
  log("-> SVM OaA algorithm: %d", _svmOaaAlg);
  log("-> SVM distance measure: %d", _svmMeasure);
  log("-> Confidence threshold: %f", _confidenceThreshold);

}


// ------------------------------------------------------
void PlaceVisualProcessor::start()
{
  // Add change filters
  addChangeFilter(createLocalTypeFilter<PlaceData::Image>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceVisualProcessor>(this,
          &PlaceVisualProcessor::newImage));

  addChangeFilter(createLocalTypeFilter<PlaceData::VisualProcessorCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<PlaceVisualProcessor>(this,
          &PlaceVisualProcessor::newVisualProcessorCommand));

  // Add empty objects first
  _visualProcessorCommandAckId = addEmptyVisualProcessorCommandAck();
  _visualProcessorStatusId = addEmptyVisualProcessorStatus();
  _visualResultsId = addEmptyVisualResults();

  // Initialize the system
  initProcessing();

  debug("Started!");
}


// ------------------------------------------------------
void PlaceVisualProcessor::stop()
{
  // Shut down the system
  finishProcessing();

  debug("Stop!");
}


// ------------------------------------------------------
std::string PlaceVisualProcessor::addEmptyVisualProcessorCommandAck()
{
  PlaceData::VisualProcessorCommandAckPtr cmdAck = new PlaceData::VisualProcessorCommandAck();
  PlaceData::VisualProcessorCommandPtr cmd = new PlaceData::VisualProcessorCommand();
  cmd->cmd=PlaceData::VpCmdInvalid;
  cmdAck->cmd=cmd;
  cmdAck->src="";

  string dataId = newDataID();
  addToWorkingMemory(dataId, cmdAck);
  debug("Empty VisualProcessorCommandAck placed in the WM.");

  return dataId;
}


// ------------------------------------------------------
void PlaceVisualProcessor::createInvalidVisualResults(PlaceData::VisualResultsPtr visRes)
{
  visRes->status = PlaceData::DsInvalid;
  visRes->frameNo = -1;
  visRes->imageRealTimeStamp.s=0;
  visRes->imageRealTimeStamp.us=0;
  visRes->imageWmTimeStamp.s=0;
  visRes->imageWmTimeStamp.us=0;
  visRes->multiclassAlg=PlaceData::SmaOaO;
  visRes->hypFindAlg=-1;
  visRes->confidenceThreshold = 0.0;
  visRes->confident = false;
}


// ------------------------------------------------------
std::string PlaceVisualProcessor::addEmptyVisualResults()
{
  PlaceData::VisualResultsPtr visRes = new PlaceData::VisualResults();

  createInvalidVisualResults(visRes);

  string dataId = newDataID();
  addToWorkingMemory(dataId, visRes);
  debug("Empty VisualResults placed in the WM.");

  return dataId;
}


// ------------------------------------------------------
void PlaceVisualProcessor::createInvalidVisualProcessorStatus(PlaceData::VisualProcessorStatusPtr visStat)
{
  visStat->status = PlaceData::DsInvalid;
  visStat->frameNo = -1;
  visStat->imageRealTimeStamp.s=0;
  visStat->imageRealTimeStamp.us=0;
  visStat->imageWmTimeStamp.s=0;
  visStat->imageWmTimeStamp.us=0;
  visStat->processingStartTimeStamp.s=0;
  visStat->processingStartTimeStamp.us=0;
  visStat->extractionEndTimeStamp.s=0;
  visStat->extractionEndTimeStamp.us=0;
  visStat->classificationEndTimeStamp.s=0;
  visStat->classificationEndTimeStamp.us=0;
}


// ------------------------------------------------------
std::string PlaceVisualProcessor::addEmptyVisualProcessorStatus()
{
  PlaceData::VisualProcessorStatusPtr visStat = new PlaceData::VisualProcessorStatus();

  createInvalidVisualProcessorStatus(visStat);

  string dataId = newDataID();
  addToWorkingMemory(dataId, visStat);
  debug("Empty VisualProcessorStatus placed in the WM.");

  return dataId;
}


// ------------------------------------------------------
void PlaceVisualProcessor::newVisualProcessorCommand(const cast::cdl::WorkingMemoryChange & change)
{
  // Get the command and remove from WM
  shared_ptr<CASTData<PlaceData::VisualProcessorCommand> > commandCast =
      getWorkingMemoryEntry<PlaceData::VisualProcessorCommand>(change.address);
  if (!commandCast)
    return;
  PlaceData::VisualProcessorCommandPtr command =
      new PlaceData::VisualProcessorCommand(*commandCast->getData());
  deleteFromWorkingMemory(change.address);

  // Send the ack
  PlaceData::VisualProcessorCommandAckPtr commandAck = new PlaceData::VisualProcessorCommandAck;
  commandAck->cmd = command;
  commandAck->src = change.src;
  overwriteWorkingMemory(_visualProcessorCommandAckId, commandAck);

  // Execute command
  if (command->cmd==PlaceData::VpCmdUpdateOnReadStart)
  {
    debug("Received command VpCmdUpdateOnReadStart.");
    _updateOnRead=true;
  }
  else if (command->cmd==PlaceData::VpCmdUpdateStop)
  {
    debug("Received command VpCmdUpdateStop.");
    _updateOnRead=false;
  }
  else
  {
    throw(CASTException(exceptionMessage(__HERE__, "Unknown command!")));
  }
}


// ------------------------------------------------------
void PlaceVisualProcessor::newImage(const cast::cdl::WorkingMemoryChange & change)
{
  pthread_mutex_lock(&_dataSignalMutex);

  // Grab data
  sched_yield();
  shared_ptr<CASTData<PlaceData::Image> > imageCast =
      getWorkingMemoryEntry<PlaceData::Image>(change.address);
  if (!imageCast)
    throw(CASTException(exceptionMessage(__HERE__, "Cannot get image from WM!")));
  const PlaceData::ImagePtr image=imageCast->getData();

  // Addd to queue
  CopiedImage ci;
  ci.valid=(image->status==PlaceData::DsValid);
  ci.frameNo=image->frameNo;
  ci.realTimeStamp=image->realTimeStamp;
  ci.wmTimeStamp=image->wmTimeStamp;
  ci.image = new CImage(const_cast<char *>(reinterpret_cast<const char *>(&(image->imageBuffer.data[0]))),
                        image->imageBuffer.width,
                        image->imageBuffer.height, IT_BYTE);
  _imageQueue.push_back(ci);

  sched_yield();

  // Signal
  pthread_cond_signal(&_dataSignalCond);
  pthread_mutex_unlock(&_dataSignalMutex);
}


// ------------------------------------------------------
void PlaceVisualProcessor::sendDpUpdateCommand()
{
  PlaceData::DataProviderCommandPtr dataProviderCommand = new PlaceData::DataProviderCommand;
  dataProviderCommand->cmd = PlaceData::DpCmdUpdate;
  string dataId = newDataID();
  addToWorkingMemory(dataId, dataProviderCommand);
  debug("Sent DpCmdUpdate command.");
}


// ------------------------------------------------------
void PlaceVisualProcessor::runComponent()
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
    if (_imageQueue.empty())
      pthread_cond_timedwait(&_dataSignalCond, &_dataSignalMutex, &ts);

    // Handle signal if signal arrived
    if ((!isRunning()) || (_imageQueue.empty()))
      pthread_mutex_unlock(&_dataSignalMutex);
    else
    {
      CopiedImage ci = _imageQueue.front();
      _imageQueue.pop_front();
      pthread_mutex_unlock(&_dataSignalMutex);

      // Send update if asked to do so
      if (_updateOnRead)
      {
        sendDpUpdateCommand();
      }

      sched_yield();

      // Create the output data
      PlaceData::VisualResultsPtr visualResults = new PlaceData::VisualResults;
      PlaceData::VisualProcessorStatusPtr visualProcessorStatus = new PlaceData::VisualProcessorStatus;

      if (ci.valid)
      {
        // Run all the feature extraction and classification
        processImage(*(ci.image), visualProcessorStatus, visualResults);
        // Set remaining fields
        visualResults->status=PlaceData::DsValid;
        visualResults->frameNo=ci.frameNo;
        visualResults->imageRealTimeStamp=ci.realTimeStamp;
        visualResults->imageWmTimeStamp=ci.wmTimeStamp;
        visualProcessorStatus->status=PlaceData::DsValid;
        visualProcessorStatus->frameNo=ci.frameNo;
        visualProcessorStatus->imageRealTimeStamp=ci.realTimeStamp;
        visualProcessorStatus->imageWmTimeStamp=ci.wmTimeStamp;
      }
      else
      {
        // Create empty, invalid outputs
        createInvalidVisualResults(visualResults);
        createInvalidVisualProcessorStatus(visualProcessorStatus);
        // Set the frameNo
        visualResults->frameNo=ci.frameNo;
        visualProcessorStatus->frameNo=ci.frameNo;
        // Wait a short while not to overwrite the results before someone reads them
        usleep(50000);
      }

      sched_yield();

      // Overwrite the outputs on the WM
      overwriteWorkingMemory(_visualResultsId, visualResults);
      overwriteWorkingMemory(_visualProcessorStatusId, visualProcessorStatus);

      // Delete the copy
      delete ci.image;

      // Wait some time to decrease frame rate
      if (_delay>0)
      {
        sched_yield();
        usleep(_delay);
      }

      sched_yield();

    } // if
  } // while

  debug("Completed!");
}


void PlaceVisualProcessor::finishProcessing()
{
  if (_crfhSystem)
    delete _crfhSystem;
  if (_svmModel)
    svm_destroy_model(_svmModel);
}


void PlaceVisualProcessor::initProcessing()
{
  // Load model
  println("Loading model...");
  if( (_svmModel = svm_load_model(_modelFilePath.c_str())) == 0 )
  {
    throw(CASTException(exceptionMessage(__HERE__, "Unable to load model file '%s'!", _modelFilePath.c_str())));
  }

  // Initialize the feature extractor
  println("Initializing the feature extractor...");
  _crfhSystem = new CSystem(_crfhDescriptors.c_str());

  println("Done!");
}


void PlaceVisualProcessor::processImage(const CImage &image,
                                        PlaceData::VisualProcessorStatusPtr visualProcessorStatus,
                                        PlaceData::VisualResultsPtr visualResults)
{
  debug("Processing image...");

  // Get processing start timestamp
  visualProcessorStatus->processingStartTimeStamp = getCASTTime();

  // Extract features
  CCrfh *crfh = _crfhSystem->computeHistogram(image, _crfhSkipBorderPixels);
  if (_crfhFiltering)
    crfh->filter(_crfhFiltThreshold);
  crfh->normalize();

  // Get feature extraction timestamp
  visualProcessorStatus->extractionEndTimeStamp = getCASTTime();

  // Convert the result to the libsvm format
  svm_node *libSvmCrfh = reinterpret_cast<svm_node*>(crfh->getLibSvmVector());
  delete crfh;

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

  svm_predict(&_svmModel, &libSvmCrfh,
              0 /*f_values*/, &predParam, 1 /*target*/,
              &v, classes, confidence, outputs,
              outputLabels1, outputLabels2, &outputCount );

  // Get classification timestamp
  visualProcessorStatus->classificationEndTimeStamp = getCASTTime();

  debug("Image classified as %d - %s", classes[0], _labels.labelNoToName(classes[0]).c_str());

  // Copy outcomes to IDL structs
  visualResults->results.resize(nrClass);
  visualResults->outputs.resize(nrHyp);
  for (int i=0; i<nrClass; ++i)
  {
    visualResults->results[i].classNo=classes[i];
    visualResults->results[i].className=_labels.labelNoToName(classes[i]).c_str();
    visualResults->results[i].confidence=confidence[i];
  }
  for (int i=0; i<nrHyp; ++i)
  {
    string outName;
    if (outputLabels2[i]<0)
      outName=lexical_cast<string>(outputLabels1[i]);
    else
      outName=lexical_cast<string>(outputLabels1[i])+"_"+lexical_cast<string>(outputLabels2[i]);
    visualResults->outputs[i].name=outName.c_str();
    visualResults->outputs[i].value=outputs[i];
  }

  // Confidence
  visualResults->confidenceThreshold = _confidenceThreshold;
  if (confidence[0]<_confidenceThreshold)
    visualResults->confident=false;
  else
    visualResults->confident=true;

  // Set the algorithms
  if (_svmModel->param.svm_type==ONE_AGAINST_ALL)
  { // OAA
    visualResults->hypFindAlg=_svmOaaAlg;
    visualResults->multiclassAlg=PlaceData::SmaOaA;
  }
  else
  { // OAO
    visualResults->hypFindAlg=_svmOaoAlg;
    visualResults->multiclassAlg=PlaceData::SmaOaO;
  }

  // Clean up
  delete classes;
  delete confidence;
  delete outputs;
  delete outputLabels1;
  delete outputLabels2;
  free(libSvmCrfh);
}
