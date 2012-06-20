/**
 * CategoricalVisualProcessor class.
 * \file VisualProcessor.cpp
 * \author Andrzej Pronobis
 */

// Categorical.SA
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
using namespace categorical;
using boost::bad_lexical_cast;
using boost::lexical_cast;
using boost::shared_ptr;

// ------------------------------------------------------
extern "C"
{
  cast::interfaces::CASTComponentPtr newComponent()
  {
    return new CategoricalVisualProcessor();
  }
}



// ------------------------------------------------------
CategoricalVisualProcessor::CategoricalVisualProcessor(): _cfgGroup("VisualProcessor")
{
  _imageQueue.clear();

  pthread_cond_init(&_dataSignalCond, 0);
  pthread_mutex_init(&_dataSignalMutex, 0);

  _svmModel=0;
  _crfhSystem=0;
}


// ------------------------------------------------------
CategoricalVisualProcessor::~CategoricalVisualProcessor()
{
  pthread_cond_destroy(&_dataSignalCond);
  pthread_mutex_destroy(&_dataSignalMutex);
}


// ------------------------------------------------------
void CategoricalVisualProcessor::configure(const map<string,string> &config)
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
    labelFile=cf.getStrValue(_cfgGroup, "LabelFile", "");

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
void CategoricalVisualProcessor::start()
{
  // Add change filters
  addChangeFilter(createLocalTypeFilter<CategoricalData::Image>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<CategoricalVisualProcessor>(this,
          &CategoricalVisualProcessor::newImage));

  // Add empty objects first
  _visualProcessorStatusId = addEmptyVisualProcessorStatus();
  _visualResultsId = addEmptyVisualResults();

  // Initialize the system
  initProcessing();

  debug("Started!");
}


// ------------------------------------------------------
void CategoricalVisualProcessor::stop()
{
  // Shut down the system
  finishProcessing();

  debug("Stop!");
}


// ------------------------------------------------------
void CategoricalVisualProcessor::createInvalidVisualResults(CategoricalData::VisualResultsPtr visRes)
{
  visRes->status = CategoricalData::DsInvalid;
  visRes->frameNo = -1;
  visRes->imageRealTimeStamp.s=0;
  visRes->imageRealTimeStamp.us=0;
  visRes->imageWmTimeStamp.s=0;
  visRes->imageWmTimeStamp.us=0;
  visRes->multiclassAlg=CategoricalData::SmaOaO;
  visRes->hypFindAlg=-1;
  visRes->confidenceThreshold = 0.0;
  visRes->confident = false;
}


// ------------------------------------------------------
std::string CategoricalVisualProcessor::addEmptyVisualResults()
{
  CategoricalData::VisualResultsPtr visRes = new CategoricalData::VisualResults();

  createInvalidVisualResults(visRes);

  string dataId = newDataID();
  addToWorkingMemory(dataId, visRes);
  debug("Empty VisualResults placed in the WM.");

  return dataId;
}


// ------------------------------------------------------
void CategoricalVisualProcessor::createInvalidVisualProcessorStatus(CategoricalData::VisualProcessorStatusPtr visStat)
{
  visStat->status = CategoricalData::DsInvalid;
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
std::string CategoricalVisualProcessor::addEmptyVisualProcessorStatus()
{
  CategoricalData::VisualProcessorStatusPtr visStat = new CategoricalData::VisualProcessorStatus();

  createInvalidVisualProcessorStatus(visStat);

  string dataId = newDataID();
  addToWorkingMemory(dataId, visStat);
  debug("Empty VisualProcessorStatus placed in the WM.");

  return dataId;
}


// ------------------------------------------------------
void CategoricalVisualProcessor::newImage(const cast::cdl::WorkingMemoryChange & change)
{
  pthread_mutex_lock(&_dataSignalMutex);

  // Grab data
  sched_yield();
  shared_ptr<CASTData<CategoricalData::Image> > imageCast =
      getWorkingMemoryEntry<CategoricalData::Image>(change.address);
  if (!imageCast)
    throw(CASTException(exceptionMessage(__HERE__, "Cannot get image from WM!")));
  const CategoricalData::ImagePtr image=imageCast->getData();

  // Addd to queue
  CopiedImage ci;
  ci.valid=(image->status==CategoricalData::DsValid);
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
void CategoricalVisualProcessor::sendDpUpdateCommand()
{
  CategoricalData::DataProviderCommandPtr dataProviderCommand = new CategoricalData::DataProviderCommand;
  dataProviderCommand->cmd = CategoricalData::DpCmdUpdate;
  string dataId = newDataID();
  addToWorkingMemory(dataId, dataProviderCommand);
  debug("Sent DpCmdUpdate command.");
}


// ------------------------------------------------------
void CategoricalVisualProcessor::runComponent()
{
  debug("Running...");

  // Send first DP Update
  sendDpUpdateCommand();

  // Run component
  while(isRunning())
  {
// Get current time and add 1sec
timeval tv;
timespec ts;
gettimeofday(&tv, NULL);
ts.tv_sec = tv.tv_sec + 1;
ts.tv_nsec = 0;

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

      // Send update
      sendDpUpdateCommand();

      sched_yield();

      // Create the output data
      CategoricalData::VisualResultsPtr visualResults = new CategoricalData::VisualResults;
      CategoricalData::VisualProcessorStatusPtr visualProcessorStatus = new CategoricalData::VisualProcessorStatus;

      if (ci.valid)
      {
        // Run all the feature extraction and classification
        processImage(*(ci.image), visualProcessorStatus, visualResults);
        // Set remaining fields
        visualResults->status=CategoricalData::DsValid;
        visualResults->frameNo=ci.frameNo;
        visualResults->imageRealTimeStamp=ci.realTimeStamp;
        visualResults->imageWmTimeStamp=ci.wmTimeStamp;
        visualProcessorStatus->status=CategoricalData::DsValid;
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


void CategoricalVisualProcessor::finishProcessing()
{
  if (_crfhSystem)
    delete _crfhSystem;
  if (_svmModel)
    svm_destroy_model(_svmModel);
}


void CategoricalVisualProcessor::initProcessing()
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


void CategoricalVisualProcessor::processImage(const CImage &image,
                                        CategoricalData::VisualProcessorStatusPtr visualProcessorStatus,
                                        CategoricalData::VisualResultsPtr visualResults)
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
      outName=lexical_cast<string>(_labels.labelNoToName(outputLabels1[i]));
    else
      outName=lexical_cast<string>(_labels.labelNoToName(outputLabels1[i]))+"_"+lexical_cast<string>(_labels.labelNoToName(outputLabels2[i]));
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
    visualResults->multiclassAlg=CategoricalData::SmaOaA;
  }
  else
  { // OAO
    visualResults->hypFindAlg=_svmOaoAlg;
    visualResults->multiclassAlg=CategoricalData::SmaOaO;
  }

  // Clean up
  free(classes);
  free(confidence);
  free(outputs);
  free(outputLabels1);
  free(outputLabels2);
  free(libSvmCrfh);
}

