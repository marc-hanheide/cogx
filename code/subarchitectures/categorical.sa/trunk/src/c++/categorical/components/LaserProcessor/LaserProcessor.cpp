/**
 * CategoricalLaserProcessor class.
 * \file LaserProcessor.cpp
 * \author Andrzej Pronobis
 */

// Categorical.SA
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
using namespace categorical;
using boost::bad_lexical_cast;
using boost::lexical_cast;
using namespace boost;

// ------------------------------------------------------
extern "C"
{
  cast::interfaces::CASTComponentPtr newComponent()
  {
    return new CategoricalLaserProcessor();
  }
}



// ------------------------------------------------------
CategoricalLaserProcessor::CategoricalLaserProcessor(): _cfgGroup("LaserProcessor")
{
  _scanQueue.clear();

  pthread_cond_init(&_dataSignalCond, 0);
  pthread_mutex_init(&_dataSignalMutex, 0);

  _svmModel=0;
  _sizeSvmModel=0;

  _featureInfoList = new dyntab_featuresInfo(2);
  _rangeExample = new RangeExampleGeneral();
}


// ------------------------------------------------------
CategoricalLaserProcessor::~CategoricalLaserProcessor()
{
  pthread_cond_destroy(&_dataSignalCond);
  pthread_mutex_destroy(&_dataSignalMutex);

  delete _featureInfoList;
  delete _rangeExample;
}


// ------------------------------------------------------
void CategoricalLaserProcessor::configure(const map<string,string> &config)
{
  // Read cmd line options
  map<string, string>::const_iterator it = config.find("--config");
  string configFile;
  if (it!=config.end())
    configFile = it->second;
  string labelFile;
  string sizeLabelFile;

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
    sizeLabelFile=cf.getStrValue(_cfgGroup, "SizeLabelFile", "");

    _useVision=cf.getBoolValue("DataProvider", "UseVision", true);
    _useSize=cf.getBoolValue(_cfgGroup, "UseSize", false);
    _delay=cf.getIntValue(_cfgGroup, "Delay", 0);

    _featureFilePath=cf.getStrValue(_cfgGroup, "FeatureConfigFile", "");
    _scaleFilePath=cf.getStrValue(_cfgGroup, "ScaleConfigFile", "");

    _modelFilePath=cf.getStrValue(_cfgGroup, "ModelFile", "");
    _sizeModelFilePath=cf.getStrValue(_cfgGroup, "SizeModelFile", "");
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

  if (_useSize)
  {
	  if (sizeLabelFile.empty())
	  {
	    throw(CASTException(exceptionMessage(__HERE__, "No size label file provided! Use the SizeLabelFile option in the config file!")));
	  }
	  else
	  {
	    if (!_sizeLabels.read(sizeLabelFile))
	      throw(CASTException(exceptionMessage(__HERE__, "Unable to load the size label file '%s'!", sizeLabelFile.c_str())));
	  }
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
bool CategoricalLaserProcessor::readScaleConfigFile()
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
bool CategoricalLaserProcessor::readLaserFeatureConfigFile()
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
void CategoricalLaserProcessor::start()
{
  // Add change filters
  addChangeFilter(createLocalTypeFilter<CategoricalData::LaserScan>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<CategoricalLaserProcessor>(this,
          &CategoricalLaserProcessor::newLaserScan));

  // Add empty objects first
  _laserProcessorStatusId = addEmptyLaserProcessorStatus();
  _laserResultsId = addEmptyLaserResults();

  // Initialize the system
  initProcessing();

  debug("Started!");
}


// ------------------------------------------------------
void CategoricalLaserProcessor::stop()
{
  // Shut down the system
  finishProcessing();

  debug("Stop!");
}


// ------------------------------------------------------
void CategoricalLaserProcessor::createInvalidLaserResults(CategoricalData::LaserResultsPtr lasRes)
{
  lasRes->status = CategoricalData::DsInvalid;
  lasRes->frameNo = -1;
  lasRes->scanRealTimeStamp.s=0;
  lasRes->scanRealTimeStamp.us=0;
  lasRes->scanWmTimeStamp.s=0;
  lasRes->scanWmTimeStamp.us=0;
  lasRes->multiclassAlg=CategoricalData::SmaOaO;
  lasRes->hypFindAlg=-1;
  lasRes->confidenceThreshold = 0.0;
  lasRes->confident = false;
  lasRes->useSize = _useSize;
}


// ------------------------------------------------------
std::string CategoricalLaserProcessor::addEmptyLaserResults()
{
  CategoricalData::LaserResultsPtr lasRes = new CategoricalData::LaserResults();

  createInvalidLaserResults(lasRes);

  string dataId = newDataID();
  addToWorkingMemory(dataId, lasRes);
  debug("Empty LaserResults placed in the WM.");

  return dataId;
}


// ------------------------------------------------------
void CategoricalLaserProcessor::createInvalidLaserProcessorStatus(CategoricalData::LaserProcessorStatusPtr lasStat)
{
  lasStat->status = CategoricalData::DsInvalid;
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
std::string CategoricalLaserProcessor::addEmptyLaserProcessorStatus()
{
  CategoricalData::LaserProcessorStatusPtr lasStat = new CategoricalData::LaserProcessorStatus();

  createInvalidLaserProcessorStatus(lasStat);

  string dataId = newDataID();
  addToWorkingMemory(dataId, lasStat);
  debug("Empty LaserProcessorStatus placed in the WM.");

  return dataId;
}


// ------------------------------------------------------
void CategoricalLaserProcessor::newLaserScan(const cast::cdl::WorkingMemoryChange & change)
{
  pthread_mutex_lock(&_dataSignalMutex);

  // Grab data
  sched_yield();

  shared_ptr<CASTData<CategoricalData::LaserScan> > scanCast =
    getWorkingMemoryEntry<CategoricalData::LaserScan>(change.address);
  if (!scanCast)
    throw(CASTException(exceptionMessage(__HERE__, "Cannot get scan from WM!")));
  const CategoricalData::LaserScanPtr scan = scanCast->getData();

  // Addd to queue
  _scanQueue.push_back(new CategoricalData::LaserScan(*scan));

  sched_yield();

  // Signal
  pthread_cond_signal(&_dataSignalCond);
  pthread_mutex_unlock(&_dataSignalMutex);
}


// ------------------------------------------------------
void CategoricalLaserProcessor::sendDpUpdateCommand()
{
  CategoricalData::DataProviderCommandPtr dataProviderCommand = new CategoricalData::DataProviderCommand;
  dataProviderCommand->cmd = CategoricalData::DpCmdUpdate;
  string dataId = newDataID();
  addToWorkingMemory(dataId, dataProviderCommand);
  debug("Sent DP_CMD_UPDATE command.");
}


// ------------------------------------------------------
void CategoricalLaserProcessor::runComponent()
{
  debug("Running...");

  // Send first DP Update
  if (!_useVision)
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
    if (_scanQueue.empty())
      pthread_cond_timedwait(&_dataSignalCond, &_dataSignalMutex, &ts);

    // Handle signal if signal arrived
    if ((!isRunning()) || (_scanQueue.empty()))
      pthread_mutex_unlock(&_dataSignalMutex);
    else
    {
      CategoricalData::LaserScanPtr scan = _scanQueue.front();
      _scanQueue.pop_front();
      pthread_mutex_unlock(&_dataSignalMutex);

      // Send update
      if (!_useVision)
         sendDpUpdateCommand();

      sched_yield();

      // Create the output data
      CategoricalData::LaserResultsPtr laserResults = new CategoricalData::LaserResults;
      CategoricalData::LaserProcessorStatusPtr laserProcessorStatus = new CategoricalData::LaserProcessorStatus;

      bool wasError=false;
      if (scan->status==CategoricalData::DsValid)
      {
        // Run all the feature extraction and classification
        processLaserScan(scan->scanBuffer, laserProcessorStatus, laserResults, wasError);
        // Set remaining fields
        laserResults->status=CategoricalData::DsValid;
        laserResults->frameNo=scan->frameNo;
        laserResults->scanRealTimeStamp=scan->realTimeStamp;
        laserResults->scanWmTimeStamp=scan->wmTimeStamp;
        laserProcessorStatus->status=CategoricalData::DsValid;
        laserProcessorStatus->frameNo=scan->frameNo;
        laserProcessorStatus->scanRealTimeStamp=scan->realTimeStamp;
        laserProcessorStatus->scanWmTimeStamp=scan->wmTimeStamp;
      }

      if ((scan->status!=CategoricalData::DsValid) || (wasError))
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

      if (_delay>0)
    	  usleep(_delay);

      // Overwrite the outputs on the WM
      overwriteWorkingMemory(_laserResultsId, laserResults);
      overwriteWorkingMemory(_laserProcessorStatusId, laserProcessorStatus);

      sched_yield();

    } // if
  } // while

  debug("Completed!");
}


void CategoricalLaserProcessor::finishProcessing()
{
  if (_svmModel)
    svm_destroy_model(_svmModel);

  if (_sizeSvmModel)
    svm_destroy_model(_sizeSvmModel);
}


void CategoricalLaserProcessor::initProcessing()
{
  // Load model
  println("Loading shape model...");
  if( (_svmModel = svm_load_model(_modelFilePath.c_str())) == 0 )
  {
    throw(CASTException(exceptionMessage(__HERE__, "Unable to load shape model file '%s'!", _modelFilePath.c_str() )));
  }

  // Load model
  if (_useSize)
  {
	  println("Loading size model...");
	  if( (_sizeSvmModel = svm_load_model(_sizeModelFilePath.c_str())) == 0 )
	  {
		throw(CASTException(exceptionMessage(__HERE__, "Unable to load size model file '%s'!", _sizeModelFilePath.c_str() )));
	  }
  }

  println("Done!");
}


void CategoricalLaserProcessor::processLaserScan(Laser::Scan2d &scan,
                                           CategoricalData::LaserProcessorStatusPtr laserProcessorStatus,
                                           CategoricalData::LaserResultsPtr laserResults, bool &wasError)
{

  debug("Processing scan...");

  // Convert the scan to the SICK format
  double *ranges = &(scan.ranges[0]);
  int rangesCount = scan.ranges.size();

  // Get processing start timestamp
  laserProcessorStatus->processingStartTimeStamp = getCASTTime();

  // Extract features
  _rangeExample->clean();
  if (_rangeExample->setRanges(rangesCount, ranges)<0)
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


  {
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
	    if (isnan(confidence[i]))
	      wasError=true;
	  }
	  for (int i=0; i<nrHyp; ++i)
	  {
	    string outName;
	    if (outputLabels2[i]<0)
	      outName=lexical_cast<string>(_labels.labelNoToName(outputLabels1[i]));
	    else
	      outName=lexical_cast<string>(_labels.labelNoToName(outputLabels1[i]))+"_"+
	      lexical_cast<string>(_labels.labelNoToName(outputLabels2[i]));
	    laserResults->outputs[i].name=outName.c_str();
	    laserResults->outputs[i].value=outputs[i];
	    if (isnan(outputs[i]))
	      wasError=true;
	  }

	  // Confidence
	  laserResults->confidenceThreshold = _confidenceThreshold;
	  if (confidence[0]<_confidenceThreshold)
	    laserResults->confident=false;
	  else
	    laserResults->confident=true;

	  free(classes);
	  free(confidence);
	  free(outputs);
	  free(outputLabels1);
	  free(outputLabels2);

  }

  laserResults->useSize = false;
  if (_useSize)
  {
	  laserResults->useSize = true;

	  // Setup prediction for size
	  predict_parameter predParam;
	  predParam.oaaAlg=_svmOaaAlg;
	  predParam.oaoAlg=_svmOaoAlg;
	  predParam.measure=_svmMeasure;
	  int nrClass = _sizeSvmModel->nr_class;
	  int nrHyp = _sizeSvmModel->nr_hyp;
	  double v;
	  int *classes = (int*)malloc(sizeof(int)*nrClass);
	  double *confidence = (double*)malloc(sizeof(double)*nrClass);
	  double *outputs = (double*)malloc(sizeof(double)*nrHyp);
	  int *outputLabels1 = (int*)malloc(sizeof(int)*nrHyp);
	  int *outputLabels2 = (int*)malloc(sizeof(int)*nrHyp);
	  int outputCount=0;

	  svm_predict(&_sizeSvmModel, &libSvmFeatures,
				  0 /*f_values*/, &predParam, 1 /*target*/,
				  &v, classes, confidence, outputs,
				  outputLabels1, outputLabels2, &outputCount );
	  //println("%f", confidence[0]);

	  debug("Laser scan classified as %d - %s", classes[0], _sizeLabels.labelNoToName(classes[0]).c_str());

	  // Copy outcomes to IDL structs
	  laserResults->sizeResults.resize(nrClass);
	  laserResults->sizeOutputs.resize(nrHyp);
	  for (int i=0; i<nrClass; ++i)
	  {
	    laserResults->sizeResults[i].classNo=classes[i];
	    laserResults->sizeResults[i].className=_sizeLabels.labelNoToName(classes[i]).c_str();
	    laserResults->sizeResults[i].confidence=confidence[i];
	    if (isnan(confidence[i]))
	      wasError=true;
	  }
	  for (int i=0; i<nrHyp; ++i)
	  {
	    string outName;
	    if (outputLabels2[i]<0)
	      outName=lexical_cast<string>(_sizeLabels.labelNoToName(outputLabels1[i]));
	    else
	      outName=lexical_cast<string>(_sizeLabels.labelNoToName(outputLabels1[i]))+"_"+
	      lexical_cast<string>(_sizeLabels.labelNoToName(outputLabels2[i]));
	    laserResults->sizeOutputs[i].name=outName.c_str();
	    laserResults->sizeOutputs[i].value=outputs[i];
	    if (isnan(outputs[i]))
	      wasError=true;
	  }

	  // Confidence
	  laserResults->confidenceThreshold = _confidenceThreshold;
	  if (confidence[0]<_confidenceThreshold)
	    laserResults->sizeConfident=false;
	  else
	    laserResults->sizeConfident=true;

	  free(classes);
	  free(confidence);
	  free(outputs);
	  free(outputLabels1);
	  free(outputLabels2);
  }


  // Set the algorithms
  if (_svmModel->param.svm_type==ONE_AGAINST_ALL)
  { // OAA
    laserResults->hypFindAlg=_svmOaaAlg;
    laserResults->multiclassAlg=CategoricalData::SmaOaA;
  }
  else
  { // OAO
    laserResults->hypFindAlg=_svmOaoAlg;
    laserResults->multiclassAlg=CategoricalData::SmaOaO;
  }

  // Clean up
  free(libSvmFeatures);
}






