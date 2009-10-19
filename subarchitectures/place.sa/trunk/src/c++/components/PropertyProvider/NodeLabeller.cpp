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
 * PlaceNodeLabeller class.
 * \file NodeLabeller.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-31
 */

// Place.SA
#include "NodeLabeller.h"
#include "OutputsCache.h"
#include "ConfidenceEstimator.h"
#include "place/shared/ConfigFile.h"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>
// Boost
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
// Std

using namespace std;
using namespace cast;
using namespace place;
using namespace boost;

// ------------------------------------------------------
extern "C"
{
  FrameworkProcess* newComponent(const string &_id)
  {
    return new PlaceNodeLabeller(_id);
  }
}


// ------------------------------------------------------
PlaceNodeLabeller::PlaceNodeLabeller(const string &_id):
                        WorkingMemoryAttachedComponent(_id),
                        ManagedProcess(_id),
                        _cfgGroup("NodeLabeller")
{
  _receivedResultsQueue.clear();

  pthread_cond_init(&_dataSignalCond, 0);
  pthread_mutex_init(&_dataSignalMutex, 0);
  pthread_mutex_init(&_intrDataMutex, 0);

  // Set the data to the "empty" sate
  _data.lastNodeId=-1;
  _data.currentNode.nodeId=-1;
  _data.currentNode.gateway=false;
  _data.currentNode.nodeOutputCount=0;
  _data.currentNode.areaOutputCount=0;
}


// ------------------------------------------------------
PlaceNodeLabeller::~PlaceNodeLabeller()
{
  pthread_cond_destroy(&_dataSignalCond);
  pthread_mutex_destroy(&_dataSignalMutex);
  pthread_mutex_destroy(&_intrDataMutex);

  // Delete the caches
  delete _nodeCache;
  delete _areaCache;
  delete _confEstimator;
}


// ------------------------------------------------------
void PlaceNodeLabeller::configure(map<string,string> &_config)
{
  ManagedProcess::configure(_config);

  // Read cmd line options
  string configFile=_config["--config"];
  string labelFile;

  // Read config file
  ConfigFile cf;
  if (configFile.empty())
    println("No configuration file provided. Using defaults instead.");
  else
  {
    if (!cf.read(configFile))
      throw CASTException(__HERE__, "Unable to load configuration file '%s'!", configFile.c_str());
  }

  // Get configuration from the file
  try
  {
    labelFile=cf.getStrValue("Common", "LabelFile", "");

    _confidenceThreshold=cf.getDoubleValue(_cfgGroup, "ConfidenceThreshold", 0.1);
    _posBinSize=cf.getDoubleValue(_cfgGroup, "PositionBinSize", 0.2);
    _headBinSize=cf.getDoubleValue(_cfgGroup, "HeadingBinSize", 0.1);
    _maxPriorOutputsCount=cf.getIntValue(_cfgGroup, "MaxPriorOutputsCount", 50);
    _accumulateOnlyConfident=cf.getBoolValue(_cfgGroup, "AccumulateOnlyConfident", true);
    _cachePoseDecay=cf.getIntValue(_cfgGroup, "CachePoseDecay", true);
  }
  catch(bad_lexical_cast &)
  {
    throw CASTException(__HERE__, "Incorrect item value in the config file '%s'!", configFile.c_str());
  }

  // Read label file
  if (labelFile.empty())
  {
    throw CASTException(__HERE__, "No label file provided! Use the LabelFile option in the config file!");
  }
  else
  {
    if (!_labels.read(labelFile))
      throw CASTException(__HERE__, "Unable to load the label file '%s'!", labelFile.c_str());

  }

  // Print the configuration
  log("Configuration:");
  log("-> Confidence threshold: %f", _confidenceThreshold);
  log("-> Position cache bin size: %f", _posBinSize);
  log("-> Heading cache bin size: %f", _headBinSize);
  log("-> Max. prior outputs count: %d", _maxPriorOutputsCount);
  log("-> Only confident outputs will be accumulated: %s", ((_accumulateOnlyConfident)?"yes":"no"));
  log("-> No. of traveled cache bins after which pose information is forgotten: %d", _cachePoseDecay);
}


// ------------------------------------------------------
void PlaceNodeLabeller::start()
{
  ManagedProcess::start();

  // Add change filters
  MemberFunctionChangeReceiver<PlaceNodeLabeller> * pReceiver =
      new MemberFunctionChangeReceiver<PlaceNodeLabeller>(this, &PlaceNodeLabeller::newIntegratedResults);
  addChangeFilter( createLocalTypeFilter<PlaceData::IntegratedResults>(cdl::OVERWRITE),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PlaceNodeLabeller>(this, &PlaceNodeLabeller::newOdometry);
  addChangeFilter( createLocalTypeFilter<PlaceData::Odometry>(cdl::OVERWRITE),
                   pReceiver );


  // Add empty NodeLabellerData first
  PlaceData::NodeLabellerData *dataCopy = new PlaceData::NodeLabellerData(_data);
  _nodeLabellerDataId = newDataID();
  addToWorkingMemory<PlaceData::NodeLabellerData>(_nodeLabellerDataId, dataCopy);
  log("Empty NodeLabellerData placed in the WM.");

  // Lock the entry so that we don't get consistency exceptions
  lockEntry(_nodeLabellerDataId, cdl::LOCKED_O);

  // Create caches
  _nodeCache = new OutputsCache(_posBinSize, _headBinSize);
  _areaCache = new OutputsCache(_posBinSize, _headBinSize);

  // Create confidence estimator
  _confEstimator = new ConfidenceEstimator(&_labels);

  log("Started!");
}


// ------------------------------------------------------
void PlaceNodeLabeller::stop()
{
  ManagedProcess::stop();
  log("Stop!");
}


// ------------------------------------------------------
void PlaceNodeLabeller::newIntegratedResults(const cast::cdl::WorkingMemoryChange & change)
{
  pthread_mutex_lock(&_dataSignalMutex);

  // Grab data
  sched_yield();
  boost::shared_ptr<const CASTTypedData<PlaceData::IntegratedResults> > resTD =
      getWorkingMemoryEntry<PlaceData::IntegratedResults>(change.m_address);
  if (!resTD)
    throw CASTException(__HERE__, "Cannot get results from WM!");
  const boost::shared_ptr<const PlaceData::IntegratedResults> res=resTD->getData();

  // Addd to queue
  _receivedResultsQueue.push_back(new PlaceData::IntegratedResults(*res));

  // Signal
  pthread_cond_signal(&_dataSignalCond);
  pthread_mutex_unlock(&_dataSignalMutex);
}


// ------------------------------------------------------
void PlaceNodeLabeller::newOdometry(const cast::cdl::WorkingMemoryChange & change)
{
  pthread_mutex_lock(&_dataSignalMutex);

  // Grab data
  sched_yield();
  boost::shared_ptr<const CASTTypedData<PlaceData::Odometry> > odomTD =
      getWorkingMemoryEntry<PlaceData::Odometry>(change.m_address);
  if (!odomTD)
    throw CASTException(__HERE__, "Cannot get odometry from WM!");
  const boost::shared_ptr<const PlaceData::Odometry> odom=odomTD->getData();

  // Addd to queue
  _receivedOdomQueue.push_back(new PlaceData::Odometry(*odom));

  // Signal
  pthread_cond_signal(&_dataSignalCond);
  pthread_mutex_unlock(&_dataSignalMutex);
}


// ------------------------------------------------------
bool PlaceNodeLabeller::readyToAccumulate()
{
  if ( (_receivedResultsQueue.empty()) || (_receivedOdomQueue.empty()) )
    return false;

  if (_receivedResultsQueue.front()->frameNo != _receivedOdomQueue.front()->frameNo)
    return false;

  return true;
}


// ------------------------------------------------------
void PlaceNodeLabeller::runComponent()
{
  log("Running...");

  // Run component
  while(m_status == STATUS_RUN)
  {
    // Get current time and add 1 sec
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 1;

    // Wait if necessary
    pthread_mutex_lock(&_dataSignalMutex);
    if (!readyToAccumulate())
      pthread_cond_timedwait(&_dataSignalCond, &_dataSignalMutex, &ts);

    // Handle signal if signal arrived
    if ((m_status!=STATUS_RUN) || (!readyToAccumulate()))
      pthread_mutex_unlock(&_dataSignalMutex);
    else
    {
      PlaceData::IntegratedResults *intgResults = _receivedResultsQueue.front();
      PlaceData::Odometry *odom = _receivedOdomQueue.front();
      _receivedResultsQueue.pop_front();
      _receivedOdomQueue.pop_front();
      pthread_mutex_unlock(&_dataSignalMutex);

      debug("Grabbed integrated results (confident=%d) and matching odometry. Accumulating...", ((intgResults->confident)?1:0) );

      sched_yield();

      if ( (intgResults->status==PlaceData::DS_VALID) && (odom->status==PlaceData::DS_VALID) )
      {
        if ( (!_accumulateOnlyConfident) || (intgResults->confident) )
        {
          // Lock the internal data
          pthread_mutex_lock(&_intrDataMutex);

          // Remember the algorithms used for obtaining the outputs
          _multiclassAlg = intgResults->multiclassAlg;
          _hypFindAlg = intgResults->hypFindAlg;

          // Add the outputs to the caches
          _nodeCache->addOutputs(odom->odometryBuffer.m_x, odom->odometryBuffer.m_y, 
                                odom->odometryBuffer.m_theta, intgResults->outputs);
          _areaCache->addOutputs(odom->odometryBuffer.m_x, odom->odometryBuffer.m_y, 
                                odom->odometryBuffer.m_theta, intgResults->outputs);

          // Get the accumulated outputs
          unsigned int nodeOutputCount;
          unsigned int areaOutputCount;
          _nodeCache->getAccumulatedOutputs(_data.currentNode.nodeAccumulatedOutputs, nodeOutputCount);
          _areaCache->getAccumulatedOutputs(_data.currentNode.areaAccumulatedOutputs, areaOutputCount);
          _data.currentNode.nodeOutputCount=nodeOutputCount;
          _data.currentNode.areaOutputCount=areaOutputCount;

          // Get results
          _confEstimator->getResultsForOutputs(_data.currentNode.nodeResults,
                                                    _data.currentNode.nodeAccumulatedOutputs, 
                                                    _data.currentNode.nodeOutputCount, 
                                                    _multiclassAlg, _hypFindAlg);
          _confEstimator->getResultsForOutputs(_data.currentNode.areaResults,
                                                    _data.currentNode.areaAccumulatedOutputs, 
                                                    _data.currentNode.areaOutputCount,
                                                    _multiclassAlg, _hypFindAlg);

          // Fill in missing data just in case
          _data.currentNode.nodeId=-1;
          _data.currentNode.gateway=false;

          // Make a copy to send to WM
          PlaceData::NodeLabellerData *dataCopy = new PlaceData::NodeLabellerData(_data);

          // Print for debugging.
          /*
          _areaCache->print();
          _areaCache->printAccumulatedOutputs();
          _areaCache->printNormalizedAccumulatedOutputs();
          */

          // Unock the internal data
          pthread_mutex_unlock(&_intrDataMutex);

          // Send to WM
          overwriteWorkingMemory<PlaceData::NodeLabellerData>(_nodeLabellerDataId, dataCopy, cdl::BLOCKING);
        } // confident
      } // valid

      sched_yield();

      // Delete the data
      delete intgResults;
      delete odom;

    } // if
  } // while

  log("Completed!");
}


// ------------------------------------------------------
void PlaceNodeLabeller::receivePullQuery(const FrameworkQuery & query,
                                         FrameworkLocalData<PlaceData::NodePlaceInfo>*& data)
{
  string arguments=query.getQuery();
  int nodeId;
  PlaceData::NodeLabellerQueryType queryType;
  bool gateway;

  // Start with invalid info
  PlaceData::NodePlaceInfo *nodePlaceInfo =  new PlaceData::NodePlaceInfo();
  nodePlaceInfo->nodeId=-1;
  nodePlaceInfo->nodeClassNo=-1;
  nodePlaceInfo->areaClassNo=-1;

  // Parse the arguments
  if (!parseQueryArguments(arguments, nodeId, queryType, gateway))
  {
    data =  new FrameworkLocalData<PlaceData::NodePlaceInfo>(getProcessIdentifier(), nodePlaceInfo);
    return;
  }

  // Debug info
  println("Received pull query (%d, %d, %d)!", nodeId, queryType, (int)gateway);

  // Lock the internal data
  pthread_mutex_lock(&_intrDataMutex);

  // Handle query
  if (queryType == PlaceData::NL_QT_INFO)
  {
    handleInfoQuery(nodeId, nodePlaceInfo);
  }
  else if (queryType == PlaceData::NL_QT_NEW)
  {
    handleNewQuery(nodeId, gateway, nodePlaceInfo);
  }
  else if (queryType == PlaceData::NL_QT_UPDATE)
  {
    handleUpdateQuery(nodeId, gateway, nodePlaceInfo);
  }

  // Make a copy of the updated data
  PlaceData::NodeLabellerData *dataCopy = new PlaceData::NodeLabellerData(_data);

   // Print for debugging.
  /*
  if (!_nodeCache->isEmpty())
  {
    _nodeCache->print();
    _nodeCache->printAccumulatedOutputs();
    _nodeCache->printNormalizedAccumulatedOutputs();
  }
  */

  // Unlock the internal data
  pthread_mutex_unlock(&_intrDataMutex);

  // Return the NodePlaceInfo
  data =  new FrameworkLocalData<PlaceData::NodePlaceInfo>(getProcessIdentifier(), nodePlaceInfo);

  // Sent the updated _data to the WM
  overwriteWorkingMemory<PlaceData::NodeLabellerData>(_nodeLabellerDataId, dataCopy, cdl::BLOCKING);
}


// ------------------------------------------------------
bool PlaceNodeLabeller::parseQueryArguments(std::string argStr, int &nodeId,
                                       PlaceData::NodeLabellerQueryType &queryType,
                                       bool &gateway)
{
  try
  {
    trim(argStr);
    int queryTypeInt;
    vector<string> tokens;
    split(tokens, argStr, is_any_of(" \t") );
    if (tokens.size()!=3)
    {
      throw bad_lexical_cast();
    }
    for (int i=0; i<3; ++i)
    {
      string token=tokens[i];
      trim(token);
      if (starts_with(token, "node_id="))
      {
        iterator_range<string::iterator> i=find_first(token, "=");
        nodeId=lexical_cast<int>(string(i.end(), token.end()));
        if (nodeId<0)
          throw bad_lexical_cast();
      }
      else if  (starts_with(token, "gateway="))
      {
        iterator_range<string::iterator> i=find_first(token, "=");
        gateway=lexical_cast<bool>(string(i.end(), token.end()));
      }
      else if  (starts_with(token, "query_type="))
      {
        iterator_range<string::iterator> i=find_first(token, "=");
        queryTypeInt=lexical_cast<int>(string(i.end(), token.end()));
        if ( (queryTypeInt!=PlaceData::NL_QT_INFO) &&
              (queryTypeInt!=PlaceData::NL_QT_NEW) && 
              (queryTypeInt!=PlaceData::NL_QT_UPDATE) )
        {
          throw bad_lexical_cast();
        }
        queryType=(PlaceData::NodeLabellerQueryType)queryTypeInt;
      }
      else
      {
        throw bad_lexical_cast();
      }
    }
  }
  catch (...)
  {
    return false;
  }

  return true;
}


// ------------------------------------------------------
void PlaceNodeLabeller::handleInfoQuery(int nodeId, PlaceData::NodePlaceInfo *nodePlaceInfo)
{
  // Setup the NodePlaceInfo to unknown class
  nodePlaceInfo->nodeId=nodeId;
  nodePlaceInfo->nodeClassNo=PlaceData::UNKNOWN_CLASS_NO;
  nodePlaceInfo->areaClassNo=PlaceData::UNKNOWN_CLASS_NO;
  nodePlaceInfo->nodeClassName=PlaceData::UNKNOWN_CLASS_NAME;
  nodePlaceInfo->areaClassName=PlaceData::UNKNOWN_CLASS_NAME;

  // Find NodeInfo for that node
  PlaceData::NodeInfo *nodeInfo=0;
  for (unsigned int i=0; i<_data.nodes.length(); ++i)
  {
    if (_data.nodes[i].nodeId==nodeId)
    {
      nodeInfo=&(_data.nodes[i]);
      break;
    }
  }

  // Check if such node is in the data
  if (nodeInfo>0)
  {
    // Check if any results are there
    if (nodeInfo->nodeOutputCount>0)
    {
      if (nodeInfo->nodeResults[0].confidence>=_confidenceThreshold)
      {
        nodePlaceInfo->nodeClassNo=nodeInfo->nodeResults[0].classNo;
        nodePlaceInfo->nodeClassName=nodeInfo->nodeResults[0].className;
      }
    }
    if (nodeInfo->areaOutputCount>0)
    {
      if (nodeInfo->areaResults[0].confidence>=_confidenceThreshold)
      {
        nodePlaceInfo->areaClassNo=nodeInfo->areaResults[0].classNo;
        nodePlaceInfo->areaClassName=nodeInfo->areaResults[0].className;
      }
    }
  }
}


// ------------------------------------------------------
void PlaceNodeLabeller::integrateWithPrior(PlaceData::ClassifierOutputs &priorOutputs, unsigned int &priorOutputsCount, const PlaceData::ClassifierOutputs &newOutputs, unsigned int newOutputsCount)
{
  // If there is nothing in the newOutputs, leave the prior as it was.
  if (newOutputsCount==0)
  {
    return;
  }

  // If there is nothing in the prior, just replace it
  if (priorOutputsCount==0)
  {
    priorOutputs=newOutputs;
    priorOutputsCount=newOutputsCount;
    return;
  }

  // Check if the prior and the outputs match, if not, raise exception
  int priorLength = priorOutputs.length();
  int newLength = newOutputs.length();
  if (priorLength != newLength)
    throw CASTException(__HERE__, "Outputs cannot be integrated with the prior as they do not match (%d, %d)!", 
                        priorLength, newLength);

  //
  double priorOutputsCountOrig = priorOutputsCount;
  if (priorOutputsCount>_maxPriorOutputsCount)
  {
    priorOutputsCount=_maxPriorOutputsCount;
  }

  // Now accumulate and check
  for (int i=0; i<priorLength; ++i)
  {
    if (string(priorOutputs[i].name) != string(newOutputs[i].name))
    {
      throw CASTException(__HERE__, "Outputs cannot be integrated with the prior as they do not match (%s, %s)!", 
                          string(priorOutputs[i].name).c_str(), string(newOutputs[i].name).c_str());
    }
    priorOutputs[i].value/=priorOutputsCountOrig;
    priorOutputs[i].value*=static_cast<double>(priorOutputsCount);
    priorOutputs[i].value+=newOutputs[i].value;
  }
  priorOutputsCount+=newOutputsCount;
}


// ------------------------------------------------------
void PlaceNodeLabeller::handleNewQuery(int nodeId, bool gateway, PlaceData::NodePlaceInfo *nodePlaceInfo)
{
  debug("Handling NEW query.");

  PlaceData::NodeInfo *currentNodeInfo=&(_data.currentNode);

  // Setup the NodePlaceInfo to unknown class
  nodePlaceInfo->nodeId=nodeId;
  nodePlaceInfo->nodeClassNo=PlaceData::UNKNOWN_CLASS_NO;
  nodePlaceInfo->areaClassNo=PlaceData::UNKNOWN_CLASS_NO;
  nodePlaceInfo->nodeClassName=PlaceData::UNKNOWN_CLASS_NAME;
  nodePlaceInfo->areaClassName=PlaceData::UNKNOWN_CLASS_NAME;

  // Set nodeId 
  nodePlaceInfo->nodeId=nodeId;

  // Check if we have some prior about this node
  PlaceData::NodeInfo *nodeInfo=0;
  for (unsigned int i=0; i<_data.nodes.length(); ++i)
  {
    if (_data.nodes[i].nodeId==nodeId)
    {
      nodeInfo=&(_data.nodes[i]);
      break;
    }
  }

  if (!nodeInfo)
  { // No, we don't
    // Add new NodeInfo to nodes
    _data.nodes.length(_data.nodes.length()+1);
    nodeInfo=&(_data.nodes[_data.nodes.length()-1]);
    nodeInfo->nodeId=nodeId;
    nodeInfo->gateway=gateway;
    nodeInfo->nodeOutputCount=0;
    nodeInfo->areaOutputCount=0;
  }

  // THIS IS HERE TO MAKE THE CODE ROBUST TO BUGS IN NAV.SA
  nodeInfo->gateway = gateway;

  // Check if the gateway status matches, if not, the update should have been used!!!
  if (nodeInfo->gateway != gateway)
    throw CASTException(__HERE__, "The gateway flag in the NodeLabeller's data is different than in the received query! Use NL_QT_UPDATE instead!");

  // Integrate the current node info with the prior in .nodes
  unsigned int nodeInfoNodeOutputCount=nodeInfo->nodeOutputCount;
  unsigned int nodeInfoAreaOutputCount=nodeInfo->areaOutputCount;
  integrateWithPrior(nodeInfo->nodeAccumulatedOutputs, nodeInfoNodeOutputCount,
                     currentNodeInfo->nodeAccumulatedOutputs, currentNodeInfo->nodeOutputCount);
  integrateWithPrior(nodeInfo->areaAccumulatedOutputs, nodeInfoAreaOutputCount,
                     currentNodeInfo->areaAccumulatedOutputs, currentNodeInfo->areaOutputCount);
  nodeInfo->nodeOutputCount=nodeInfoNodeOutputCount;
  nodeInfo->areaOutputCount=nodeInfoAreaOutputCount;

  // Re-calculate the results
  _confEstimator->getResultsForOutputs(nodeInfo->nodeResults,
                                            nodeInfo->nodeAccumulatedOutputs,
                                            nodeInfo->nodeOutputCount,
                                            _multiclassAlg, _hypFindAlg);
  _confEstimator->getResultsForOutputs(nodeInfo->areaResults,
                                            nodeInfo->areaAccumulatedOutputs,
                                            nodeInfo->areaOutputCount,
                                            _multiclassAlg, _hypFindAlg);

  // Set the last node index
  _data.lastNodeId=nodeId;

  // Set the NodePlaceInfo based on nodeInfo
  if (nodeInfo->nodeOutputCount>0)
  {
    if (nodeInfo->nodeResults[0].confidence>=_confidenceThreshold)
    {
      nodePlaceInfo->nodeClassNo=nodeInfo->nodeResults[0].classNo;
      nodePlaceInfo->nodeClassName=nodeInfo->nodeResults[0].className;
    }
  }
  if (nodeInfo->areaOutputCount>0)
  {
    if (nodeInfo->areaResults[0].confidence>=_confidenceThreshold)
    {
      nodePlaceInfo->areaClassNo=nodeInfo->areaResults[0].classNo;
      nodePlaceInfo->areaClassName=nodeInfo->areaResults[0].className;
    }
  }

  // Clean some caches
  _nodeCache->clear();
  currentNodeInfo->nodeAccumulatedOutputs = PlaceData::ClassifierOutputs();
  currentNodeInfo->nodeResults = PlaceData::ClassifierResults();
  currentNodeInfo->nodeOutputCount=0;
  if (gateway)
  {
    _areaCache->clear();
    currentNodeInfo->areaAccumulatedOutputs = PlaceData::ClassifierOutputs();
    currentNodeInfo->areaResults = PlaceData::ClassifierResults();
    currentNodeInfo->areaOutputCount=0;
  }
}


// ------------------------------------------------------
void PlaceNodeLabeller::handleUpdateQuery(int nodeId, bool gateway, PlaceData::NodePlaceInfo *nodePlaceInfo)
{
  debug("Handling UPDATE query.");

  // Setup the NodePlaceInfo to unknown class
  nodePlaceInfo->nodeId=nodeId;
  nodePlaceInfo->nodeClassNo=PlaceData::UNKNOWN_CLASS_NO;
  nodePlaceInfo->areaClassNo=PlaceData::UNKNOWN_CLASS_NO;
  nodePlaceInfo->nodeClassName=PlaceData::UNKNOWN_CLASS_NAME;
  nodePlaceInfo->areaClassName=PlaceData::UNKNOWN_CLASS_NAME;

  // Check if we have a node like that in the data
    // Check if we have some prior about this node
  PlaceData::NodeInfo *nodeInfo=0;
  for (unsigned int i=0; i<_data.nodes.length(); ++i)
  {
    if (_data.nodes[i].nodeId==nodeId)
    {
      nodeInfo=&(_data.nodes[i]);
      break;
    }
  }

  // If a node stopped being a gateway, do nothing
  if ( !gateway )
    return;


  // If we don't have the node in the data yet, behave as the NEW query was sent
  if (!nodeInfo)
  {
    handleNewQuery(nodeId, gateway, nodePlaceInfo);
    return;
  }

  // If we have it, and the gateway status did not change, do nothing
  if (nodeInfo->gateway==gateway) 
    return;

  // So, only cases when an existing node became a gateway
  nodeInfo->gateway=true;
  handleNewQuery(nodeId, gateway, nodePlaceInfo);
}



