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
 * The PlaceDataProvider class.
 * \file DataProvider.cpp
 * \author Andrzej Pronobis
 * \date 2008-07-16
 */

// Place.SA
#include "DataProvider.h"
#include "shared/ConfigFile.h"
#include "shared/CastTools.h"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <cast/core/CASTUtils.hpp>
// Std
#include <math.h>
#include <values.h>
#include <boost/lexical_cast.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>


using namespace std;
using namespace cast;
using namespace place;
using boost::bad_lexical_cast;
using boost::shared_ptr;


// ------------------------------------------------------
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new PlaceDataProvider();
  }
}


// ------------------------------------------------------
PlaceDataProvider::PlaceDataProvider(): _cfgGroup("DataProvider")
{
  // Initalize thread synchronization
  _wasSignal=false;
  pthread_cond_init(&_signalCond, 0);
  pthread_mutex_init(&_signalMutex, 0);

  // Mutexes
  pthread_mutex_init(&_scanQueueMutex, 0);
  pthread_mutex_init(&_odometryQueueMutex, 0);

  // Other
  _frameNo=0;
  _lastGrabTimestamp.s=0;
  _lastGrabTimestamp.us=0;
}


// ------------------------------------------------------
PlaceDataProvider::~PlaceDataProvider()
{
  pthread_cond_destroy(&_signalCond);
  pthread_mutex_destroy(&_signalMutex);
  pthread_mutex_destroy(&_scanQueueMutex);
  pthread_mutex_destroy(&_odometryQueueMutex);
}


// ------------------------------------------------------
void PlaceDataProvider::configure(const std::map<std::string,std::string> &config)
{
  // Read cmd line options
  map<string, string>::const_iterator it = config.find("--config");
  string configFile;
   if (it!=config.end())
      configFile = it->second;
  it = config.find("--load");
  string dataFile;
  if (it!=config.end())
    dataFile = it->second;
  it = config.find("--videoserver");
  string vidServ;
  if (it!=config.end())
    vidServ = it->second;
  it = config.find("--startservers");
  _startServers=(((it!=config.end()) && (it->second == "true")));


  // Read config file
  ConfigFile cf;
  if (configFile.empty())
    println("No configuration file provided. Using defaults instead.");
  else
  {
    if (!cf.read(configFile))
      throw(CASTException(exceptionMessage(__HERE__, "Unable to load configuration file '%s'!",
          configFile.c_str() )));
  }

  // Get configuration from the file
  try
  {
    _useLaser=cf.getBoolValue(_cfgGroup, "UseLaser", true);
    _useVision=cf.getBoolValue(_cfgGroup, "UseVision", true);
    _useOdometry=cf.getBoolValue(_cfgGroup, "UseOdometry", true);
    _queueTimeWindow=cf.getDoubleValue(_cfgGroup, "QueueTimeWindow", 5.0);
    _queueSize=cf.getIntValue(_cfgGroup, "QueueSize", 100);
    _cameraId=cf.getIntValue(_cfgGroup, "CameraId", 0);
    _videoServerName=cf.getStrValue(_cfgGroup, "VideoServerName", "VideoServer");
    _scanDelay=cf.getIntValue(_cfgGroup, "ScanDelay", 0);
  }
  catch(bad_lexical_cast &)
  {
    throw(CASTException(exceptionMessage(__HERE__,
        "Incorrect item value in the config file '%s'!", configFile.c_str() )));
  }

  // Check the configuration
  if ((!_useVision) && (!_useLaser) && (!_useOdometry))
    throw(CASTException(exceptionMessage(__HERE__, "There is nothing to do!" )));

  // Choose video server type
  _videoServerType=VS_STD;
  if (!vidServ.empty())
  {
    if (vidServ == "std")
      _videoServerType=VS_STD;
    else
      throw(CASTException(exceptionMessage(__HERE__,
          "Unknown video server type!" )));
  }

  // Read data file
  _loadDataFromDisk=false;
  if (!dataFile.empty())
  {
    debug("Loading data config file.");
    _loadDataFromDisk=true;

    if (!_dataReader.readDataConfigFile(dataFile))
      throw(CASTException(exceptionMessage(__HERE__,
          "Unable to load data config file '%s'!", dataFile.c_str() )));
  }

  // Print the configuration
  log("Configuration:");
  log("-> Data will be acquired from: %s", (_loadDataFromDisk)?"disk":"hardware");
  log("-> Use laser: %s", (_useLaser)?"on":"off");
  log("-> Use vision: %s", (_useVision)?"on":"off");
  log("-> Use odometry: %s", (_useOdometry)?"on":"off");
  log("-> Queue time window: %g", _queueTimeWindow);
  log("-> Queue size: %d", _queueSize);
  log("-> Camera id: %d", _cameraId);
  log("-> Video server name: %s", _videoServerName.c_str());
  log("-> Scan delay: %d", _scanDelay);

  // Start the robotbase interface
  if (_startServers)
  {
    Ice::Identity id1;
    id1.name = "RobotbaseServer";
    id1.category = "RobotbaseServer";
    getObjectAdapter()->add(&_robotbaseServer, id1);
    println("Robotbase server registered.");

    // Start the laser interface
    Ice::Identity id2;
    id2.name = "LaserServer";
    id2.category = "LaserServer";
    getObjectAdapter()->add(&_laserServer, id2);
    println("Laser server registered.");
  }
}


// ------------------------------------------------------
void PlaceDataProvider::start()
{
  // Get connection to the video server
  if ((!_loadDataFromDisk) && (_useVision))
  {
    _videoServer = getIceServer<Video::VideoInterface>(_videoServerName);
  }

  addChangeFilter(createLocalTypeFilter<PlaceData::DataProviderCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<PlaceDataProvider>(this,
          &PlaceDataProvider::newDataProviderCommandAdded) );

  debug("Started!");
}


// ------------------------------------------------------
void PlaceDataProvider::stop()
{
  debug("Stop!");
}


// ------------------------------------------------------
void PlaceDataProvider::receiveScan2d(const Laser::Scan2d &scan)
{
  if (scan.ranges.empty())
  {
    println("Empty scan, ignoring it");
    return;
  }

  debug("Received scan acquired at %d.%d.", scan.time.s, scan.time.us);

  pthread_mutex_lock(&_scanQueueMutex);

  _scanQueue.push_back(scan);
  cdl::CASTTime tDiff = _scanQueue.front().time -_scanQueue.back().time;
  while ((castTimeToSeconds(tDiff) > _queueTimeWindow) || (_scanQueue.size() > 100))
  {
    _scanQueue.pop_front();
    tDiff = _scanQueue.front().time - _scanQueue.back().time;
  }

  pthread_mutex_unlock(&_scanQueueMutex);
}


// ------------------------------------------------------
void PlaceDataProvider::receiveOdometry(const Robotbase::Odometry &odom)
{
  if (odom.odompose.empty())
  {
    println("WARNING: Odometry struct contained no odompose which is needed!");
    return;
  }

  debug("Received odometry acquired at %d.%d.", odom.time.s, odom.time.us);

  pthread_mutex_lock(&_odometryQueueMutex);

  _odometryQueue.push_back(odom);
  cdl::CASTTime tDiff = _odometryQueue.front().time - _odometryQueue.back().time;
  while((castTimeToSeconds(tDiff) > _queueTimeWindow) || (_odometryQueue.size() > 100))
  {
    _odometryQueue.pop_front();
    tDiff = _odometryQueue.front().time - _odometryQueue.back().time;
  }

  pthread_mutex_unlock(&_odometryQueueMutex);
}


// ------------------------------------------------------
Robotbase::Odometry PlaceDataProvider::RobotbaseServer::pullOdometry(const Ice::Current&)
{
  throw(CASTException(exceptionMessage(__HERE__, "Pulling odometry not implemented!")));
}


// ------------------------------------------------------
void PlaceDataProvider::RobotbaseServer::execMotionCommand(const ::Robotbase::MotionCommand& cmd,
                       const Ice::Current&)
{
  cerr << "LOL! You've just tried to move the robot while I stream from a database :-)" << endl;
}


// ------------------------------------------------------
void PlaceDataProvider::RobotbaseServer::registerOdometryPushClient(const Robotbase::OdometryPushClientPrx& clientPrx,
                                Ice::Double desiredInterval, const Ice::Current&)
{
  OdometryClient client;
  client.prx = clientPrx;
  client.interval = desiredInterval;
  _odomPushClients.push_back(client);
}


// ------------------------------------------------------
Laser::Scan2d PlaceDataProvider::LaserServer::pullScan2d(const Ice::Current&)
{
  throw(CASTException(exceptionMessage(__HERE__, "Pulling scans not implemented!")));
}


// ------------------------------------------------------
void PlaceDataProvider::LaserServer::registerScan2dPushClient(const Laser::Scan2dPushClientPrx& clientPrx,
                              Ice::Double desiredInterval, const Ice::Current&)
{
  Scan2dClient client;
  client.prx = clientPrx;
  client.interval = desiredInterval;
  _scanPushClients.push_back(client);
}


// ------------------------------------------------------
void PlaceDataProvider::LaserServer::pushScanToClients(PlaceData::LaserScanPtr scan)
{
  for(unsigned int i = 0; i<_scanPushClients.size(); i++)
  {
    _scanPushClients[i].prx->receiveScan2d(scan->scanBuffer);
  }
}


// ------------------------------------------------------
void PlaceDataProvider::RobotbaseServer::pushOdometryToClients(PlaceData::OdometryPtr odom)
{
  for(unsigned int i = 0; i<_odomPushClients.size(); i++)
  {
    _odomPushClients[i].prx->receiveOdometry(odom->odometryBuffer);
  }
}


// ------------------------------------------------------
void PlaceDataProvider::runComponent()
{
  debug("Running...");

  // Add empty objects to WM
  _dataProviderCommandAckId = addEmptyDataProviderCommandAck();
  _imageId = addEmptyImage();
  _laserScanId = addEmptyLaserScan();
  _odometryId = addEmptyOdometry();
  _targetId = addEmptyTarget();

  // Run component
  while(isRunning())
  {
    // Get current time and add 1sec
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 1;

    // Wait if necessary
    pthread_mutex_lock(&_signalMutex);
    if (!_wasSignal)
      pthread_cond_timedwait(&_signalCond, &_signalMutex, &ts);

    // Handle signal if signal arrived
    if ((!isRunning()) || (!_wasSignal))
      pthread_mutex_unlock(&_signalMutex);
    else
    {
      _wasSignal=false;
      pthread_mutex_unlock(&_signalMutex);

      // Check for too high framerate (we shouldn't output data too fast!)
      double timeDiff = castTimeToSeconds(getCASTTime() - _lastGrabTimestamp);
      if (timeDiff < 0.05)
        usleep((0.05-timeDiff)*1000000.0);
      _lastGrabTimestamp = getCASTTime();

      debug("Grabbing data...");

      // Init reference time
      cdl::CASTTime refTimestamp;
      refTimestamp.s=0;
      refTimestamp.us=0;

      // Grab data and put to WM
      cdl::CASTTime imageTimestamp = outputImage();
      sched_yield();
      refTimestamp=imageTimestamp;
      if ((_scanDelay>0) && (!_loadDataFromDisk))
      {
        usleep(_scanDelay);
      }
      cdl::CASTTime scanTimestamp = outputLaserScan(refTimestamp);
      sched_yield();
      // NOW SCANS AND ODOMETRY SHOULD BE IN SYNC
//      if ((refTimestamp.m_s==0) && (refTimestamp.m_us==0))
        refTimestamp=scanTimestamp;
      cdl::CASTTime odomTimestamp = outputOdometryReading(refTimestamp);
      sched_yield();
      outputTarget();
      sched_yield();

      // Increment frame number
      _frameNo++;

      // Calculate statistics
      log("Synchronized data placed in the WM:");
      if (_useVision)
        log("-> Image (timestamp: %fs, difference to ref. time: %fs)",
                castTimeToSeconds(imageTimestamp),
                castTimeToSeconds(imageTimestamp-refTimestamp) );
      if (_useLaser)
        log("-> Laser scan (timestamp: %fs, difference to ref. time: %fs)",
                castTimeToSeconds(scanTimestamp),
                castTimeToSeconds(scanTimestamp - refTimestamp) );
      if (_useOdometry)
        log("-> Odometry (timestamp: %fs, difference to ref. time: %fs)",
                castTimeToSeconds(odomTimestamp),
                castTimeToSeconds(odomTimestamp - refTimestamp) );
    }
  }

  debug("Completed!");
}


// ------------------------------------------------------
void PlaceDataProvider::newDataProviderCommandAdded(const cdl::WorkingMemoryChange& wmc)
{
  // Get the id of the changed entry
  string dataId = wmc.address.id;

  // Get the command and remove from WM
  shared_ptr<CASTData<PlaceData::DataProviderCommand> > commandCast =
      getWorkingMemoryEntry<PlaceData::DataProviderCommand>(wmc.address);
  if (!commandCast)
    return;
  PlaceData::DataProviderCommandPtr command =
      new PlaceData::DataProviderCommand(*commandCast->getData());
  deleteFromWorkingMemory(wmc.address);

  // Send the ack
  PlaceData::DataProviderCommandAckPtr commandAck = new PlaceData::DataProviderCommandAck;
  commandAck->cmd = command;
  commandAck->src = wmc.src;
  overwriteWorkingMemory(_dataProviderCommandAckId, commandAck);

  // Execute command
  if (command->cmd==PlaceData::DpCmdUpdate)
  {
    debug("Received command DpCmdUpdate.");

    // Signal that new data should be grabbed
    pthread_mutex_lock(&_signalMutex);
    _wasSignal=true;
    pthread_cond_signal(&_signalCond);
    pthread_mutex_unlock(&_signalMutex);
  }
  else
  {
    throw(CASTException(exceptionMessage(__HERE__, "Unknown command!")));
  }
}


// ------------------------------------------------------
std::string PlaceDataProvider::addEmptyImage()
{
  PlaceData::ImagePtr image = new PlaceData::Image();
  image->status = PlaceData::DsInvalid;
  image->frameNo = -1;
  image->realTimeStamp.s=0;
  image->realTimeStamp.us=0;
  image->wmTimeStamp.s=0;
  image->wmTimeStamp.us=0;
  image->imageBuffer.width=0;
  image->imageBuffer.height=0;

  string dataId = newDataID();
  addToWorkingMemory(dataId, image);
  debug("Empty image placed in the WM.");

  return dataId;
}


// ------------------------------------------------------
std::string PlaceDataProvider::addEmptyLaserScan()
{
  PlaceData::LaserScanPtr scan = new PlaceData::LaserScan();
  scan->status = PlaceData::DsInvalid;
  scan->frameNo = -1;
  scan->realTimeStamp.s=0;
  scan->realTimeStamp.us=0;
  scan->wmTimeStamp.s=0;
  scan->wmTimeStamp.us=0;

  string dataId = newDataID();
  addToWorkingMemory(dataId, scan);
  debug("Empty laser scan placed in the WM.");

  return dataId;
}


// ------------------------------------------------------
std::string PlaceDataProvider::addEmptyOdometry()
{
  PlaceData::OdometryPtr odom = new PlaceData::Odometry();
  odom->status = PlaceData::DsInvalid;
  odom->frameNo = -1;
  odom->realTimeStamp.s=0;
  odom->realTimeStamp.us=0;
  odom->wmTimeStamp.s=0;
  odom->wmTimeStamp.us=0;

  string dataId = newDataID();
  addToWorkingMemory(dataId, odom);
  debug("Empty odometry reading placed in the WM.");

  return dataId;
}


// ------------------------------------------------------
std::string PlaceDataProvider::addEmptyTarget()
{
  PlaceData::TargetPtr target = new PlaceData::Target();
  target->frameNo=-1;
  target->targetNo=-1;
  target->targetName="";
  target->status = PlaceData::DsInvalid;

  string dataId = newDataID();
  addToWorkingMemory(dataId, target);
  debug("Empty target placed in the WM.");

  return dataId;
}


// ------------------------------------------------------
std::string PlaceDataProvider::addEmptyDataProviderCommandAck()
{
  PlaceData::DataProviderCommandAckPtr cmdAck = new PlaceData::DataProviderCommandAck();
  PlaceData::DataProviderCommandPtr cmd = new PlaceData::DataProviderCommand();
  cmd->cmd=PlaceData::DpCmdInvalid;
  cmdAck->cmd=cmd;
  cmdAck->src="";

  string dataId = newDataID();
  addToWorkingMemory(dataId, cmdAck);
  debug("Empty DataProviderCommandAck placed in the WM.");

  return dataId;
}


// ------------------------------------------------------
void PlaceDataProvider::pullImage(PlaceData::ImagePtr image)
{
  debug("Pulling an image...");

  cdl::CASTTime t1 = getCASTTime();
  Video::Image imageData;
  _videoServer->getImage(_cameraId, imageData);
  cdl::CASTTime t2 = getCASTTime();

  if ((imageData.width < 8) || (imageData.height<8))
  {
    throw(CASTException(exceptionMessage(__HERE__, "Unable to pull an image!")));
  }
  debug("Pulled image at %f in %fs.", castTimeToSeconds(t2), castTimeToSeconds(t2-t1));

  // Fill in the PlaceData::Image structure, convert image if necessary
  image->status = PlaceData::DsValid;
  image->frameNo = _frameNo;
  image->realTimeStamp = imageData.time;
  if (_videoServer==VS_STD)
    convertImageRGB24(imageData, image->imageBuffer);
  else
    throw(CASTException(exceptionMessage(__HERE__, "Incorrect video server mode in pullImage()!")));
}


// ------------------------------------------------------
cast::cdl::CASTTime PlaceDataProvider::outputImage()
{
  // Generate PlaceData::Image
  PlaceData::ImagePtr image = new PlaceData::Image();
  if (_useVision)
  {
    // Acquiring image
    if (_loadDataFromDisk)
    { // Load form disk
      if (!_dataReader.readImage(image))
        throw(CASTException(exceptionMessage(__HERE__, "Unable to load image from disk!" )));
      image->frameNo = _frameNo;
      debug("Loaded image from disk.");
    }
    else
    { // Grab from hardware server
      if (_videoServer==VS_STD)
        pullImage(image);
    }
  }
  else
  {
    // Return empty
    image->status = PlaceData::DsInvalid;
    image->frameNo = _frameNo;
    image->realTimeStamp.s=0;
    image->realTimeStamp.us=0;
    image->imageBuffer.width=0;
    image->imageBuffer.height=0;

    debug("Vision not used, created invalid image.");
  }

  // Set the WM timestamp
  cdl::CASTTime wmTimeStamp = getCASTTime();
  image->wmTimeStamp = wmTimeStamp;

  // Overwrite in the working memory
  cdl::CASTTime imageTimeStamp = image->realTimeStamp;
  overwriteWorkingMemory(_imageId, image);
  debug("Image placed in the WM at %f.", castTimeToSeconds(wmTimeStamp));

  // Return the image timestamp
  return imageTimeStamp;
}


// ------------------------------------------------------
void PlaceDataProvider::findLaserScan(const cast::cdl::CASTTime& refT, PlaceData::LaserScanPtr scan)
{
  // If refT is meaningful, grab closest
  // If refT.m_s=0, m_us=0, then grab the most recent scan
  if ((refT.s==0) && (refT.us==0))
  {
    // Most recent in the back of the queue
    pthread_mutex_lock(&_scanQueueMutex);
    if (_scanQueue.empty())
    {
      scan->status = PlaceData::DsInvalid;
      scan->frameNo = _frameNo;
      scan->realTimeStamp.s=0;
      scan->realTimeStamp.us=0;

      debug("The queue was empty! Returned invalid scan!");
    }
    else
    {
      scan->status = PlaceData::DsValid;
      scan->frameNo = _frameNo;
      scan->realTimeStamp = _scanQueue.back().time;
      scan->scanBuffer = _scanQueue.back();
    }
    pthread_mutex_unlock(&_scanQueueMutex);

    debug("Grabbed most recent laser scan from the queue");
  }
  else
  {
    pthread_mutex_lock(&_scanQueueMutex);
      double minTimeDiff = MAXDOUBLE;
      list<Laser::Scan2d>::iterator minDiffScanIter=_scanQueue.begin();
      for(list<Laser::Scan2d>::iterator iter=_scanQueue.begin(); iter!=_scanQueue.end(); iter++)
      {
        double tDiff = fabs(castTimeToSeconds(refT - iter->time));
        if (tDiff < minTimeDiff)
        {
          minTimeDiff = tDiff;
          minDiffScanIter = iter;
        }
      }
      if (minDiffScanIter==_scanQueue.end())
      { // The queue was empty!
        scan->status = PlaceData::DsInvalid;
        scan->frameNo = _frameNo;
        scan->realTimeStamp.s=0;
        scan->realTimeStamp.us=0;

        debug("The queue was empty! Returned invalid scan!");
      }
      else
      { // We've got scan!
        scan->status = PlaceData::DsValid;
        scan->frameNo = _frameNo;
        scan->realTimeStamp = minDiffScanIter->time;
        scan->scanBuffer = (*minDiffScanIter);

        debug("Returned valid scan from the queue!");
      }
    pthread_mutex_unlock(&_scanQueueMutex);
  } // if ((refT.m_s==0) && (refT.m_us==0))
}


// ------------------------------------------------------
cast::cdl::CASTTime PlaceDataProvider::outputLaserScan(const cast::cdl::CASTTime& refT)
{
  // Generate PlaceData::LaserScan
  PlaceData::LaserScanPtr scan = new PlaceData::LaserScan();
  if (_useLaser)
  {
    // Acquiring scan
    if (_loadDataFromDisk)
    { // Load form disk
      if (!_dataReader.readLaserScan(scan))
        throw(CASTException(exceptionMessage(__HERE__, "Unable to load laser scan from disk!")));
      scan->frameNo = _frameNo;
      debug("Loaded laser scan from disk.");
    }
    else
    { // Grab from queue
      findLaserScan(refT, scan);
    } // if (_loadDataFromDisk)
  } // if (_useLaser)
  else
  {
    // Return empty
    scan->status = PlaceData::DsInvalid;
    scan->frameNo = _frameNo;
    scan->realTimeStamp.s=0;
    scan->realTimeStamp.us=0;

    debug("Laser scans not used, created invalid scan.");
  }

  // Push to clients
  _laserServer.pushScanToClients(scan);

  // Set the WM timestamp
  cdl::CASTTime wmTimeStamp = getCASTTime();
  scan->wmTimeStamp = wmTimeStamp;

  // Overwrite in the working memory
  cdl::CASTTime scanTimeStamp=scan->realTimeStamp;
  overwriteWorkingMemory(_laserScanId, scan);
  debug("Laser scan placed in the WM at %f.", castTimeToSeconds(wmTimeStamp));

  // Return the scan timestamp
  return scanTimeStamp;
}


// ------------------------------------------------------
void PlaceDataProvider::findOdometryReading(const cast::cdl::CASTTime& refT, PlaceData::OdometryPtr odom)
{
  // If refT is meaningful, grab closest
  // If refT.m_s=0, m_us=0, then grab the most recent
  if ((refT.s==0) && (refT.us==0))
  {
    // Most recent in the back of the queue
    pthread_mutex_lock(&_odometryQueueMutex);
    if (_odometryQueue.empty())
    {
      odom->status = PlaceData::DsInvalid;
      odom->frameNo = _frameNo;
      odom->realTimeStamp.s=0;
      odom->realTimeStamp.us=0;

      debug("The queue was empty! Returned invalid odometry!");
    }
    else
    {
      odom->status = PlaceData::DsValid;
      odom->frameNo = _frameNo;
      odom->realTimeStamp = _odometryQueue.back().time;
      odom->odometryBuffer = _odometryQueue.back();
    }
    pthread_mutex_unlock(&_odometryQueueMutex);

    debug("Grabbed most recent odometry reading from the queue");
  }
  else
  {
    pthread_mutex_lock(&_odometryQueueMutex);
      double minTimeDiff = MAXDOUBLE;
      list<Robotbase::Odometry>::iterator minDiffOdomIter=_odometryQueue.begin();
      for(list<Robotbase::Odometry>::iterator iter=_odometryQueue.begin(); iter!=_odometryQueue.end(); iter++)
      {
        double tDiff = fabs(castTimeToSeconds(refT-iter->time));
        if (tDiff < minTimeDiff)
        {
          minTimeDiff = tDiff;
          minDiffOdomIter = iter;
        }
      }
      if (minDiffOdomIter==_odometryQueue.end())
      { // The queue was empty!
        odom->status = PlaceData::DsInvalid;
        odom->frameNo = _frameNo;
        odom->realTimeStamp.s=0;
        odom->realTimeStamp.us=0;

        debug("The queue was empty! Returned invalid odometry!");
      }
      else
      { // We've got odometry reading!
        odom->status = PlaceData::DsValid;
        odom->frameNo = _frameNo;
        odom->realTimeStamp = minDiffOdomIter->time;
        odom->odometryBuffer = (*minDiffOdomIter);

        debug("Returned valid odometry reading from the queue!");
      }
    pthread_mutex_unlock(&_odometryQueueMutex);
  } // if ((refT.m_s==0) && (refT.m_us==0))
}


// ------------------------------------------------------
cast::cdl::CASTTime PlaceDataProvider::outputOdometryReading(const cast::cdl::CASTTime& refT)
{
  // Generate PlaceData::Odometry
  PlaceData::OdometryPtr odom = new PlaceData::Odometry();
  if (_useOdometry)
  {
    // Acquiring odometry
    if (_loadDataFromDisk)
    { // Load form disk
      if (!_dataReader.readOdometry(odom))
        throw(CASTException(exceptionMessage(__HERE__, "Unable to load odometry from disk!")));
      odom->frameNo = _frameNo;
      debug("Loaded odometry reading from disk.");
    }
    else
    { // Grab from queue
      findOdometryReading(refT, odom);
    } // if (_loadDataFromDisk)
  } // if (_useOdometry)
  else
  {
    // Return empty
    odom->status = PlaceData::DsInvalid;
    odom->frameNo = _frameNo;
    odom->realTimeStamp.s=0;
    odom->realTimeStamp.us=0;

    debug("Odometry not used, created invalid odometry reading.");
  }

  // Push to clients
  _robotbaseServer.pushOdometryToClients(odom);

  // Set the WM timestamp
  cdl::CASTTime wmTimeStamp = getCASTTime();
  odom->wmTimeStamp = wmTimeStamp;

  // Overwrite in the working memory
  cdl::CASTTime odomTimeStamp=odom->realTimeStamp;
  overwriteWorkingMemory(_odometryId, odom);
  debug("Odometry reading placed in the WM at %f.", castTimeToSeconds(wmTimeStamp));

  // Return the odometry timestamp
  return odomTimeStamp;
}


// ------------------------------------------------------
void PlaceDataProvider::outputTarget()
{
  // Generate PlaceData::Target
  PlaceData::TargetPtr target = new PlaceData::Target();
  // Acquiring target
  if (_loadDataFromDisk)
  { // Load form disk
    if (!_dataReader.readTarget(target))
      throw(CASTException(exceptionMessage(__HERE__, "Unable to load target from disk!")));
    target->frameNo = _frameNo;
    debug("Loaded target from disk.");
  }
  else
  { // Return empty
    target->status = PlaceData::DsInvalid;
    target->frameNo = _frameNo;
    target->targetNo = -1;
    target->targetName="";
  }

  // Overwrite in the working memory
  overwriteWorkingMemory(_targetId, target);
  debug("Target placed in the WM.");
}


// ------------------------------------------------------
void PlaceDataProvider::convertImageBGR24(const Video::Image &src, PlaceData::ImageData &dest) const
{
  // The src image is always BGR24
  dest.width=src.width;
  dest.height=src.height;
  unsigned int l=dest.width*dest.height;
  dest.data.resize(l);

  for(unsigned int i = 0; i < l; i++)
  {
    // Y = 0.3R + 0.59G + 0.11B
    dest.data[i] = static_cast<char>( (
    static_cast<unsigned int>(static_cast<unsigned char>(src.data[i*3+0]))*114 +   // B
    static_cast<unsigned int>(static_cast<unsigned char>(src.data[i*3+1]))*587 +   // G
    static_cast<unsigned int>(static_cast<unsigned char>(src.data[i*3+2]))*299     // R
                                 ) / 1000 );
  }
}


// ------------------------------------------------------
void PlaceDataProvider::convertImageRGB24(const Video::Image &src, PlaceData::ImageData &dest) const
{
  dest.width=src.width;
  dest.height=src.height;
  unsigned int l=dest.width*dest.height;
  dest.data.resize(l);

  for(unsigned int i = 0; i < l; i++)
  {
    // Y = 0.3R + 0.59G + 0.11B
    dest.data[i] = static_cast<char>( (
                      static_cast<unsigned int>(static_cast<unsigned char>(src.data[i*3+0]))*299 +    // R
                      static_cast<unsigned int>(static_cast<unsigned char>(src.data[i*3+1]))*587 +    // G
                      static_cast<unsigned int>(static_cast<unsigned char>(src.data[i*3+2]))*114      // B
                                         ) / 1000 );
  }
}







// UNUSED CODE

/*
// ------------------------------------------------------
bool PlaceDataProvider::grabImageFromDCamVideoServerSocket(PlaceData::Image *image)
{
  // Create the socket
  int sock = socket(PF_INET, SOCK_STREAM, 0);
  if (sock < 0)
  {
    println("DCamVideoServer socket client: Cannot open socket!");
    return false;
  }

  // Init server address;
  sockaddr_in serverName;
  hostent *hostInfo;
  serverName.sin_family = AF_INET;
  serverName.sin_port = htons(_socketServerPort);
  hostInfo = gethostbyname(_socketServerHostname.c_str());
  if (hostInfo == NULL)
  {
    println("DCamVideoServer socket client: Unknown host %s!", _socketServerHostname.c_str());
    close(sock);
    return false;
  }
  serverName.sin_addr = *(struct in_addr *) hostInfo->h_addr;

  // Connect to the server
  if (0 > connect (sock, (struct sockaddr *) &serverName, sizeof (serverName)))
  {
    println("DCamVideoServer socket client: Cannot connect to the server!");
    close(sock);
    return false;
  }

  // Send the image format and size information
  char format = ImageSocketServer::FORMAT_GREY;
  char size = ImageSocketServer::SIZE_320240;
  if (write(sock, &format, 1) != 1)
  {
    println("DCamVideoServer socket client: Cannot send the image format!");
    close(sock);
    return false;
  }
  if (write(sock, &size, 1) != 1)
  {
    println("DCamVideoServer socket client: Cannot send the image size!");
    close(sock);
    return false;
  }

  // Read the size
  uint16_t width;
  uint16_t height;

  int bytesRead;
  bytesRead = read(sock, &width, sizeof(uint16_t));
  if (bytesRead!=sizeof(uint16_t))
  {
    println("DCamVideoServer socket client: Incorrect number of bytes received from server!");
    close(sock);
    return false;
  }

  bytesRead = read(sock, &height, sizeof(uint16_t));
  if (bytesRead!=sizeof(uint16_t))
  {
    println("DCamVideoServer socket client: Incorrect number of bytes received from server!");
    close(sock);
    return false;
  }

  // Check the size received
  if ((width!=320) || (height!=240))
    throw CASTException(__HERE__, "Incorrect image size (%d, %d)!", width, height);

  // Set image size and channels
  image->imageBuffer.m_width = width;
  image->imageBuffer.m_height = height;
  image->imageBuffer.m_nChannels = 1;

  // Read the time
  timeval time;

  bytesRead = read(sock, &time, sizeof(timeval));
  if (bytesRead!=sizeof(timeval))
  {
    println("DCamVideoServer socket client: Incorrect number of bytes received from server!");
    close(sock);
    return false;
  }

  image->realTimeStamp.m_s = time.tv_sec;
  image->realTimeStamp.m_us = time.tv_usec;

  // Read the data
  image->imageBuffer.m_image.length(width*height);
  bytesRead=0;
  while (bytesRead<width*height)
  {
    int br = read(sock, &(image->imageBuffer.m_image[0])+bytesRead, width*height-bytesRead);
    if (br<=0)
    {
      println("DCamVideoServer socket client: Connection broken!");
      close(sock);
      return false;
    }
    bytesRead+=br;
  }

  // Close socket
  close(sock);

  return true;
}


// ------------------------------------------------------
void PlaceDataProvider::pullImageFromSocket(PlaceData::Image *image)
{
  debug("Pulling image from socket server...");

  FrameworkBasics::BALTTime t1 = BALTTimer::getBALTTime();
  if (!grabImageFromDCamVideoServerSocket(image))
  {
    println("Couldn't grab the image from the DCamVideoServer! Returning invalid image!");
    // If failed to grab image, return empty one
    image->status = PlaceData::DS_INVALID;
    image->frameNo = _frameNo;
    image->realTimeStamp.m_s=0;
    image->realTimeStamp.m_us=0;
    image->imageBuffer.m_width=0;
    image->imageBuffer.m_height=0;
    image->imageBuffer.m_nChannels=0;
  }
  FrameworkBasics::BALTTime t2 = BALTTimer::getBALTTime();

  // Message
  debug("Pulled an image from a socket at %f in %fs.", BALTTimer::toSeconds(t2), BALTTimer::toSeconds(BALTTimer::timeDiff(t1, t2)));

  // Fill in the PlaceData::Image structure
  image->status = PlaceData::DS_VALID;
  image->frameNo = _frameNo;
}

 */
