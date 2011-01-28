/**
 * CategoricalDataSaver class.
 * \file DataSaver.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-14
 */

// Categorical.SA
#include "DataSaver.h"
#include "shared/ConfigFile.h"
#include "shared/DataWriter.h"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>
// Boost
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace cast;
using namespace categorical;
using boost::bad_lexical_cast;
using boost::shared_ptr;


// ------------------------------------------------------
const string doNotSaveStr = "<DO_NOT_SAVE>";

// ------------------------------------------------------
extern "C" 
{
  cast::interfaces::CASTComponentPtr newComponent()
  {
    return new CategoricalDataSaver();
  }
}


// ------------------------------------------------------
CategoricalDataSaver::CategoricalDataSaver(): _cfgGroup("DataSaver")
{
  _wasSaveImageSignal=false;
  _wasSaveScanSignal=false;
  _wasSaveOdometrySignal=false;
  _saveData=false;

  pthread_mutex_init(&_saveMutex, 0);
  pthread_cond_init(&_saveSignalCond, 0);
  pthread_mutex_init(&_saveSignalMutex, 0);
}


// ------------------------------------------------------
CategoricalDataSaver::~CategoricalDataSaver()
{
  pthread_mutex_destroy(&_saveMutex);
  pthread_cond_destroy(&_saveSignalCond);
  pthread_mutex_destroy(&_saveSignalMutex);
}


// ------------------------------------------------------
void CategoricalDataSaver::configure(const map<string,string> &config)
{
  // Read cmd line options
  map<string, string>::const_iterator it = config.find("--config");
  string configFile;
  if (it!=config.end())
    configFile = it->second;

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
    _updateDelay=cf.getIntValue(_cfgGroup, "UpdateDelay", 100000);
  }
  catch(bad_lexical_cast &)
  {
    throw(CASTException(exceptionMessage(__HERE__, "Incorrect item value in the config file '%s'!", configFile.c_str())));
  }

  // Print the configuration
  log("Configuration:");
  log("-> Update delay: %i", _updateDelay);
}


// ------------------------------------------------------
void CategoricalDataSaver::start()
{
  // newImageAdded
  addChangeFilter(createLocalTypeFilter<CategoricalData::Image>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<CategoricalDataSaver>(this,
          &CategoricalDataSaver::newImage));

  // newLaserScanAdded
  addChangeFilter(createLocalTypeFilter<CategoricalData::LaserScan>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<CategoricalDataSaver>(this,
          &CategoricalDataSaver::newLaserScan));

  // newOdometryAdded
  addChangeFilter(createLocalTypeFilter<CategoricalData::Odometry>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<CategoricalDataSaver>(this,
          &CategoricalDataSaver::newOdometry));

  debug("Started!");
}


// ------------------------------------------------------
void CategoricalDataSaver::stop()
{
  debug("Stop!");
}


// ------------------------------------------------------
void CategoricalDataSaver::startSavingData(std::string dirPath, std::string baseName, int targetNo, std::string targetName)
{
  pthread_mutex_lock(&_saveMutex);

  if (_saveData)
  {
    pthread_mutex_unlock(&_saveMutex);
    println("Cannot start data saving process as the previous one is still running!");
    return;
  }

  // Save paths
  _saveDirPath=dirPath;
  _saveBaseName=baseName;
  _targetNo=targetNo;
  _targetName=targetName;

  // Say that all data were saved, this will be corrected later
  _imagesSaved=true;
  _scansSaved=true;
  _odometrySaved=true;
  _framesSaved=0;

  // Pause saving
  _saveData=false;

  pthread_mutex_unlock(&_saveMutex);
}


// ------------------------------------------------------
void CategoricalDataSaver::stopSavingData()
{
  pthread_mutex_lock(&_saveMutex);

  // Save data config file
  if (_saveDirPath!=doNotSaveStr)
  {
    DataWriter dw(_saveDirPath, _saveBaseName);
    if ( !dw.writeDataConfigFile(_imagesSaved, _scansSaved, _odometrySaved, _framesSaved) )
      println("Error! Cannot save the data config file!");
  }

  // Stop saving
  _saveDirPath="";
  _saveBaseName="";
  _saveData=false;

  pthread_mutex_unlock(&_saveMutex);
}


// ------------------------------------------------------
void CategoricalDataSaver::pauseSavingData()
{
  pthread_mutex_lock(&_saveMutex);

  _saveData=false;
  if ((_saveDirPath.empty()) and (_saveBaseName.empty()))
  {
    println("Cannot pause data saving as it's not running!");
  }

  pthread_mutex_unlock(&_saveMutex);
}


// ------------------------------------------------------
void CategoricalDataSaver::unpauseSavingData()
{
  pthread_mutex_lock(&_saveMutex);

  if ((_saveDirPath.empty()) and (_saveBaseName.empty()))
  {
    pthread_mutex_unlock(&_saveMutex);
    throw(CASTException(exceptionMessage(__HERE__, "Cannot unpause data saving as it's not paused!")));
  }
  _saveData=true;

  pthread_mutex_unlock(&_saveMutex);
}


// ------------------------------------------------------
void CategoricalDataSaver::setNewTarget(int targetNo, std::string targetName)
{
  pthread_mutex_lock(&_saveMutex);

  _targetNo=targetNo;
  _targetName=targetName;

  pthread_mutex_unlock(&_saveMutex);
}


// ------------------------------------------------------
void CategoricalDataSaver::newImage(const cast::cdl::WorkingMemoryChange & change)
{
  pthread_mutex_lock(&_saveSignalMutex);
  _wasSaveImageSignal=true;
  _saveImageAddress=change.address;
  pthread_cond_signal(&_saveSignalCond);
  pthread_mutex_unlock(&_saveSignalMutex);
}


// ------------------------------------------------------
void CategoricalDataSaver::newLaserScan(const cast::cdl::WorkingMemoryChange & change)
{
  pthread_mutex_lock(&_saveSignalMutex);
  _wasSaveScanSignal=true;
  _saveScanAddress=change.address;
  pthread_cond_signal(&_saveSignalCond);
  pthread_mutex_unlock(&_saveSignalMutex);
}


// ------------------------------------------------------
void CategoricalDataSaver::newOdometry(const cast::cdl::WorkingMemoryChange & change)
{
  pthread_mutex_lock(&_saveSignalMutex);
  _wasSaveOdometrySignal=true;
  _saveOdometryAddress=change.address;
  pthread_cond_signal(&_saveSignalCond);
  pthread_mutex_unlock(&_saveSignalMutex);
}



// ------------------------------------------------------
void CategoricalDataSaver::runComponent()
{
  debug("Running...");

  usleep(5e6);

  timeval tv;
  gettimeofday(&tv, NULL);

  startSavingData("data", boost::lexical_cast<string>(tv.tv_sec), 0, "unknown");
  unpauseSavingData();

  // Add empty objects first
  //  _dataSaverStatusId = addEmptyDataSaverStatus();

  // Send first dpupdate
  sendDpUpdateCommand();

  // Run component
  while(isRunning())
  {
	  // Get current time and add 1sec
	  timespec ts;
	  gettimeofday(&tv, NULL);
	  ts.tv_sec = tv.tv_sec + 1;
	  ts.tv_nsec = 0;
    // Wait if necessary
    pthread_mutex_lock(&_saveSignalMutex);
    if ((!_wasSaveImageSignal) || (!_wasSaveScanSignal) || (!_wasSaveOdometrySignal))
      pthread_cond_timedwait(&_saveSignalCond, &_saveSignalMutex, &ts);

    // Handle signal if signal arrived
    if ((!isRunning()) || (!_wasSaveImageSignal) || (!_wasSaveScanSignal) || (!_wasSaveOdometrySignal))
      pthread_mutex_unlock(&_saveSignalMutex);
    else
    {
      _wasSaveImageSignal=false;
      _wasSaveScanSignal=false;
      _wasSaveOdometrySignal=false;
      cdl::WorkingMemoryAddress imageAddress=_saveImageAddress;
      cdl::WorkingMemoryAddress scanAddress=_saveScanAddress;
      cdl::WorkingMemoryAddress odometryAddress=_saveOdometryAddress;
      pthread_mutex_unlock(&_saveSignalMutex);

      pthread_mutex_lock(&_saveMutex);
      if (_saveData)
      {
        string saveDirPath=_saveDirPath;
        string saveBaseName=_saveBaseName;
        long frameNo = _framesSaved;
        int targetNo = _targetNo;
        string targetName = _targetName;
        _framesSaved++;
        pthread_mutex_unlock(&_saveMutex);

        // Grab and check frameNo
        sched_yield();
        shared_ptr<CASTData<CategoricalData::Image> > imageCast =
            getWorkingMemoryEntry<CategoricalData::Image>(imageAddress);
        if (!imageCast)
          throw(CASTException(exceptionMessage(__HERE__, "Cannot get image from WM!")));
        const CategoricalData::ImagePtr image = imageCast->getData();
        shared_ptr<CASTData<CategoricalData::LaserScan> > laserScanCast =
                    getWorkingMemoryEntry<CategoricalData::LaserScan>(scanAddress);
        if (!laserScanCast)
          throw(CASTException(exceptionMessage(__HERE__, "Cannot get laser scan from WM!" )));
        const CategoricalData::LaserScanPtr laserScan = laserScanCast->getData();
        shared_ptr<CASTData<CategoricalData::Odometry> > odometryCast =
            getWorkingMemoryEntry<CategoricalData::Odometry>(odometryAddress);
        if (!odometryCast)
          throw(CASTException(exceptionMessage(__HERE__, "Cannot get odometry from WM")));
        const CategoricalData::OdometryPtr odometry = odometryCast->getData();

        if ((image->frameNo!=laserScan->frameNo) || (image->frameNo!=odometry->frameNo))
          throw(CASTException(exceptionMessage(__HERE__,
              "Something is wrong, the frame numbers of synchronized data do not match!")));

        bool imageValid=(image->status==CategoricalData::DsValid);
        bool scanValid=(laserScan->status==CategoricalData::DsValid);
        bool odomValid=(odometry->status==CategoricalData::DsValid);

        // Make copies of the data so that they disappear faster from WM
        sched_yield();
        CategoricalData::ImagePtr imageCopy = new CategoricalData::Image( *image );
        sched_yield();
        CategoricalData::LaserScanPtr scanCopy = new CategoricalData::LaserScan( *laserScan );
        sched_yield();
        CategoricalData::OdometryPtr odometryCopy = new CategoricalData::Odometry( *odometry );
        sched_yield();


        // Save
        bool error=false;
        if (saveDirPath!=doNotSaveStr) // for testing
        {
          DataWriter dw(_saveDirPath, _saveBaseName);
          dw.setFrameNo(frameNo);
          sched_yield();
          if (imageValid)
          {
            	log("Writing image!");
              if (!dw.writeImage(imageCopy, DataWriter::F_PGM))
              error=true;
            sched_yield();
          }
          if (scanValid)
          {
          	log("Writing scan!");
            if (!dw.writeLaserScan(scanCopy))
              error=true;
            sched_yield();
          }
          if (odomValid)
          {
            	log("Writing odometry!");
            if (!dw.writeOdometry(odometryCopy))
              error=true;
            sched_yield();
          }
      	log("Writing target!");
          if (!dw.writeTarget(targetNo, targetName))
            error=true;
          sched_yield();

          if ( !dw.writeDataConfigFile(_imagesSaved, _scansSaved, _odometrySaved, _framesSaved) )
              error=true;

        }

        // Correct bools saying what was actually saved
        pthread_mutex_lock(&_saveMutex);
        if (!imageValid)
          _imagesSaved=false;
        if (!scanValid)
          _scansSaved=false;
        if (!odomValid)
          _odometrySaved=false;
        pthread_mutex_unlock(&_saveMutex);

        // Send the status
        debug("Saved frame!");
        sched_yield();
//        sendNewDataSaverStatus(imageValid, scanValid, odomValid, frameNo+1, image->frameNo, error);
      }
      else
      {
        pthread_mutex_unlock(&_saveMutex);
        // Send status
//        sendNewDataSaverStatus(false, false, false, 0, -1, false);
      }

      usleep(_updateDelay);
      sendDpUpdateCommand();

    } // if
  } // while

  debug("Completed.");
}


// ------------------------------------------------------
void CategoricalDataSaver::sendDpUpdateCommand()
{
  CategoricalData::DataProviderCommandPtr dataProviderCommand = new CategoricalData::DataProviderCommand;
  dataProviderCommand->cmd = CategoricalData::DpCmdUpdate;
  string dataId = newDataID();
  addToWorkingMemory(dataId, dataProviderCommand);
  debug("Sent DpCmdUpdate command.");
}


// ------------------------------------------------------
//void CategoricalDataSaver::sendNewDataSaverStatus(bool savedImage, bool savedLaserScan, bool savedOdometry, long framesSaved, long frameNo, bool error)
//{
//  // Generate new status
//  CategoricalData::DataSaverStatusPtr status = new CategoricalData::DataSaverStatus;
//  status->savedImage = savedImage;
//  status->savedLaserScan = savedLaserScan;
//  status->savedOdometry = savedOdometry;
//  status->framesSaved = framesSaved;
//  status->frameNo = frameNo;
//  status->error = error;
//
//  // Get the current status and overwrite
//  overwriteWorkingMemory(_dataSaverStatusId, status);
//
//  debug("Sent DataSaverStatus.");
//}


// ------------------------------------------------------
//std::string CategoricalDataSaver::addEmptyDataSaverStatus()
//{
//  // Generate new status
//  CategoricalData::DataSaverStatusPtr status = new CategoricalData::DataSaverStatus;
//  status->savedImage = false;
//  status->savedLaserScan = false;
//  status->savedOdometry = false;
//  status->framesSaved = -1;
//  status->frameNo = -1;
//  status->error = false;
//
//  string dataId = newDataID();
//  addToWorkingMemory(dataId, status);
//  debug("Added empty DataSaverStatus.");
//
//  return dataId;
//}


