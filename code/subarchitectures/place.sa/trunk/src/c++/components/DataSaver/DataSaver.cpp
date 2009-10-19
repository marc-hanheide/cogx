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
 * PlaceDataSaver class.
 * \file DataSaver.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-14
 */

// Place.SA
#include "DataSaver.h"
#include "shared/ConfigFile.h"
#include "shared/DataWriter.h"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>
// Boost
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace cast;
using namespace place;
using boost::bad_lexical_cast;
using boost::shared_ptr;


// ------------------------------------------------------
const string doNotSaveStr = "<DO_NOT_SAVE>";

// ------------------------------------------------------
extern "C" 
{
  cast::interfaces::CASTComponentPtr newComponent()
  {
    return new PlaceDataSaver();
  }
}


// ------------------------------------------------------
PlaceDataSaver::PlaceDataSaver(): _cfgGroup("DataSaver")
{
  _wasSaveImageSignal=false;
  _wasSaveScanSignal=false;
  _wasSaveOdometrySignal=false;
  _saveData=false;
  _updateOnRead=false;

  pthread_mutex_init(&_saveMutex, 0);
  pthread_cond_init(&_saveSignalCond, 0);
  pthread_mutex_init(&_saveSignalMutex, 0);
}


// ------------------------------------------------------
PlaceDataSaver::~PlaceDataSaver()
{
  pthread_mutex_destroy(&_saveMutex);
  pthread_cond_destroy(&_saveSignalCond);
  pthread_mutex_destroy(&_saveSignalMutex);
}


// ------------------------------------------------------
void PlaceDataSaver::configure(const map<string,string> &config)
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
void PlaceDataSaver::start()
{
  // newImageAdded
  addChangeFilter(createLocalTypeFilter<PlaceData::Image>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceDataSaver>(this,
          &PlaceDataSaver::newImage));

  // newLaserScanAdded
  addChangeFilter(createLocalTypeFilter<PlaceData::LaserScan>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceDataSaver>(this,
          &PlaceDataSaver::newLaserScan));

  // newOdometryAdded
  addChangeFilter(createLocalTypeFilter<PlaceData::Odometry>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceDataSaver>(this,
          &PlaceDataSaver::newOdometry));

  // newDataSaverCommandAdded
  addChangeFilter(createLocalTypeFilter<PlaceData::DataSaverCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<PlaceDataSaver>(this,
          &PlaceDataSaver::newDataSaverCommand));

  debug("Started!");
}


// ------------------------------------------------------
void PlaceDataSaver::stop()
{
  debug("Stop!");
}


// ------------------------------------------------------
void PlaceDataSaver::newDataSaverCommand(const cast::cdl::WorkingMemoryChange & change)
{
  // Get the command and remove from WM
  shared_ptr<CASTData<PlaceData::DataSaverCommand> > commandCast =
    getWorkingMemoryEntry<PlaceData::DataSaverCommand>(change.address);
  if (!commandCast)
    return;
  PlaceData::DataSaverCommandPtr command =
      new PlaceData::DataSaverCommand(*commandCast->getData());
  deleteFromWorkingMemory(change.address);

  // Send the ack
  PlaceData::DataSaverCommandAckPtr commandAck = new PlaceData::DataSaverCommandAck;
  commandAck->cmd = command;
  commandAck->src = change.src;
  overwriteWorkingMemory(_dataSaverCommandAckId, commandAck);

  // Execute command
  if (command->cmd==PlaceData::DsCmdStart)
  {
    debug("Received command DsCmdStart.");
    startSavingData(command->dataDirPath, command->dataBaseName,
                    (int)(command->targetNo), command->targetName);
  }
  else if (command->cmd==PlaceData::DsCmdStop)
  {
    debug("Received command DsCmdStop.");
    stopSavingData();
  }
  else if (command->cmd==PlaceData::DsCmdPause)
  {
    debug("Received command DsCmdPause.");
    pauseSavingData();
  }
  else if (command->cmd==PlaceData::DsCmdUnpause)
  {
    debug("Received command DsCmdUnpause.");
    unpauseSavingData();
  }
  else if (command->cmd==PlaceData::DsCmdNewTarget)
  {
    debug("Received command DsCmdNewTarget.");
    setNewTarget((int)(command->targetNo), command->targetName);
  }
  else if (command->cmd==PlaceData::DsCmdUpdateStart)
  {
    debug("Received command DsCmdUpdateStart.");
    _updateOnRead=true;
  }
  else if (command->cmd==PlaceData::DsCmdUpdateStop)
  {
    debug("Received command DsCmdUpdateStop.");
    _updateOnRead=false;
  }
  else
  {
    throw(CASTException(exceptionMessage(__HERE__, "Unknown command!")));
  }
}


// ------------------------------------------------------
void PlaceDataSaver::startSavingData(std::string dirPath, std::string baseName, int targetNo, std::string targetName)
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
void PlaceDataSaver::stopSavingData()
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
void PlaceDataSaver::pauseSavingData()
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
void PlaceDataSaver::unpauseSavingData()
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
void PlaceDataSaver::setNewTarget(int targetNo, std::string targetName)
{
  pthread_mutex_lock(&_saveMutex);

  _targetNo=targetNo;
  _targetName=targetName;

  pthread_mutex_unlock(&_saveMutex);
}


// ------------------------------------------------------
void PlaceDataSaver::newImage(const cast::cdl::WorkingMemoryChange & change)
{
  pthread_mutex_lock(&_saveSignalMutex);
  _wasSaveImageSignal=true;
  _saveImageAddress=change.address;
  pthread_cond_signal(&_saveSignalCond);
  pthread_mutex_unlock(&_saveSignalMutex);
}


// ------------------------------------------------------
void PlaceDataSaver::newLaserScan(const cast::cdl::WorkingMemoryChange & change)
{
  pthread_mutex_lock(&_saveSignalMutex);
  _wasSaveScanSignal=true;
  _saveScanAddress=change.address;
  pthread_cond_signal(&_saveSignalCond);
  pthread_mutex_unlock(&_saveSignalMutex);
}


// ------------------------------------------------------
void PlaceDataSaver::newOdometry(const cast::cdl::WorkingMemoryChange & change)
{
  pthread_mutex_lock(&_saveSignalMutex);
  _wasSaveOdometrySignal=true;
  _saveOdometryAddress=change.address;
  pthread_cond_signal(&_saveSignalCond);
  pthread_mutex_unlock(&_saveSignalMutex);
}



// ------------------------------------------------------
void PlaceDataSaver::runComponent()
{
  debug("Running...");

  // Add empty objects first
  _dataSaverStatusId = addEmptyDataSaverStatus();
  _dataSaverCommandAckId = addEmptyDataSaverCommandAck();

  // Run component
  while(isRunning())
  {
    // Get current time and add 1 sec
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 1;

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
        shared_ptr<CASTData<PlaceData::Image> > imageCast =
            getWorkingMemoryEntry<PlaceData::Image>(imageAddress);
        if (!imageCast)
          throw(CASTException(exceptionMessage(__HERE__, "Cannot get image from WM!")));
        const PlaceData::ImagePtr image = imageCast->getData();
        shared_ptr<CASTData<PlaceData::LaserScan> > laserScanCast =
                    getWorkingMemoryEntry<PlaceData::LaserScan>(scanAddress);
        if (!laserScanCast)
          throw(CASTException(exceptionMessage(__HERE__, "Cannot get laser scan from WM!" )));
        const PlaceData::LaserScanPtr laserScan = laserScanCast->getData();
        shared_ptr<CASTData<PlaceData::Odometry> > odometryCast =
            getWorkingMemoryEntry<PlaceData::Odometry>(odometryAddress);
        if (!odometryCast)
          throw(CASTException(exceptionMessage(__HERE__, "Cannot get odometry from WM")));
        const PlaceData::OdometryPtr odometry = odometryCast->getData();

        if ((image->frameNo!=laserScan->frameNo) || (image->frameNo!=odometry->frameNo))
          throw(CASTException(exceptionMessage(__HERE__,
              "Something is wrong, the frame numbers of synchronized data do not match!")));

        bool imageValid=(image->status==PlaceData::DsValid);
        bool scanValid=(laserScan->status==PlaceData::DsValid);
        bool odomValid=(odometry->status==PlaceData::DsValid);

        // Make copies of the data so that they disappear faster from WM
        sched_yield();
        PlaceData::ImagePtr imageCopy = new PlaceData::Image( *image );
        sched_yield();
        PlaceData::LaserScanPtr scanCopy = new PlaceData::LaserScan( *laserScan );
        sched_yield();
        PlaceData::OdometryPtr odometryCopy = new PlaceData::Odometry( *odometry );
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
            if (!dw.writeImage(imageCopy, DataWriter::F_PGM))
              error=true;
            sched_yield();
          }
          if (scanValid)
          {
            if (!dw.writeLaserScan(scanCopy))
              error=true;
            sched_yield();
          }
          if (odomValid)
          {
            if (!dw.writeOdometry(odometryCopy))
              error=true;
            sched_yield();
          }
          if (!dw.writeTarget(targetNo, targetName))
            error=true;
          sched_yield();
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
        sendNewDataSaverStatus(imageValid, scanValid, odomValid, frameNo+1, image->frameNo, error);
      }
      else
      {
        pthread_mutex_unlock(&_saveMutex);
        // Send status
        sendNewDataSaverStatus(false, false, false, 0, -1, false);
      }

      // Send update if asked to do so
      if (_updateOnRead)
      {
        usleep(_updateDelay);
        sendDpUpdateCommand();
      }

    } // if
  } // while

  debug("Completed.");
}


// ------------------------------------------------------
void PlaceDataSaver::sendDpUpdateCommand()
{
  PlaceData::DataProviderCommandPtr dataProviderCommand = new PlaceData::DataProviderCommand;
  dataProviderCommand->cmd = PlaceData::DpCmdUpdate;
  string dataId = newDataID();
  addToWorkingMemory(dataId, dataProviderCommand);
  debug("Sent DpCmdUpdate command.");
}


// ------------------------------------------------------
void PlaceDataSaver::sendNewDataSaverStatus(bool savedImage, bool savedLaserScan, bool savedOdometry, long framesSaved, long frameNo, bool error)
{
  // Generate new status
  PlaceData::DataSaverStatusPtr status = new PlaceData::DataSaverStatus;
  status->savedImage = savedImage;
  status->savedLaserScan = savedLaserScan;
  status->savedOdometry = savedOdometry;
  status->framesSaved = framesSaved;
  status->frameNo = frameNo;
  status->error = error;

  // Get the current status and overwrite
  overwriteWorkingMemory(_dataSaverStatusId, status);

  debug("Sent DataSaverStatus.");
}


// ------------------------------------------------------
std::string PlaceDataSaver::addEmptyDataSaverStatus()
{
  // Generate new status
  PlaceData::DataSaverStatusPtr status = new PlaceData::DataSaverStatus;
  status->savedImage = false;
  status->savedLaserScan = false;
  status->savedOdometry = false;
  status->framesSaved = -1;
  status->frameNo = -1;
  status->error = false;

  string dataId = newDataID();
  addToWorkingMemory(dataId, status);
  debug("Added empty DataSaverStatus.");

  return dataId;
}



// ------------------------------------------------------
std::string PlaceDataSaver::addEmptyDataSaverCommandAck()
{
  PlaceData::DataSaverCommandAckPtr cmdAck = new PlaceData::DataSaverCommandAck();
  PlaceData::DataSaverCommandPtr cmd = new PlaceData::DataSaverCommand();
  cmd->cmd=PlaceData::DsCmdInvalid;
  cmdAck->cmd=cmd;
  cmdAck->src="";

  string dataId = newDataID();
  addToWorkingMemory(dataId, cmdAck);
  debug("Empty DataSaverCommandAck placed in the WM.");

  return dataId;
}

