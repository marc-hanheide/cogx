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
 * \file DataSaver.h
 * \author Andrzej Pronobis
 * \date 2008-08-14
 */

#ifndef __PLACE_DATA_SAVER__
#define __PLACE_DATA_SAVER__

#include <PlaceData.hpp>
#include <cast/architecture/ManagedComponent.hpp>
#include "boost/filesystem.hpp"


/**
 * Implements the DataSaver component.
 */
class PlaceDataSaver: public cast::ManagedComponent
{
public: // Component management

  /** Constructor. */
  PlaceDataSaver();

  /** Destructor. */
  virtual ~PlaceDataSaver();

  /** Handles component configuration. */
  virtual void configure(const std::map<std::string,std::string> &config);

  /** Main thread of the component. */
  virtual void runComponent();

  /** Invoked on start. */
  virtual void start();

  /** Invoked on stop. */
  virtual void stop();


private:

  /** New Image added to wm. */
  void newImage(const cast::cdl::WorkingMemoryChange & change);

  /** New LaserScan added to wm. */
  void newLaserScan(const cast::cdl::WorkingMemoryChange & change);

  /** New Odometry added to wm. */
  void newOdometry(const cast::cdl::WorkingMemoryChange & change);

  /** New DataSaverCommand added to wm. */
  void newDataSaverCommand(const cast::cdl::WorkingMemoryChange & change);


private: // Commands

  void startSavingData(std::string dirPath, std::string baseName, int targetNo, std::string targetName);
  void stopSavingData();
  void pauseSavingData();
  void unpauseSavingData();
  void setNewTarget(int targetNo, std::string targetName);

  /** Overwrites the status on the WM with a new one. */
  void sendNewDataSaverStatus(bool savedImage, bool savedLaserScan, bool savedOdometry, long framesSaved, long frameNo, bool error);

  /** Creates an empty status on the WM. */
  std::string addEmptyDataSaverStatus();

  /** Creates an empty ack on the WM. */
  std::string addEmptyDataSaverCommandAck();

  void sendDpUpdateCommand();


private:

  /** If true, data should be saved.*/
  bool _saveData;

  /** Directory to which the saving thread saves data. */
  std::string _saveDirPath;

  /** Base of the saved data config file name. */
  std::string _saveBaseName;

  int _targetNo;
  std::string _targetName;

  /** Protects saveDirPath and saveBaseName. */
  pthread_mutex_t _saveMutex;

  /** Address of the available image in the WM. */
  cast::cdl::WorkingMemoryAddress _saveImageAddress;
  cast::cdl::WorkingMemoryAddress _saveScanAddress;
  cast::cdl::WorkingMemoryAddress _saveOdometryAddress;

  /** If true there is an image to grab from WM. */
  bool _wasSaveImageSignal;
  bool _wasSaveScanSignal; 
  bool _wasSaveOdometrySignal;

  pthread_cond_t _saveSignalCond;
  pthread_mutex_t _saveSignalMutex;

  bool _imagesSaved;
  bool _scansSaved;
  bool _odometrySaved;

  /** Number of frames saved. */
  long _framesSaved;

  std::string _dataSaverStatusId;
  std::string _dataSaverCommandAckId;

  /** When true, the update will be sent after data read.*/
  bool _updateOnRead;


  /** Name of the config file group. */
  const std::string _cfgGroup;

  int _updateDelay;

};


#endif

