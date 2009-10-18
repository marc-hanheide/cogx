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
 * PlaceMonitor class.
 * \file Monitor.h
 * \author Andrzej Pronobis
 * \date 2008-08-12
 */

#ifndef __PLACE_MONITOR__
#define __PLACE_MONITOR__

#include "place/shared/LabelFile.h"
#include "place/shared/NodePlaceInfoPullSender.h"
#include <place/idl/PlaceData.hh>
#include <cast/architecture/ManagedProcess.hpp>

namespace place 
{
  class MonitorDialog;
}

class QApplication;

/**
 * Implements the Monitor component
 */
class PlaceMonitor: public cast::ManagedProcess,
                    public place::NodePlaceInfoPullSender
{
public: // Component management

  /** Constructor. */
  PlaceMonitor(const std::string &_id);

  /** Destructor. */
  virtual ~PlaceMonitor();

  /** Handles component configuration. */
  virtual void configure(std::map<std::string,std::string> &_config);

  /** Main thread of the component. */
  virtual void runComponent();

  /** Invoked on start. */
  virtual void start();

  /** Invoked on stop. */
  virtual void stop();


public: // Public monitor interace

  void sendPlaceCommand(PlaceData::PlaceCommandType cmd);
  void sendDataProviderCommand(PlaceData::DataProviderCommandType cmd);
  void sendVisualProcessorCommand(PlaceData::VisualProcessorCommandType cmd);
  void sendDataSaverCommand(PlaceData::DataSaverCommandType cmd, std::string dirPath, std::string baseName, int targetNo, std::string targetName);

  bool isStatusRun()
    { return m_status == STATUS_RUN; }

  const place::LabelFile &getLabels()
  { return _labels; }


protected: // Pure virtual

  /** Needed as pure virtual in the parent class. */
  virtual void taskAdopted(const std::string &_taskID)
    {}

  /** Needed as pure virtual in the parent class. */
  virtual void taskRejected(const std::string &_taskID)
    {}


private:

  void wmChange(const cast::cdl::WorkingMemoryChange & change);

  /** New Image added to wm. */
  void newImage(const cast::cdl::WorkingMemoryChange & change);

  /** New LaserScan added to wm. */
  void newLaserScan(const cast::cdl::WorkingMemoryChange & change);

  /** New Odometry added to wm. */
  void newOdometry(const cast::cdl::WorkingMemoryChange & change);

  /** New Target added to wm. */
  void newTarget(const cast::cdl::WorkingMemoryChange & change);

  /** New DataProviderCommandAck added to wm. */
  void newDataProviderCommandAck(const cast::cdl::WorkingMemoryChange & change);

  /** New DataSaverCommandAck added to wm. */
  void newDataSaverCommandAck(const cast::cdl::WorkingMemoryChange & change);

  /** New DataSaverStatus on the wm. */
  void newDataSaverStatus(const cast::cdl::WorkingMemoryChange & change);

  /** New VisualProcessorCommandAck added to wm. */
  void newVisualProcessorCommandAck(const cast::cdl::WorkingMemoryChange & change);

  /** New VisualProcessorStatus added to wm. */
  void newVisualProcessorStatus(const cast::cdl::WorkingMemoryChange & change);

  /** New VisualResults added to wm. */
  void newVisualResults(const cast::cdl::WorkingMemoryChange & change);

  /** New LaserProcessorCommandAck added to wm. */
  void newLaserProcessorCommandAck(const cast::cdl::WorkingMemoryChange & change);

  /** New LaserProcessorStatus added to wm. */
  void newLaserProcessorStatus(const cast::cdl::WorkingMemoryChange & change);

  /** New LaserResults added to wm. */
  void newLaserResults(const cast::cdl::WorkingMemoryChange & change);

  /** New IntegratedResults added to wm. */
  void newIntegratedResults(const cast::cdl::WorkingMemoryChange & change);

  /** New NodeLabellerData added to wm. */
  void newNodeLabellerData(const cast::cdl::WorkingMemoryChange & change);


private: // Interface


  /* Entry point for the gui thread. */
  static void *guiThreadEntryPoint(void *monitor);
  void startGuiThread();
  pthread_t _guiThread;

  void runGui();
  void runConsole();

private:

  /** Name of the config file group. */
  const std::string _cfgGroup;

  /** True if GUI should be used. */
  bool _gui;

  /** QT GUI dialog. */
  place::MonitorDialog *_dialog;

  QApplication *_qApp;

  /** List of labels and names. */
  place::LabelFile _labels;

};


#endif

