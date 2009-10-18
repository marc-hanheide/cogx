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
 * \file Monitor.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-12
 */

// Place.SA
#include "Monitor.h"
#include "MonitorDialog.h"
#include "place/shared/ConfigFile.h"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>
// Qt
#include <QApplication>
// Boost
#include <boost/lexical_cast.hpp>


using namespace std;
using namespace cast;
using namespace place;
using boost::bad_lexical_cast;


// ------------------------------------------------------
extern "C" 
{
  FrameworkProcess* newComponent(const string &_id)
  {
    return new PlaceMonitor(_id);
  }
}


// ------------------------------------------------------
PlaceMonitor::PlaceMonitor(const string &_id):
                        WorkingMemoryAttachedComponent(_id),
                        ManagedProcess(_id),
                        _cfgGroup("Monitor")
{
  _qApp=0;
  _dialog=0;
}


// ------------------------------------------------------
PlaceMonitor::~PlaceMonitor()
{
}


// ------------------------------------------------------
void PlaceMonitor::configure(map<string,string> &_config)
{
  ManagedProcess::configure(_config);

  // Read cmd line options
  string configFile=_config["--config"];
  _gui=!(_config["--nogui"]=="true");
  string labelFile="";

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
  log("-> GUI: %s", (_gui)?"on":"off");
}


// ------------------------------------------------------
void PlaceMonitor::start()
{
  ManagedProcess::start();

  // wmChange
  MemberFunctionChangeReceiver<PlaceMonitor> * pReceiver =
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::wmChange);

  addChangeFilter( createChangeFilter("", // All types
                                      cdl::WILDCARD, // All operations
                                      "", // All sources
                                      "", // All IDs
                                      "",
                                      cdl::LOCAL_SA // Only on local SA
                                     ),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newImage);
  addChangeFilter( createLocalTypeFilter<PlaceData::Image>(cdl::OVERWRITE),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newLaserScan);
  addChangeFilter( createLocalTypeFilter<PlaceData::LaserScan>(cdl::OVERWRITE),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newOdometry);
  addChangeFilter( createLocalTypeFilter<PlaceData::Odometry>(cdl::OVERWRITE),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newTarget);
  addChangeFilter( createLocalTypeFilter<PlaceData::Target>(cdl::OVERWRITE),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newDataProviderCommandAck);
  addChangeFilter( createLocalTypeFilter<PlaceData::DataProviderCommandAck>(cdl::OVERWRITE),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newDataSaverCommandAck);
  addChangeFilter( createLocalTypeFilter<PlaceData::DataSaverCommandAck>(cdl::OVERWRITE),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newDataSaverStatus);
  addChangeFilter( createLocalTypeFilter<PlaceData::DataSaverStatus>(cdl::OVERWRITE),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newVisualProcessorCommandAck);
  addChangeFilter( createLocalTypeFilter<PlaceData::VisualProcessorCommandAck>(cdl::OVERWRITE),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newVisualProcessorStatus);
  addChangeFilter( createLocalTypeFilter<PlaceData::VisualProcessorStatus>(cdl::OVERWRITE),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newVisualResults);
  addChangeFilter( createLocalTypeFilter<PlaceData::VisualResults>(cdl::OVERWRITE),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newLaserProcessorCommandAck);
  addChangeFilter( createLocalTypeFilter<PlaceData::LaserProcessorCommandAck>(cdl::OVERWRITE),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newLaserProcessorStatus);
  addChangeFilter( createLocalTypeFilter<PlaceData::LaserProcessorStatus>(cdl::OVERWRITE),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newLaserResults);
  addChangeFilter( createLocalTypeFilter<PlaceData::LaserResults>(cdl::OVERWRITE),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newIntegratedResults);
  addChangeFilter( createLocalTypeFilter<PlaceData::IntegratedResults>(cdl::OVERWRITE),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newNodeLabellerData);
  addChangeFilter( createLocalTypeFilter<PlaceData::NodeLabellerData>(cdl::OVERWRITE),
                   pReceiver );


  debug("Started!");
}


// ------------------------------------------------------
void PlaceMonitor::stop()
{
  ManagedProcess::stop();

  debug("Stop!");
}


// ------------------------------------------------------
void PlaceMonitor::runComponent()
{
  debug("Running...");
  set_priority(PRIORITY_HIGH);

  if (_gui)
    //startGuiThread();
    runGui();
  else
    runConsole();

  debug("Completed!");
}


// ------------------------------------------------------
void PlaceMonitor::runConsole()
{
  throw CASTException(__HERE__, "Not implemented! Use GUI instead!");
}


// ------------------------------------------------------
void PlaceMonitor::startGuiThread()
{
  pthread_attr_t thrAttr;
  pthread_attr_init(&thrAttr);

  int retval=pthread_attr_setinheritsched(&thrAttr, PTHREAD_EXPLICIT_SCHED);
  if (retval)
    throw CASTException(__HERE__, "Cannot set inherit params for the GUI thread.");
  retval=pthread_attr_setscope(&thrAttr, PTHREAD_SCOPE_SYSTEM);
  if (retval)
    throw CASTException(__HERE__, "Cannot set scope of the GUI thread.");
  retval=pthread_attr_setschedpolicy(&thrAttr, SCHED_FIFO);
  if (retval)
    throw CASTException(__HERE__, "Cannot set policy of the GUI thread.");
  struct sched_param thrParam;
  retval=pthread_attr_getschedparam(&thrAttr, &thrParam);
  if (retval)
    throw CASTException(__HERE__, "Cannot get scheduling params of the GUI thread.");
  thrParam.sched_priority = sched_get_priority_max(SCHED_FIFO);
  retval=pthread_attr_setschedparam(&thrAttr, &thrParam);
  if (retval)
    throw CASTException(__HERE__, "Cannot set scheduling params of the GUI thread.");
  // Creating the thread
  retval=pthread_create(&_guiThread, &thrAttr, &guiThreadEntryPoint, reinterpret_cast<void*>(this));
  if (retval)
    throw CASTException(__HERE__, "Cannot create the GUI thread!");
}


// ------------------------------------------------------
void *PlaceMonitor::guiThreadEntryPoint(void *monitor)
{
  reinterpret_cast<PlaceMonitor*>(monitor)->runGui();
  return 0;
}


// ------------------------------------------------------
void PlaceMonitor::runGui()
{
  // Create application
  _qApp = new QApplication(0,0);

  // Start dialog
  _dialog = new MonitorDialog(this);
  _dialog->exec();

  // Thread safe delete
  place::MonitorDialog *dialog=_dialog;
  QApplication *app=_qApp;
  _dialog=0;
  delete dialog;
  _qApp=0;
  delete app;

}


// ------------------------------------------------------
void PlaceMonitor::wmChange(const cast::cdl::WorkingMemoryChange & change)
{
  FrameworkBasics::BALTTime time = BALTTimer::getBALTTime();
  if (_dialog)
  {
    // Send the change to the dialog
    _dialog->addWmChange(change.m_operation, change.m_address.m_id, change.m_src, change.m_type, BALTTimer::toSeconds(time));
  }
}


// ------------------------------------------------------
void PlaceMonitor::newDataProviderCommandAck(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  FrameworkBasics::BALTTime time = BALTTimer::getBALTTime();
  boost::shared_ptr<const CASTTypedData<PlaceData::DataProviderCommandAck> > cmdTD =
      getWorkingMemoryEntry<PlaceData::DataProviderCommandAck>(change.m_address);
  if (!cmdTD)
    return;
  const boost::shared_ptr<const PlaceData::DataProviderCommandAck> cmd=cmdTD->getData();

  // Convert the command id to name
  std::string cmdName="UNKNOWN";
  if (cmd->cmd.cmd==PlaceData::DP_CMD_UPDATE)
    cmdName="DP_CMD_UPDATE";

  if (_dialog)
  {
    _dialog->newCommand(cmdName.c_str(), "", cmd->src, BALTTimer::toSeconds(time));
  }
}


// ------------------------------------------------------
void PlaceMonitor::newDataSaverCommandAck(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  FrameworkBasics::BALTTime time = BALTTimer::getBALTTime();
  boost::shared_ptr<const CASTTypedData<PlaceData::DataSaverCommandAck> > cmdTD =
      getWorkingMemoryEntry<PlaceData::DataSaverCommandAck>(change.m_address);
  if (!cmdTD)
    return;
  const boost::shared_ptr<const PlaceData::DataSaverCommandAck> cmd=cmdTD->getData();

  // Convert the command id to name
  std::string cmdName="UNKNOWN";
  std::string cmdParams="";
  if (cmd->cmd.cmd==PlaceData::DS_CMD_START)
    cmdName="DS_CMD_START";
  else if (cmd->cmd.cmd==PlaceData::DS_CMD_UPDATE_START)
    cmdName="DS_CMD_UPDATE_START";
  else if (cmd->cmd.cmd==PlaceData::DS_CMD_UPDATE_STOP)
    cmdName="DS_CMD_UPDATE_STOP";
  else if (cmd->cmd.cmd==PlaceData::DS_CMD_STOP)
    cmdName="DS_CMD_STOP";
  else if (cmd->cmd.cmd==PlaceData::DS_CMD_PAUSE)
    cmdName="DS_CMD_PAUSE";
  else if (cmd->cmd.cmd==PlaceData::DS_CMD_UNPAUSE)
    cmdName="DS_CMD_UNPAUSE";
  else if (cmd->cmd.cmd==PlaceData::DS_CMD_NEW_TARGET)
  {
    cmdName="DS_CMD_NEW_TARGET";
    cmdParams="Target: "+boost::lexical_cast<string>(cmd->cmd.targetNo)+" "+string(cmd->cmd.targetName);
  }

  if (_dialog)
  {
    _dialog->newCommand(cmdName.c_str(), cmdParams.c_str(), cmd->src, BALTTimer::toSeconds(time));
  }
}


// ------------------------------------------------------
void PlaceMonitor::newVisualProcessorCommandAck(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  FrameworkBasics::BALTTime time = BALTTimer::getBALTTime();
  boost::shared_ptr<const CASTTypedData<PlaceData::VisualProcessorCommandAck> > cmdTD =
      getWorkingMemoryEntry<PlaceData::VisualProcessorCommandAck>(change.m_address);
  if (!cmdTD)
    return;
  const boost::shared_ptr<const PlaceData::VisualProcessorCommandAck> cmd=cmdTD->getData();

  // Convert the command id to name
  std::string cmdName="UNKNOWN";
  if (cmd->cmd.cmd==PlaceData::VP_CMD_UPDATE_ON_READ_START)
    cmdName="VP_CMD_UPDATE_ON_READ_START";
  else if (cmd->cmd.cmd==PlaceData::VP_CMD_UPDATE_STOP)
    cmdName="VP_CMD_UPDATE_STOP";

  if (_dialog)
  {
    _dialog->newCommand(cmdName.c_str(), "", cmd->src, BALTTimer::toSeconds(time));
  }
}


// ------------------------------------------------------
void PlaceMonitor::newVisualProcessorStatus(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  boost::shared_ptr<const CASTTypedData<PlaceData::VisualProcessorStatus> > statTD =
      getWorkingMemoryEntry<PlaceData::VisualProcessorStatus>(change.m_address);
  if (!statTD)
    return;
  const boost::shared_ptr<const PlaceData::VisualProcessorStatus> stat=statTD->getData();

  if (_dialog)
  {
    _dialog->newVisualProcessorStatus(*stat);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newVisualResults(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  boost::shared_ptr<const CASTTypedData<PlaceData::VisualResults> > resTD =
      getWorkingMemoryEntry<PlaceData::VisualResults>(change.m_address);
  if (!resTD)
    return;
  const boost::shared_ptr<const PlaceData::VisualResults> res=resTD->getData();

  if (_dialog)
  {
    _dialog->newVisualResults(*res);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newLaserProcessorCommandAck(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  FrameworkBasics::BALTTime time = BALTTimer::getBALTTime();
  boost::shared_ptr<const CASTTypedData<PlaceData::LaserProcessorCommandAck> > cmdTD =
      getWorkingMemoryEntry<PlaceData::LaserProcessorCommandAck>(change.m_address);
  if (!cmdTD)
    return;
  const boost::shared_ptr<const PlaceData::LaserProcessorCommandAck> cmd=cmdTD->getData();

  // Convert the command id to name
  std::string cmdName="UNKNOWN";
  if (cmd->cmd.cmd==PlaceData::LP_CMD_UPDATE_ON_READ_START)
    cmdName="LP_CMD_UPDATE_ON_READ_START";
  else if (cmd->cmd.cmd==PlaceData::LP_CMD_UPDATE_STOP)
    cmdName="LP_CMD_UPDATE_STOP";

  if (_dialog)
  {
    _dialog->newCommand(cmdName.c_str(), "", cmd->src, BALTTimer::toSeconds(time));
  }
}


// ------------------------------------------------------
void PlaceMonitor::newLaserProcessorStatus(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  boost::shared_ptr<const CASTTypedData<PlaceData::LaserProcessorStatus> > statTD =
      getWorkingMemoryEntry<PlaceData::LaserProcessorStatus>(change.m_address);
  if (!statTD)
    return;
  const boost::shared_ptr<const PlaceData::LaserProcessorStatus> stat=statTD->getData();

  if (_dialog)
  {
    _dialog->newLaserProcessorStatus(*stat);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newLaserResults(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  boost::shared_ptr<const CASTTypedData<PlaceData::LaserResults> > resTD =
      getWorkingMemoryEntry<PlaceData::LaserResults>(change.m_address);
  if (!resTD)
    return;
  const boost::shared_ptr<const PlaceData::LaserResults> res=resTD->getData();

  if (_dialog)
  {
    _dialog->newLaserResults(*res);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newIntegratedResults(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  boost::shared_ptr<const CASTTypedData<PlaceData::IntegratedResults> > resTD =
      getWorkingMemoryEntry<PlaceData::IntegratedResults>(change.m_address);
  if (!resTD)
    return;
  const boost::shared_ptr<const PlaceData::IntegratedResults> res=resTD->getData();

  if (_dialog)
  {
    _dialog->newIntegratedResults(*res);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newNodeLabellerData(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  boost::shared_ptr<const CASTTypedData<PlaceData::NodeLabellerData> > infoTD =
      getWorkingMemoryEntry<PlaceData::NodeLabellerData>(change.m_address);
  if (!infoTD)
    return;
  const boost::shared_ptr<const PlaceData::NodeLabellerData> info=infoTD->getData();

  if (_dialog)
  {
    _dialog->newNodeLabellerData(*info);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newDataSaverStatus(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  FrameworkBasics::BALTTime time = BALTTimer::getBALTTime();
  boost::shared_ptr<const CASTTypedData<PlaceData::DataSaverStatus> > statusTD =
      getWorkingMemoryEntry<PlaceData::DataSaverStatus>(change.m_address);
  if (!statusTD)
    return;
  const boost::shared_ptr<const PlaceData::DataSaverStatus> status=statusTD->getData();

  if (_dialog)
  {
    // Notify dialog
    _dialog->newDataSaverStatus(status->savedImage, status->savedLaserScan, status->savedOdometry, 
                                status->framesSaved, status->frameNo, status->error);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newImage(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  boost::shared_ptr<const CASTTypedData<PlaceData::Image> > imageTD =
      getWorkingMemoryEntry<PlaceData::Image>(change.m_address);
  if (!imageTD)
    return;
  const boost::shared_ptr<const PlaceData::Image> image=imageTD->getData();

  if (_dialog)
  {
    _dialog->newImage(*image);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newLaserScan(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  boost::shared_ptr<const CASTTypedData<PlaceData::LaserScan> > laserScanTD =
      getWorkingMemoryEntry<PlaceData::LaserScan>(change.m_address);
  if (!laserScanTD)
    return;
  const boost::shared_ptr<const PlaceData::LaserScan> laserScan=laserScanTD->getData();

  if (_dialog)
  {
    _dialog->newLaserScan(*laserScan);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newOdometry(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  boost::shared_ptr<const CASTTypedData<PlaceData::Odometry> > odometryTD =
      getWorkingMemoryEntry<PlaceData::Odometry>(change.m_address);
  if (!odometryTD)
    return;
  const boost::shared_ptr<const PlaceData::Odometry> odometry=odometryTD->getData();

  if (_dialog)
  {
    _dialog->newOdometry(*odometry);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newTarget(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  boost::shared_ptr<const CASTTypedData<PlaceData::Target> > targetTD =
      getWorkingMemoryEntry<PlaceData::Target>(change.m_address);
  if (!targetTD)
    return;
  const boost::shared_ptr<const PlaceData::Target> target=targetTD->getData();

  if (_dialog)
  {
    _dialog->newTarget(*target);
  }
}


// ------------------------------------------------------
void PlaceMonitor::sendPlaceCommand(PlaceData::PlaceCommandType cmd)
{
  PlaceData::PlaceCommand *placeCommand = new PlaceData::PlaceCommand;
  placeCommand->cmd = cmd;
  string dataId = newDataID();
  addToWorkingMemory<PlaceData::PlaceCommand>(dataId, placeCommand);
  if (cmd==PlaceData::CMD_UPDATE)
    println("Sent CMD_UPDATE command.");
  else if (cmd==PlaceData::CMD_START)
    println("Sent CMD_START command.");
  else if (cmd==PlaceData::CMD_STOP)
    println("Sent CMD_STOP command.");
  else
    println("Sent unknown command.");
}


// ------------------------------------------------------
void PlaceMonitor::sendDataProviderCommand(PlaceData::DataProviderCommandType cmd)
{
  PlaceData::DataProviderCommand *dataProviderCommand = new PlaceData::DataProviderCommand;
  dataProviderCommand->cmd = cmd;
  string dataId = newDataID();
  addToWorkingMemory<PlaceData::DataProviderCommand>(dataId, dataProviderCommand);
  if (cmd==PlaceData::DP_CMD_UPDATE)
    println("Sent DP_CMD_UPDATE command.");
  else
    println("Sent unknown command.");
}


// ------------------------------------------------------
void PlaceMonitor::sendVisualProcessorCommand(PlaceData::VisualProcessorCommandType cmd)
{
  PlaceData::VisualProcessorCommand *visualProcessorCommand = new PlaceData::VisualProcessorCommand;
  visualProcessorCommand->cmd = cmd;
  string dataId = newDataID();
  addToWorkingMemory<PlaceData::VisualProcessorCommand>(dataId, visualProcessorCommand);
  if (cmd==PlaceData::VP_CMD_UPDATE_ON_READ_START)
    println("Sent VP_CMD_UPDATE_ON_READ_START command.");
  if (cmd==PlaceData::VP_CMD_UPDATE_STOP)
    println("Sent VP_CMD_UPDATE_STOP command.");
  else
    println("Sent unknown command.");
}


// ------------------------------------------------------
void PlaceMonitor::sendDataSaverCommand(PlaceData::DataSaverCommandType cmd, std::string dirPath, std::string baseName, int targetNo, std::string targetName)
{
  PlaceData::DataSaverCommand *dsCommand = new PlaceData::DataSaverCommand;
  dsCommand->cmd = cmd;
  dsCommand->dataDirPath=dirPath.c_str();
  dsCommand->dataBaseName=baseName.c_str();
  dsCommand->targetNo=(long)targetNo;
  dsCommand->targetName=targetName.c_str();
  string dataId = newDataID();
  addToWorkingMemory<PlaceData::DataSaverCommand>(dataId, dsCommand);
  if (cmd==PlaceData::DS_CMD_START)
    println("Sent DS_CMD_START command.");
  else if (cmd==PlaceData::DS_CMD_UPDATE_START)
    println("Sent DS_CMD_UPDATE_START command.");
  else if (cmd==PlaceData::DS_CMD_UPDATE_STOP)
    println("Sent DS_CMD_UPDATE_STOP command.");
  else if (cmd==PlaceData::DS_CMD_STOP)
    println("Sent DS_CMD_STOP command.");
  else if (cmd==PlaceData::DS_CMD_PAUSE)
    println("Sent DS_CMD_PAUSE command.");
  else if (cmd==PlaceData::DS_CMD_UNPAUSE)
    println("Sent DS_CMD_UNPAUSE command.");
  else if (cmd==PlaceData::DS_CMD_NEW_TARGET)
    println("Sent DS_CMD_NEW_TARGET command.");
  else
    println("Sent unknown command.");
}


