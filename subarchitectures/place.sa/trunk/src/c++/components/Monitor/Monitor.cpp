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
#include "shared/ConfigFile.h"
#include "shared/CastTools.h"
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
using boost::shared_ptr;


// ------------------------------------------------------
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new PlaceMonitor();
  }
}


// ------------------------------------------------------
PlaceMonitor::PlaceMonitor(): _cfgGroup("Monitor")
{
  _qApp=0;
  _dialog=0;
}


// ------------------------------------------------------
PlaceMonitor::~PlaceMonitor()
{
}


// ------------------------------------------------------
void PlaceMonitor::configure(const map<string,string> &config)
{
  // Read cmd line options
  map<string, string>::const_iterator it = config.find("--config");
  string configFile;
  if (it!=config.end())
    configFile = it->second;
  it = config.find("--nogui");
  _gui=!(((it!=config.end()) && (it->second == "true")));
  string labelFile="";

  // Read config file
  ConfigFile cf;
  if (configFile.empty())
    println("No configuration file provided. Using defaults instead.");
  else
  {
    if (!cf.read(configFile))
      throw(CASTException(exceptionMessage(__HERE__, "Unable to load configuration file '%s'!", configFile.c_str() )));
  }

  // Get configuration from the file
  try
  {
    labelFile=cf.getStrValue("Common", "LabelFile", "");
  }
  catch(bad_lexical_cast &)
  {
    throw(CASTException(exceptionMessage(__HERE__, "Incorrect item value in the config file '%s'!", configFile.c_str() )));
  }

  // Read label file
  if (labelFile.empty())
  {
    throw(CASTException(exceptionMessage(__HERE__, "No label file provided! Use the LabelFile option in the config file!" )));
  }
  else
  {
    if (!_labels.read(labelFile))
      throw(CASTException(exceptionMessage(__HERE__, "Unable to load the label file '%s'!", labelFile.c_str() )));

  }

  // Print the configuration
  log("Configuration:");
  log("-> GUI: %s", (_gui)?"on":"off");
}


// ------------------------------------------------------
void PlaceMonitor::start()
{
  // wmChange
  addChangeFilter(createChangeFilter("", // All types
                                     cdl::WILDCARD, // All operations
                                     "", // All sources
                                     "", // All IDs
                                     "",
                                     cdl::LOCALSA // Only on local SA
                                     ),
     new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::wmChange) );

  addChangeFilter(createLocalTypeFilter<PlaceData::Image>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newImage) );

  addChangeFilter(createLocalTypeFilter<PlaceData::LaserScan>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newLaserScan) );

  addChangeFilter(createLocalTypeFilter<PlaceData::Odometry>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newOdometry) );

  addChangeFilter(createLocalTypeFilter<PlaceData::Target>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newTarget) );

  addChangeFilter(createLocalTypeFilter<PlaceData::DataProviderCommandAck>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newDataProviderCommandAck) );

  addChangeFilter(createLocalTypeFilter<PlaceData::DataSaverCommandAck>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newDataSaverCommandAck) );

  addChangeFilter(createLocalTypeFilter<PlaceData::DataSaverStatus>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newDataSaverStatus) );

  addChangeFilter(createLocalTypeFilter<PlaceData::VisualProcessorCommandAck>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newVisualProcessorCommandAck) );

  addChangeFilter(createLocalTypeFilter<PlaceData::VisualProcessorStatus>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newVisualProcessorStatus) );

  addChangeFilter(createLocalTypeFilter<PlaceData::VisualResults>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newVisualResults) );

  addChangeFilter(createLocalTypeFilter<PlaceData::LaserProcessorCommandAck>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newLaserProcessorCommandAck) );

  addChangeFilter(createLocalTypeFilter<PlaceData::LaserProcessorStatus>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newLaserProcessorStatus) );

  addChangeFilter(createLocalTypeFilter<PlaceData::LaserResults>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newLaserResults) );

  addChangeFilter(createLocalTypeFilter<PlaceData::IntegratedResults>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newIntegratedResults) );

  addChangeFilter(createLocalTypeFilter<PlaceData::NodeLabellerData>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceMonitor>(this, &PlaceMonitor::newNodeLabellerData) );

  debug("Started!");
}


// ------------------------------------------------------
void PlaceMonitor::stop()
{
  debug("Stop!");
}


// ------------------------------------------------------
void PlaceMonitor::runComponent()
{
  debug("Running...");
  // set_priority(PRIORITY_HIGH); Couldn't find counterpart in CAST 2.0

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
  throw(CASTException(exceptionMessage(__HERE__, "Not implemented! Use GUI instead!" )));
}


// ------------------------------------------------------
void PlaceMonitor::startGuiThread()
{
  pthread_attr_t thrAttr;
  pthread_attr_init(&thrAttr);

  int retval=pthread_attr_setinheritsched(&thrAttr, PTHREAD_EXPLICIT_SCHED);
  if (retval)
    throw(CASTException(exceptionMessage(__HERE__, "Cannot set inherit params for the GUI thread." )));
  retval=pthread_attr_setscope(&thrAttr, PTHREAD_SCOPE_SYSTEM);
  if (retval)
    throw(CASTException(exceptionMessage(__HERE__, "Cannot set scope of the GUI thread." )));
  retval=pthread_attr_setschedpolicy(&thrAttr, SCHED_FIFO);
  if (retval)
    throw(CASTException(exceptionMessage(__HERE__, "Cannot set policy of the GUI thread." )));
  struct sched_param thrParam;
  retval=pthread_attr_getschedparam(&thrAttr, &thrParam);
  if (retval)
    throw(CASTException(exceptionMessage(__HERE__, "Cannot get scheduling params of the GUI thread." )));
  thrParam.sched_priority = sched_get_priority_max(SCHED_FIFO);
  retval=pthread_attr_setschedparam(&thrAttr, &thrParam);
  if (retval)
    throw(CASTException(exceptionMessage(__HERE__, "Cannot set scheduling params of the GUI thread." )));
  // Creating the thread
  retval=pthread_create(&_guiThread, &thrAttr, &guiThreadEntryPoint, reinterpret_cast<void*>(this));
  if (retval)
    throw(CASTException(exceptionMessage(__HERE__, "Cannot create the GUI thread!" )));
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
  cdl::CASTTime time = getCASTTime();
  if (_dialog)
  {
    // Send the change to the dialog
    _dialog->addWmChange(change.operation, change.address.id,
        change.src, change.type, castTimeToSeconds(time));
  }
}


// ------------------------------------------------------
void PlaceMonitor::newDataProviderCommandAck(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  cdl::CASTTime time = getCASTTime();
  shared_ptr<CASTData<PlaceData::DataProviderCommandAck> > cmdCast =
      getWorkingMemoryEntry<PlaceData::DataProviderCommandAck>(change.address);
  if (!cmdCast)
    return;
  const PlaceData::DataProviderCommandAckPtr cmd = cmdCast->getData();

  // Convert the command id to name
  std::string cmdName="UNKNOWN";
  if (cmd->cmd->cmd==PlaceData::DpCmdUpdate)
    cmdName="DpCmdUpdate";

  if (_dialog)
  {
    _dialog->newCommand(cmdName, "", cmd->src, castTimeToSeconds(time));
  }
}


// ------------------------------------------------------
void PlaceMonitor::newDataSaverCommandAck(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  cdl::CASTTime time = getCASTTime();
  shared_ptr<CASTData<PlaceData::DataSaverCommandAck> > cmdCast =
      getWorkingMemoryEntry<PlaceData::DataSaverCommandAck>(change.address);
  if (!cmdCast)
    return;
  const PlaceData::DataSaverCommandAckPtr cmd=cmdCast->getData();

  // Convert the command id to name
  string cmdName="UNKNOWN";
  string cmdParams="";
  if (cmd->cmd->cmd==PlaceData::DsCmdStart)
    cmdName="DsCmdStart";
  else if (cmd->cmd->cmd==PlaceData::DsCmdUpdateStart)
    cmdName="DsCmdUpdateStart";
  else if (cmd->cmd->cmd==PlaceData::DsCmdUpdateStop)
    cmdName="DsCmdUpdateStop";
  else if (cmd->cmd->cmd==PlaceData::DsCmdStop)
    cmdName="DsCmdStop";
  else if (cmd->cmd->cmd==PlaceData::DsCmdPause)
    cmdName="DsCmdPause";
  else if (cmd->cmd->cmd==PlaceData::DsCmdUnpause)
    cmdName="DsCmdUnpause";
  else if (cmd->cmd->cmd==PlaceData::DsCmdNewTarget)
  {
    cmdName="DsCmdNewTarget";
    cmdParams="Target: "+boost::lexical_cast<string>(cmd->cmd->targetNo)+" "+cmd->cmd->targetName;
  }

  if (_dialog)
  {
    _dialog->newCommand(cmdName, cmdParams, cmd->src, castTimeToSeconds(time));
  }
}


// ------------------------------------------------------
void PlaceMonitor::newVisualProcessorCommandAck(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  cdl::CASTTime time = getCASTTime();
  shared_ptr<CASTData<PlaceData::VisualProcessorCommandAck> > cmdCast =
      getWorkingMemoryEntry<PlaceData::VisualProcessorCommandAck>(change.address);
  if (!cmdCast)
    return;
  const PlaceData::VisualProcessorCommandAckPtr cmd=cmdCast->getData();

  // Convert the command id to name
  string cmdName="UNKNOWN";
  if (cmd->cmd->cmd==PlaceData::VpCmdUpdateOnReadStart)
    cmdName="VpCmdUpdateOnReadStart";
  else if (cmd->cmd->cmd==PlaceData::VpCmdUpdateStop)
    cmdName="VpCmdUpdateStop";

  if (_dialog)
  {
    _dialog->newCommand(cmdName, "", cmd->src, castTimeToSeconds(time));
  }
}


// ------------------------------------------------------
void PlaceMonitor::newVisualProcessorStatus(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  shared_ptr<CASTData<PlaceData::VisualProcessorStatus> > statCast =
      getWorkingMemoryEntry<PlaceData::VisualProcessorStatus>(change.address);
  if (!statCast)
    return;
  const PlaceData::VisualProcessorStatusPtr stat = statCast->getData();

  if (_dialog)
  {
    _dialog->newVisualProcessorStatus(stat);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newVisualResults(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  shared_ptr<CASTData<PlaceData::VisualResults> > resCast =
      getWorkingMemoryEntry<PlaceData::VisualResults>(change.address);
  if (!resCast)
    return;
  const PlaceData::VisualResultsPtr res = resCast->getData();

  if (_dialog)
  {
    _dialog->newVisualResults(res);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newLaserProcessorCommandAck(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  cdl::CASTTime time = getCASTTime();
  shared_ptr<CASTData<PlaceData::LaserProcessorCommandAck> > cmdCast =
      getWorkingMemoryEntry<PlaceData::LaserProcessorCommandAck>(change.address);
  if (!cmdCast)
    return;
  const PlaceData::LaserProcessorCommandAckPtr cmd=cmdCast->getData();

  // Convert the command id to name
  std::string cmdName="UNKNOWN";
  if (cmd->cmd->cmd==PlaceData::LpCmdUpdateOnReadStart)
    cmdName="LpCmdUpdateOnReadStart";
  else if (cmd->cmd->cmd==PlaceData::LpCmdUpdateStop)
    cmdName="LpCmdUpdateStop";

  if (_dialog)
  {
    _dialog->newCommand(cmdName, "", cmd->src, castTimeToSeconds(time));
  }
}


// ------------------------------------------------------
void PlaceMonitor::newLaserProcessorStatus(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  shared_ptr<CASTData<PlaceData::LaserProcessorStatus> > statCast =
      getWorkingMemoryEntry<PlaceData::LaserProcessorStatus>(change.address);
  if (!statCast)
    return;
  const PlaceData::LaserProcessorStatusPtr stat = statCast->getData();

  if (_dialog)
  {
    _dialog->newLaserProcessorStatus(stat);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newLaserResults(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  shared_ptr<CASTData<PlaceData::LaserResults> > resCast =
      getWorkingMemoryEntry<PlaceData::LaserResults>(change.address);
  if (!resCast)
    return;
  const PlaceData::LaserResultsPtr res = resCast->getData();

  if (_dialog)
  {
    _dialog->newLaserResults(res);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newIntegratedResults(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  shared_ptr<CASTData<PlaceData::IntegratedResults> > resCast =
      getWorkingMemoryEntry<PlaceData::IntegratedResults>(change.address);
  if (!resCast)
    return;
  const PlaceData::IntegratedResultsPtr res = resCast->getData();

  if (_dialog)
  {
    _dialog->newIntegratedResults(res);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newNodeLabellerData(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  shared_ptr<CASTData<PlaceData::NodeLabellerData> > infoCast =
      getWorkingMemoryEntry<PlaceData::NodeLabellerData>(change.address);
  if (!infoCast)
    return;
  const PlaceData::NodeLabellerDataPtr info = infoCast->getData();

  if (_dialog)
  {
    _dialog->newNodeLabellerData(info);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newDataSaverStatus(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  cdl::CASTTime time = getCASTTime();
  shared_ptr<CASTData<PlaceData::DataSaverStatus> > statusCast =
      getWorkingMemoryEntry<PlaceData::DataSaverStatus>(change.address);
  if (!statusCast)
    return;
  const PlaceData::DataSaverStatusPtr status=statusCast->getData();

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
  shared_ptr<CASTData<PlaceData::Image> > imageCast =
      getWorkingMemoryEntry<PlaceData::Image>(change.address);
  if (!imageCast)
    return;
  const PlaceData::ImagePtr image = imageCast->getData();

  if (_dialog)
  {
    _dialog->newImage(image);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newLaserScan(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  shared_ptr<CASTData<PlaceData::LaserScan> > laserScanCast =
      getWorkingMemoryEntry<PlaceData::LaserScan>(change.address);
  if (!laserScanCast)
    return;
  const PlaceData::LaserScanPtr laserScan = laserScanCast->getData();

  if (_dialog)
  {
    _dialog->newLaserScan(laserScan);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newOdometry(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  shared_ptr<CASTData<PlaceData::Odometry> > odometryCast =
      getWorkingMemoryEntry<PlaceData::Odometry>(change.address);
  if (!odometryCast)
    return;
  PlaceData::OdometryPtr odometry =
      new PlaceData::Odometry( *odometryCast->getData() );

  if (_dialog)
  {
    _dialog->newOdometry(odometry);
  }
}


// ------------------------------------------------------
void PlaceMonitor::newTarget(const cast::cdl::WorkingMemoryChange & change)
{
  // Get
  shared_ptr<CASTData<PlaceData::Target> > targetCast =
      getWorkingMemoryEntry<PlaceData::Target>(change.address);
  if (!targetCast)
    return;
  const PlaceData::TargetPtr target = targetCast->getData();

  if (_dialog)
  {
    _dialog->newTarget(target);
  }
}


// ------------------------------------------------------
void PlaceMonitor::sendPlaceCommand(PlaceData::PlaceCommandType cmd)
{
  PlaceData::PlaceCommandPtr placeCommand = new PlaceData::PlaceCommand;
  placeCommand->cmd = cmd;
  string dataId = newDataID();
  addToWorkingMemory(dataId, placeCommand);
  if (cmd==PlaceData::CmdUpdate)
    println("Sent CmdUpdate command.");
  else if (cmd==PlaceData::CmdStart)
    println("Sent CmdStart command.");
  else if (cmd==PlaceData::CmdStop)
    println("Sent CmdStop command.");
  else
    println("Sent unknown command.");
}


// ------------------------------------------------------
void PlaceMonitor::sendDataProviderCommand(PlaceData::DataProviderCommandType cmd)
{
  PlaceData::DataProviderCommandPtr dataProviderCommand =
      new PlaceData::DataProviderCommand;
  dataProviderCommand->cmd = cmd;
  string dataId = newDataID();
  addToWorkingMemory(dataId, dataProviderCommand);
  if (cmd==PlaceData::DpCmdUpdate)
    println("Sent DpCmdUpdate command.");
  else
    println("Sent unknown command.");
}


// ------------------------------------------------------
void PlaceMonitor::sendVisualProcessorCommand(PlaceData::VisualProcessorCommandType cmd)
{
  PlaceData::VisualProcessorCommandPtr visualProcessorCommand =
      new PlaceData::VisualProcessorCommand;
  visualProcessorCommand->cmd = cmd;
  string dataId = newDataID();
  addToWorkingMemory(dataId, visualProcessorCommand);
  if (cmd==PlaceData::VpCmdUpdateOnReadStart)
    println("Sent VpCmdUpdateOnReadStart command.");
  if (cmd==PlaceData::VpCmdUpdateStop)
    println("Sent VpCmdUpdateStop command.");
  else
    println("Sent unknown command.");
}


// ------------------------------------------------------
void PlaceMonitor::sendDataSaverCommand(PlaceData::DataSaverCommandType cmd, std::string dirPath, std::string baseName, int targetNo, std::string targetName)
{
  PlaceData::DataSaverCommandPtr dsCommand = new PlaceData::DataSaverCommand;
  dsCommand->cmd = cmd;
  dsCommand->dataDirPath=dirPath.c_str();
  dsCommand->dataBaseName=baseName.c_str();
  dsCommand->targetNo=(long)targetNo;
  dsCommand->targetName=targetName.c_str();
  string dataId = newDataID();
  addToWorkingMemory(dataId, dsCommand);
  if (cmd==PlaceData::DsCmdStart)
    println("Sent DsCmdStart command.");
  else if (cmd==PlaceData::DsCmdUpdateStart)
    println("Sent DsCmdUpdateStart command.");
  else if (cmd==PlaceData::DsCmdUpdateStop)
    println("Sent DsCmdUpdateStop command.");
  else if (cmd==PlaceData::DsCmdStop)
    println("Sent DsCmdStop command.");
  else if (cmd==PlaceData::DsCmdPause)
    println("Sent DsCmdPause command.");
  else if (cmd==PlaceData::DsCmdUnpause)
    println("Sent DsCmdUnpause command.");
  else if (cmd==PlaceData::DsCmdNewTarget)
    println("Sent DsCmdNewTarget command.");
  else
    println("Sent unknown command.");
}


