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
 * PlaceController class.
 * \file Controller.h
 * \author Andrzej Pronobis
 * \date 2008-09-02
 */

// Place.SA
#include "Controller.h"
#include "shared/ConfigFile.h"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>
// Boost
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace place;
using namespace cast;
using boost::bad_lexical_cast;
using boost::lexical_cast;
using boost::shared_ptr;


// ------------------------------------------------------
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new PlaceController();
  }
}


// ------------------------------------------------------
PlaceController::PlaceController(): _cfgGroup("Controller")
{
  _start=false;
  _updatesSentOn=VP_READ;
}


// ------------------------------------------------------
PlaceController::~PlaceController()
{
}


// ------------------------------------------------------
void PlaceController::configure(const std::map<std::string, std::string> &config)
{
  // Read cmd line options
  map<string, string>::const_iterator it = config.find("--config");
  string configFile;
  if (it!=config.end())
    configFile = it->second;
  it = config.find("--start");
  _start=((it!=config.end()) && (it->second == "true"));

  // Read config file
  ConfigFile cf;
  if (configFile.empty())
    println("No configuration file provided. Using defaults instead.");
  else
  {
    if (!cf.read(configFile))
    {
      throw(CASTException(exceptionMessage(__HERE__,
          "Unable to load configuration file '%s'!", configFile.c_str() )));
    }
  }

  // Get configuration from the file
  string updatesSentOnStr;
  try
  {
    updatesSentOnStr=cf.getStrValue(_cfgGroup, "UpdatesSentOn", "VP_READ");
  }
  catch(bad_lexical_cast &)
  {
    throw(CASTException(exceptionMessage(__HERE__,
        "Incorrect item value in the config file '%s'!", configFile.c_str() )));
  }
  // Check the values and decode
  if (updatesSentOnStr=="VP_READ")
    _updatesSentOn=VP_READ;
  else if (updatesSentOnStr=="DS_READ")
    _updatesSentOn=DS_READ;
  else
    throw(CASTException(exceptionMessage(__HERE__,
        "Incorrect value of UpdatesSentOn in the config file '%s'!", configFile.c_str() )));

  // Print the configuration
  log("Configuration:");
  log("-> Updates sent on: %s", updatesSentOnStr.c_str());
  log("-> Automatic start: %s", (_start)?"yes":"no");
}


// ------------------------------------------------------
void PlaceController::start()
{
  // Add change filters
  addChangeFilter(createLocalTypeFilter<PlaceData::PlaceCommand>(cdl::ADD),
	                  new MemberFunctionChangeReceiver<PlaceController>(this,
	                                        &PlaceController::newPlaceCommand));
}


// ------------------------------------------------------
void PlaceController::stop()
{
}


// ------------------------------------------------------
void PlaceController::runComponent()
{
  // Wait 1 second
  usleep(1000000);

  // If we are supposed to start, sent the proper commands
  if (_start)
  {
    switch(_updatesSentOn)
    {
      case VP_READ:
        sendVisualProcessorCommand(PlaceData::VpCmdUpdateOnReadStart);
        break;
      case DS_READ:
        sendDataSaverCommand(PlaceData::DsCmdUpdateStart);
        break;
    }
    usleep(100000);
    sendDataProviderCommand(PlaceData::DpCmdUpdate);
  }

  while(isRunning())
  {
    usleep(100000);
  }
}


// ------------------------------------------------------
void PlaceController::newPlaceCommand(const cast::cdl::WorkingMemoryChange & change)
{
  // Get the command and remove from WM
  shared_ptr<CASTData<PlaceData::PlaceCommand> > commandCast =
    getWorkingMemoryEntry<PlaceData::PlaceCommand>(change.address);
  if (!commandCast)
    return;
  PlaceData::PlaceCommandPtr command = commandCast->getData();
  deleteFromWorkingMemory(change.address);

  // Execute command
  if (command->cmd==PlaceData::CmdUpdate)
  {
    debug("Received command CmdUpdate.");
    sendDataProviderCommand(PlaceData::DpCmdUpdate);
  }
  else if (command->cmd==PlaceData::CmdStart)
  {
    debug("Received command CmdStart.");
    switch(_updatesSentOn)
    {
      case VP_READ:
        sendVisualProcessorCommand(PlaceData::VpCmdUpdateOnReadStart);
        break;
      case DS_READ:
        sendDataSaverCommand(PlaceData::DsCmdUpdateStart);
        break;
    }
    usleep(100000);
    sendDataProviderCommand(PlaceData::DpCmdUpdate);
  }
  else if (command->cmd==PlaceData::CmdStop)
  {
    debug("Received command CmdStop.");
    switch(_updatesSentOn)
    {
      case VP_READ:
        sendVisualProcessorCommand(PlaceData::VpCmdUpdateStop);
        break;
      case DS_READ:
        sendDataSaverCommand(PlaceData::DsCmdUpdateStop);
        break;
    }
  }
  else
  {
    throw(CASTException(exceptionMessage(__HERE__, "Unknown command!" )));
  }
}


// ------------------------------------------------------
void PlaceController::sendDataProviderCommand(PlaceData::DataProviderCommandType cmd)
{
  PlaceData::DataProviderCommandPtr dataProviderCommand = new PlaceData::DataProviderCommand;
  dataProviderCommand->cmd = cmd;
  string dataId = newDataID();
  addToWorkingMemory(dataId, dataProviderCommand);
  if (cmd==PlaceData::DpCmdUpdate)
    println("Sent DpCmdUpdate command.");
  else
    println("Sent unknown DP command.");
}



// ------------------------------------------------------
void PlaceController::sendVisualProcessorCommand(PlaceData::VisualProcessorCommandType cmd)
{
  PlaceData::VisualProcessorCommandPtr visualProcessorCommand = new PlaceData::VisualProcessorCommand;
  visualProcessorCommand->cmd = cmd;
  string dataId = newDataID();
  addToWorkingMemory(dataId, visualProcessorCommand);
  if (cmd==PlaceData::VpCmdUpdateOnReadStart)
    println("Sent VpCmdUpdateOnReadStart command.");
  else if (cmd==PlaceData::VpCmdUpdateStop)
    println("Sent VpCmdUpdateStop command.");
  else
    println("Sent unknown VP command.");
}



// ------------------------------------------------------
void PlaceController::sendDataSaverCommand(PlaceData::DataSaverCommandType cmd)
{
  PlaceData::DataSaverCommandPtr dataSaverCommand = new PlaceData::DataSaverCommand;
  dataSaverCommand->cmd = cmd;
  string dataId = newDataID();
  addToWorkingMemory(dataId, dataSaverCommand);
  if (cmd==PlaceData::DsCmdUpdateStart)
    println("Sent DsCmdUpdateStart command.");
  else if (cmd==PlaceData::DsCmdUpdateStop)
    println("Sent DsCmdUpdateStop command.");
  else
    println("Sent unknown DS command.");
}


