//
// = FILENAME
//    
//
// = FUNCTION
//
// = AUTHOR(S)
//    Dorian Galvez Lopez
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2008 Dorian Galvez Lopez
//                  2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/
 
#include "KeyCommander.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <NavData.hpp>

#include <iostream>
#include <string>
#include <sstream>

// Cure
#include <Utils/KeyReader.hh>

using namespace cast;
using namespace navsa;
using namespace std;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new KeyCommander();
  }
}

// ----------------------------------------------------------------------------

KeyCommander::KeyCommander()
{
  m_USleep = 0;
  m_AutomaticCommands = 0;
  m_FixedPrio = "";
  m_InitSleep = 0;
}

// ----------------------------------------------------------------------------

void KeyCommander::configure(const std::map<std::string,std::string> &_config)
{
  log("configure");

  std::map<std::string, std::string>::const_iterator it;

  it = _config.find("-u");
  if(it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_USleep;
  }

  it = _config.find("--auto");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_AutomaticCommands;
  }

  it = _config.find("--fixed-prio");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_FixedPrio;
  }

  it = _config.find("--init-sleep");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_InitSleep;
  }

  m_NoKey = (_config.find("--no-key") != _config.end());
}

// ----------------------------------------------------------------------------

void KeyCommander::start() {

  addChangeFilter(createLocalTypeFilter<NavData::NavCommand>(cdl::ADD),
                  new MemberFunctionChangeReceiver<KeyCommander>(this,
                                        &KeyCommander::owtNavCommand));

  addChangeFilter(createLocalTypeFilter<NavData::NavCommand>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<KeyCommander>(this,
                                        &KeyCommander::owtNavCommand));
}

// ----------------------------------------------------------------------------

void KeyCommander::runComponent() {

  m_CanAbort = false;
  std::string objId;
  // How to get a single char.
  char myChar  = {0};
string input = "";


  while(isRunning()) {
   
    cout << "Please enter 1 char: ";
    getline(cin, input);
    
    if (input.length() > 0) {
      myChar = input[0];
      break;
    }
    cout << "Invalid character, please try again" << endl;
    usleep(10000);
  
  }
}

// ----------------------------------------------------------------------------

void KeyCommander::owtNavCommand(const cdl::WorkingMemoryChange & objID)
{
  // Ignore if we made the change
  if(objID.src == getComponentID()) return;
	
  try {
	
    boost::shared_ptr<CASTData<NavData::NavCommand> > oobj =
      getWorkingMemoryEntry<NavData::NavCommand>(objID.address);

    if (oobj != 0){
      switch(oobj->getData()->comp){
      case NavData::SUCCEEDED:
        log("Task accomplished!");
        break;
      case NavData::FAILED:
        log("Task failed");
        break;
      case NavData::ABORTED:
        log("Task aborted :(");
        break;
      case NavData::INPROGRESS:
        log("Task in progress...");
        break;
      case NavData::PENDING:
        log("Task pending...");
        break;
      }
      switch(oobj->getData()->status){
      case NavData::PERSONNOTFOUND:
        log("Status: person not found");
        break;
      case NavData::TARGETUNREACHABLE:
        log("Status: target unreachable");
        break;
      case NavData::NONE:
        log("Status: none");
        break;
      case NavData::CMDMALFORMATTED:
        log("Status: command malformatted");
        break;
      case NavData::REPLACEDBYNEWCMD:
        log("Status: replaced by new command");
        break;
      case NavData::UNKNOWN:
        log("Status: unknown");
        break;
      }
      log(objID.address.id);
    }

  }catch(...){
    log("Nav task removed by other component");
  }
}

