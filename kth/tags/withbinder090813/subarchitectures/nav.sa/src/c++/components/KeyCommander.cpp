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

// Cure
#include <Utils/KeyReader.hh>

using namespace cast;
using namespace navsa;

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
  while(isRunning()) {

    if (m_InitSleep > 0) {
      sleepComponent(m_InitSleep);
      m_InitSleep = 0;
    }

    int type = 0;
    bool automatic = false;

    if(m_AutomaticCommands > 0){
      if(m_AutomaticCommands > 1){
	type = 'f'; // move forward
	automatic = true;
	println("Sending automatic command (move forward)");
      }else{
	type = 'o';
	println("Sending automatic command (object search)");
      }
      m_AutomaticCommands--;

    }else if(m_NoKey){
      // nothing to do, exit the loop
      log("No interactive mode, just listening...");
      break;

    }else{

      Cure::KeyReader keyr;

      println("Select the NavCommand to send:");
      println("a: GOTOAREA; n: GOTONODE; x: GOTOXY");
      println("f: GOFORWARD; b: GOBACK; t: TURN (right)");
      println("s: STOP; : BLOCKCONTROL; k: removes a sent task (only valid with blocking commands);");
      println("F: FOLLOWPERSON; e: EXPLORE; r: EXPLOREFLOOR; ");
      println("o: add object observation");

      std::string valid = "anxfbtskFero";
      while(valid.find((char)type, 0) == std::string::npos){
	type = keyr.keyPressed();
	usleep(500000);
      }
    } // if(automatic)
    if(type == 'k'){
      println("Give me the task id: ");
      std::string id;
      std::cin >> id;

      std::string sa = "nav.sa";			
      if(existsOnWorkingMemory(id, sa)){
	log("The given id is on WM");
	try {
	  deleteFromWorkingMemory(id, sa);
	  log("Not any more, muhaha");
	}catch(...){
	  log("But someone removed it before than me");
	}
      }else{
	log("The given id does not exist on WM");
      }
    } else{
      int priority = 0;

      int anid;
      double x,y;

      std::string id;

      if (type == 'o') {

        for (int i = 0; i < 2; i++) {
        NavData::ObjObsPtr oo = new NavData::ObjObs;
        //println("Give the object category: ");
        //std::cin >> oo->category;
        oo->category = "greenbox";
        oo->time = getCASTTime();

        id = "objobs-" + newDataID();
        addToWorkingMemory<NavData::ObjObs>(id, oo);
        println("Added object obs of objct \"%s\", task id:%s", 
                oo->category.c_str(), id.c_str());
        }

        NavData::InhibitNavControlPtr tmp = new NavData::InhibitNavControl;
        tmp->message.resize(1);
        tmp->message[0] = "KeyCommander testing"; 
        id = "inhibit-" + newDataID();
        addToWorkingMemory<NavData::InhibitNavControl>(id, tmp);
        println("Added inhibitor to WM");
        
        sleepComponent(5000);

        deleteFromWorkingMemory(id);
        println("Deleted from working inhibitor from WM");

        continue;

      } else if (type == 'a') {
        println("Give the id of the area to go to: ");
        std::cin >> anid;
      } else if (type == 'n') {
        println("Give the id of the node to go to: ");
        std::cin >> anid;
      } else if (type == 'x') {
        println("Give the x-coordinate of the goal: ");
        std::cin >> x;
        println("Give the y-coordinate of the goal: ");
        std::cin >> y;
      }

      if(automatic) {

	priority = 'n';
      
      } else if (m_FixedPrio != "") {

        if (m_FixedPrio == "n") {
          priority = 'n';
        } else if (m_FixedPrio == "h") {
          priority = 'h';
        } else if (m_FixedPrio == "u") {
          priority = 'u';
        } else {
          println("Unknown fixed prio \"%s\", only handling n,h,u --> n",
                  m_FixedPrio.c_str());
          priority = 'n';
        }
      } else{
        Cure::KeyReader keyr;
	println("Set the priority:");
	println("n: normal; h: high; u: urgent");


	while(priority != 'n' && priority != 'h' && priority != 'u'){
	  priority = keyr.keyPressed();
	  usleep(10000);
	}
      }

      log("Posting a NavCommand...");

      NavData::NavCommandPtr cmd = new NavData::NavCommand;
      switch(priority){
      case 'n': cmd->prio = NavData::NORMAL; break;
      case 'h': cmd->prio = NavData::HIGH; break;
      case 'u': cmd->prio = NavData::URGENT; break;
      }
      switch(type){
      case 's':
        cmd->cmd = NavData::STOP;
        id = "stop-";
        break;
      case 'a':
        cmd->destId.resize(1);
        cmd->destId[0] = anid;
        cmd->cmd = NavData::GOTOAREA;
        id = "gotoarea-";
        break;
      case 'n':
        cmd->destId.resize(1);
        cmd->destId[0] = anid;
        cmd->cmd = NavData::GOTONODE;
        id = "gotonode-";
        break;
      case 'x':
        cmd->pose.resize(2);
        cmd->pose[0] = x;
        cmd->pose[1] = y;
        cmd->cmd = NavData::GOTOPOSITION;
        id = "gotoxy-";
        break;
      case 'f':
        cmd->distance.resize(1);
        cmd->distance[0] = 1.0;
        cmd->cmd = NavData::GOFORWARD;
        id = "goforward-";
        break;
      case 'F':
        cmd->cmd = NavData::PERSONFOLLOW;
        id = "personfollow-";
        break;
      case 'b':
        cmd->cmd = NavData::GOBACK;
        cmd->distance.resize(1);
        cmd->distance[0] = 1.0;
        id = "goback-";
        break;
      case 't':
        cmd->cmd = NavData::TURN;
        cmd->target.resize(1);
        cmd->target[0] = "right";
        id = "turn-";
        break;
      case 'w':
        cmd->cmd = NavData::BLOCKCONTROL;
        id = "block-";
        break;
      case 'e':
        cmd->cmd = NavData::EXPLORE;
        println("*** EXPLORE ONLY THIS AREA");
        id = "explore-";
        break;
      case 'r':
        cmd->cmd = NavData::EXPLOREFLOOR;
        println("*** EXPLORE FLOOR");
        id = "explorefloor-";
        break;
      }

      id += newDataID();
      addToWorkingMemory<NavData::NavCommand>(id, cmd);
      println("The task id is: %s", id.c_str());
    }
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

