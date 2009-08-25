//
// = FILENAME
//    
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "RobotbaseClientUtils.hpp"

Robotbase::RobotbaseServerPrx 
RobotbaseClientUtils::getServerPrx(cast::CASTComponent &owner,
                               const std::string serverHost,
                               int serverPort,
                               const std::string &serverName)
{
  Ice::CommunicatorPtr ic = owner.getCommunicator();
  
  Ice::Identity id;
  id.name = serverName;
  id.category = "RobotbaseServer";

  std::ostringstream str;
  str << ic->identityToString(id) 
      << ":default"
      << " -h " << serverHost
      << " -p " << serverPort; 

  //owner.debug(str.str());

  Ice::ObjectPrx base = ic->stringToProxy(str.str());    
  return Robotbase::RobotbaseServerPrx::uncheckedCast(base);
}

