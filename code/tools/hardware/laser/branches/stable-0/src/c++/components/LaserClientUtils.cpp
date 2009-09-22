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

#include "LaserClientUtils.hpp"

Laser::LaserServerPrx 
LaserClientUtils::getServerPrx(cast::CASTComponent &owner,
                               const std::string serverHost,
                               int serverPort,
                               const std::string &serverName)
{
  Ice::CommunicatorPtr ic = owner.getCommunicator();
  
  Ice::Identity id;
  id.name = serverName;
  id.category = "LaserServer";

  std::ostringstream str;
  str << ic->identityToString(id) 
      << ":default"
      << " -h " << serverHost
      << " -p " << serverPort; 

  //owner.debug(str.str());

  Ice::ObjectPrx base = ic->stringToProxy(str.str());    
  return Laser::LaserServerPrx::uncheckedCast(base);
  return NULL;
}


