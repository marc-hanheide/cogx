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

#ifndef LaserClientUtils_hpp
#define LaserClientUtils_hpp

#include <Ice/Ice.h>
#include <string>

#include <cast/core/CASTComponent.hpp>

#include <Laser.hpp>

namespace LaserClientUtils {

  // Fixme: Should really be a template
  Laser::LaserServerPrx getServerPrx(cast::CASTComponent &owner,
 			     const std::string serverHost = "localhost",
 			     int serverPort = cast::cdl::CPPSERVERPORT,
                             const std::string &serverName = "LaserServer");
  
}; // namespace LaserClientUtils

#endif // LaserClientUtils_hpp
