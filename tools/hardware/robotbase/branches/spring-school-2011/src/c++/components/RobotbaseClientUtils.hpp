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

#ifndef RobotbaseClientUtils_hpp
#define RobotbaseClientUtils_hpp

#include <Ice/Ice.h>
#include <string>

#include <cast/core/CASTComponent.hpp>

#include <Robotbase.hpp>

namespace RobotbaseClientUtils {

  // Fixme: Should really be a template
  Robotbase::RobotbaseServerPrx getServerPrx(cast::CASTComponent &owner,
                            const std::string serverHost = "localhost",
                            int serverPort = cast::cdl::CPPSERVERPORT,
                            const std::string &serverName = "RobotbaseServer");

}; // namespace RobotbaseClientUtils

#endif // RobotbaseClientUtils_hpp
