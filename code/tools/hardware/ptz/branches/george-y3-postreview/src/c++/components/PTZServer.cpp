#include "PTZServer.hpp"

using namespace cast;

namespace ptz {

  ptz::PTZReading PTZServerI::getPose(const Ice::Current & _crt) const {
    return m_ptzServer->getPose();
  }
  
  void PTZServerI::setPose(const ptz::PTZPose & _pose, 
			   const Ice::Current & _crt) {
    return m_ptzServer->setPose(_pose);
  }
  

  PTZServer::PTZServer()  {
  }


  void PTZServer::configure(const std::map<std::string,std::string> & _config)  {
	//setup ice server
  	PTZInterfacePtr servant = new PTZServerI(this);
  	registerIceServer<PTZInterface, PTZInterface>(servant);
  }
}
