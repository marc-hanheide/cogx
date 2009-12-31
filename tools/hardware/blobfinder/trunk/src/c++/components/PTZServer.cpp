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
  

  PTZServer::PTZServer() : m_iceName("PTZServer"),
			   m_icePort(cdl::CPPSERVERPORT) {
  }

  void PTZServer::setupIceServer() {
    Ice::Identity id;
    id.name = m_iceName;
    id.category = "PTZServer";
    getObjectAdapter()->add(new PTZServerI(this), id);
  }

  void PTZServer::start()  {
    setupIceServer();
  }
}
