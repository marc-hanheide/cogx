#include "PlayerPTZServer.hpp"

/**
 * Create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr newComponent() {
    return new ptz::PlayerPTZServer();
  }
}


using namespace boost;

namespace ptz {

  PlayerPTZServer::PlayerPTZServer() : m_playerHost("localhost"),
				       m_playerPort(6665),
				       m_playerPTZDeviceID(0) 
  {}
  

  void 
  PlayerPTZServer::configure(const std::map<std::string,std::string> & _config) {
    //ensure generic server part is configured
    PTZServer::configure(_config);
    
    std::map<std::string,std::string>::const_iterator it;

    if ((it = _config.find("--player-host")) != _config.end()) {
      m_playerHost = it->second;
    }
    log("Using m_playerHost=%s", m_playerHost.c_str());

    if ((it = _config.find("--player-port")) != _config.end()) {
      std::istringstream str(it->second);
      str >> m_playerPort;
    }
    log("Using m_playerPort=%d", m_playerPort);     
 
    if ((it = _config.find("--ptz-device")) != _config.end()) {
      std::istringstream str(it->second);
      str >> m_playerPTZDeviceID;
    }
    log("Using m_playerPTZDeviceID=%d", m_playerPTZDeviceID);     
 
  }

  void 
  PlayerPTZServer::start() {
    //ensure generic server part is configured
    PTZServer::start();

    m_playerClient = shared_ptr<PlayerCc::PlayerClient>(new PlayerCc::PlayerClient(m_playerHost, m_playerPort));
    m_playerClient->SetDataMode(PLAYER_DATAMODE_PULL);
    m_playerClient->SetReplaceRule(true, PLAYER_MSGTYPE_DATA);

    m_ptzProxy = shared_ptr<PlayerCc::PtzProxy>(new PlayerCc::PtzProxy(m_playerClient.get(), 
								       m_playerPTZDeviceID));

    //going to work in position mode 
    m_ptzProxy->SelectControlMode(PLAYER_PTZ_POSITION_CONTROL);

  }


  PTZReading 
  PlayerPTZServer::getPose() const {
    assert(m_ptzProxy);

    m_playerClient->Read();

    PTZReading reading;    
    reading.time = getCASTTime();
    reading.pose.pan = m_ptzProxy->GetPan(); 
    reading.pose.tilt = m_ptzProxy->GetTilt();
    reading.pose.zoom = m_ptzProxy->GetZoom();
    log("PlayerPTZServer::getPose %f %f %f",reading.pose.pan,reading.pose.tilt,reading.pose.zoom);
    return reading;      
  }
  
  void 
  PlayerPTZServer::setPose(const PTZPose & _pose) {
    assert(m_ptzProxy);
    log("PlayerPTZServer::setPose %f %f %f",_pose.pan,_pose.tilt,_pose.zoom);
    //m_ptzProxy->SetCam(_pose.pan * M_PI / 180, _pose.tilt * M_PI / 180, _pose.zoom * M_PI / 180);
    m_ptzProxy->SetCam(_pose.pan, _pose.tilt, _pose.zoom);
  }
}
