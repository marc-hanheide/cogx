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
				       m_playerPTZDeviceID(0),
				       m_defaultZoom(0)
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
    
    //TODO: more safety checks!
    m_playerClient = shared_ptr<playerc_client_t>(playerc_client_create(NULL, m_playerHost.c_str(), m_playerPort));
    
    if(0 != playerc_client_connect(m_playerClient.get())) {
      throw cast::CASTException("unable to connect to player");
    }

    if(0 != playerc_client_datamode(m_playerClient.get(),PLAYER_DATAMODE_PULL)) {
      throw cast::CASTException("unable to connect to player");
    }

     if(0 != playerc_client_set_replace_rule(m_playerClient.get(), -1, -1, PLAYER_MSGTYPE_DATA, -1, true)) {
       throw cast::CASTException("unable to connect to player");
     }

    m_ptzProxy = shared_ptr<playerc_ptz_t>(playerc_ptz_create(m_playerClient.get(), m_playerPTZDeviceID));
    playerc_ptz_subscribe(m_ptzProxy.get(), PLAYER_OPEN_MODE);


    //going to work in position mode 
    playerc_ptz_set_control_mode(m_ptzProxy.get(),PLAYER_PTZ_POSITION_CONTROL);


    //get a default value for zoom
    playerc_client_read(m_playerClient.get());

    m_defaultZoom = m_ptzProxy->zoom;
  }


  PTZReading 
  PlayerPTZServer::getPose() {
    assert(m_ptzProxy);

    lockComponent();
    playerc_client_read(m_playerClient.get());
    
    PTZReading reading;    
    reading.time = getCASTTime();
    reading.pose.pan = m_ptzProxy->pan; 
    reading.pose.tilt = m_ptzProxy->tilt;
    reading.pose.zoom = m_ptzProxy->zoom;
    unlockComponent();
    //debug("PlayerPTZServer::getPose %f %f %f",reading.pose.pan,reading.pose.tilt,reading.pose.zoom);
    return reading;      
  }
  
  void 
  PlayerPTZServer::setPose(const PTZPose & _pose) {
     assert(m_ptzProxy);
    
//     //log("PlayerPTZServer::setPose %f %f %f",_pose.pan,_pose.tilt,_pose.zoom);    
//     //m_ptzProxy->SetCam(_pose.pan, _pose.tilt, _pose.zoom);
     //playerc_ptz_set_ws(m_ptzProxy.get(), _pose.pan, _pose.tilt, _pose.zoom, 0.5, 0.5);

     //debug("PlayerPTZServer::setPose %f %f %f",_pose.pan,_pose.tilt,m_defaultZoom);
     lockComponent();
     playerc_ptz_set_ws(m_ptzProxy.get(), _pose.pan, _pose.tilt, m_defaultZoom, MAX_SPEED, MAX_SPEED);

     unlockComponent();
  }
}
