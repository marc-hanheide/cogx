#include "PlayerActServer.hpp"

/**
 * Create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr newComponent() {
    return new ptz::PlayerActServer();
  }
}


using namespace boost;

namespace ptz {

  PlayerActServer::PlayerActServer() : m_playerHost("localhost"),
				       m_playerPort(6665),
				       m_playerPTZDeviceID(0),
				       m_defaultZoom(0)
  {}


  void
  PlayerActServer::configure(const std::map<std::string,std::string> & _config) {
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

    m_ptzProxy = shared_ptr<playerc_actarray_t>(playerc_actarray_create(m_playerClient.get(), m_playerPTZDeviceID));
    playerc_actarray_subscribe(m_ptzProxy.get(), PLAYER_OPEN_MODE);


    //get a default value for zoom
    playerc_client_read(m_playerClient.get());

    m_defaultZoom = 1.0;
  }

  void
  PlayerActServer::start() {
    //ensure generic server part is configured
    PTZServer::start();
    //going to work in position mode
    //playerc_ptz_set_control_mode(m_ptzProxy.get(),PLAYER_PTZ_POSITION_CONTROL);
    //playerc_actarray_power (m_ptzProxy.get(), 1);

  }


  PTZReading
  PlayerActServer::getPose() {
    assert(m_ptzProxy);

    lockComponent();
    playerc_client_read(m_playerClient.get());

    player_actarray_actuator_t actData = playerc_actarray_get_actuator_data (m_ptzProxy.get(), 0);

    PTZReading reading;
    reading.time = getCASTTime();
    reading.pose.pan = actData.position;
    reading.pose.tilt = m_lastTilt;
    reading.pose.zoom = m_defaultZoom;
    unlockComponent();
    debug("PlayerActServer::getPose %f %f %f",reading.pose.pan,reading.pose.tilt,reading.pose.zoom);
    return reading;
  }

  void
  PlayerActServer::setPose(const PTZPose & _pose) {
     assert(m_ptzProxy);

//     //log("PlayerPTZServer::setPose %f %f %f",_pose.pan,_pose.tilt,_pose.zoom);
//     //m_ptzProxy->SetCam(_pose.pan, _pose.tilt, _pose.zoom);
     //playerc_ptz_set_ws(m_ptzProxy.get(), _pose.pan, _pose.tilt, _pose.zoom, 0.5, 0.5);

     debug("PlayerActServer::setPose %f %f %f",_pose.pan,_pose.tilt,m_defaultZoom);
     lockComponent();
     //playerc_ptz_set_ws(m_ptzProxy.get(), _pose.pan, _pose.tilt, m_defaultZoom, 0.5, 0.5);

     playerc_actarray_position_cmd (m_ptzProxy.get(), 0, _pose.pan);
     m_lastTilt=_pose.tilt;
     unlockComponent();
  }
}
