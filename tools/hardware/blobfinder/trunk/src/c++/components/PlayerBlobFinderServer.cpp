#include "PlayerBlobFinderServer.hpp"

/**
 * Create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr newComponent() {
    return new blobfinder::PlayerBlobFinderServer();
  }
}


using namespace boost;

namespace blobfinder {

  PlayerBlobFinderServer::PlayerBlobFinderServer() : m_playerHost("localhost"),
				       m_playerPort(6665),
				       m_playerBlobFinderDeviceID(0) 
  {}
  

  void 
  PlayerBlobFinderServer::configure(const std::map<std::string,std::string> & _config) {
    
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
      str >> m_playerBlobFinderDeviceID;
    }
    log("Using m_playerBlobFinderDeviceID=%d", m_playerBlobFinderDeviceID);     
 
  }

  void 
  PlayerBlobFinderServer::start() {
 
    m_playerClient = shared_ptr<PlayerCc::PlayerClient>(new PlayerCc::PlayerClient(m_playerHost, m_playerPort));
    m_playerClient->SetDataMode(PLAYER_DATAMODE_PULL);
    m_playerClient->SetReplaceRule(true, PLAYER_MSGTYPE_DATA);

    
    //instantiate the server implementation...
    BlobFinderInterfacePtr servant 
      = new BlobFinderI(shared_ptr<PlayerCc::BlobfinderProxy>(new PlayerCc::BlobfinderProxy(m_playerClient.get(),
											    m_playerBlobFinderDeviceID)));
    //... and register it with the runtime
    registerIceServer<BlobFinderInterface,BlobFinderInterface>(servant);
  }


  Ice::Int PlayerBlobFinderServer::BlobFinderI::getBlobCount(const Ice::Current & _crt) const {
    return m_blobFinderProxy->GetCount();
  }

  blobfinder::BlobInfoSequence PlayerBlobFinderServer::BlobFinderI::getBlobs(const Ice::Current & _crt) const {
    BlobInfoSequence blobs;

    return blobs;
  }

//   BlobFinderReading 
//   PlayerBlobFinderServer::getPose() const {
//     assert(m_ptzProxy);

//     m_playerClient->Read();

//     BlobFinderReading reading;    
//     reading.time = getCASTTime();
//     reading.pose.pan = m_ptzProxy->GetPan(); 
//     reading.pose.tilt = m_ptzProxy->GetTilt();
//     reading.pose.zoom = m_ptzProxy->GetZoom();
//     log("PlayerBlobFinderServer::getPose %f %f %f",reading.pose.pan,reading.pose.tilt,reading.pose.zoom);
//     return reading;      
//   }
  
//   void 
//   PlayerBlobFinderServer::setPose(const BlobFinderPose & _pose) {
//     assert(m_ptzProxy);
//     log("PlayerBlobFinderServer::setPose %f %f %f",_pose.pan,_pose.tilt,_pose.zoom);
//     //m_ptzProxy->SetCam(_pose.pan * M_PI / 180, _pose.tilt * M_PI / 180, _pose.zoom * M_PI / 180);
//     m_ptzProxy->SetCam(_pose.pan, _pose.tilt, _pose.zoom);
//   }
}
