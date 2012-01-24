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
using namespace std;

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
 


    //register everything in configure so the server is available during start() calls

    boost::shared_ptr<PlayerCc::PlayerClient> playerClient = shared_ptr<PlayerCc::PlayerClient>(new PlayerCc::PlayerClient(m_playerHost, m_playerPort));
    playerClient->SetDataMode(PLAYER_DATAMODE_PULL);
    playerClient->SetReplaceRule(true, PLAYER_MSGTYPE_DATA);
    
    //instantiate the server implementation...
    BlobFinderInterfacePtr servant 
      = new BlobFinderI(playerClient,
			shared_ptr<PlayerCc::BlobfinderProxy>(new PlayerCc::BlobfinderProxy(playerClient.get(),
											    m_playerBlobFinderDeviceID)));
    //... and register it with the runtime
    registerIceServer<BlobFinderInterface,BlobFinderInterface>(servant);
  }

  void 
  PlayerBlobFinderServer::start() {
 

  }

  void 
  PlayerBlobFinderServer::runComponent() {


  }


  Ice::Int PlayerBlobFinderServer::BlobFinderI::getBlobCount(const Ice::Current & _crt) const {
    m_playerClient->Read();
    return m_blobFinderProxy->GetCount();
  }

  blobfinder::BlobInfoSequence PlayerBlobFinderServer::BlobFinderI::getBlobs(const Ice::Current & _crt) const {
    
    //oddly requires 2 reads to produce the correct answer
    m_playerClient->Read();    
    m_playerClient->Read();    
    
    int blobCount = m_blobFinderProxy->GetCount();
    
    BlobInfoSequence blobs;
    for(int i = 0; i < blobCount; i++) {
      player_blobfinder_blob_t playerBlob(m_blobFinderProxy->GetBlob(i));
      BlobInfo blob;
      blob.id = playerBlob.id;
      blob.area = playerBlob.area;

      //0,0 is top-left of camera image
      blob.boundingBox.pos.x = playerBlob.x + 0.001; // a position of 0.0 causes a div by zero so add epsilon here
      blob.boundingBox.pos.y = playerBlob.y + 0.001;
      blob.boundingBox.width = playerBlob.right - playerBlob.left;
      blob.boundingBox.height = playerBlob.bottom - playerBlob.top;

      //colour information is packed 32 bit, i.e., 0x00RRGGBB
      blob.colour.r = (playerBlob.color >> 16) & 255;
      blob.colour.g = (playerBlob.color >> 8) & 255;
      blob.colour.b = playerBlob.color & 255;

      blob.range = playerBlob.range;

      blobs.push_back(blob);
    }
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
