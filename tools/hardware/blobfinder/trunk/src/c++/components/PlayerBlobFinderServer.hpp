#include <cast/core.hpp>
#include <autogen/BlobFinder.hpp>
#include <libplayerc++/playerc++.h>
#include <boost/shared_ptr.hpp>

#ifndef PLAYER_BLOBFINDER_SERVER_HPP
#define PLAYER_BLOBFINDER_SERVER_HPP

namespace blobfinder {

  /**
   * An implementation of the ptz server using the player ptz device.
   *
  */
  class PlayerBlobFinderServer : public cast::CASTComponent {
  private:
    std::string m_playerHost;
    int m_playerPort;
    int m_playerBlobFinderDeviceID;

    boost::shared_ptr<PlayerCc::PlayerClient> m_playerClient;
    boost::shared_ptr<PlayerCc::BlobfinderProxy> m_blobFinderProxy;


    class BlobFinderI : public blobfinder::BlobFinderInterface {
      
    };

  protected:

    virtual void 
    configure(const std::map<std::string,std::string> & _config);

    virtual void start();


  public:
    PlayerBlobFinderServer();
    virtual ~PlayerBlobFinderServer() {}
  };

}

#endif //PLAYER_BLOBFINDER_SERVER_HPP
