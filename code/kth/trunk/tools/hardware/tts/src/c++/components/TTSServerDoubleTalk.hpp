#ifndef TTS_SERVER_DOUBLETALK_HPP_
#define TTS_SERVER_DOUBLETALK_HPP_

#include <cast/architecture/ManagedComponent.hpp>
#include "AbstractTTSServer.hpp"

class TTSServerDoubleTalk : 
  public AbstractTTSServer {
  
public:

  TTSServerDoubleTalk();
  /**
   * Empty destructor.
   */
  virtual ~TTSServerDoubleTalk(){};

protected:

  virtual void 
  say(const std::string &_message);
  
  std::string m_serialDevice;

};



#endif
