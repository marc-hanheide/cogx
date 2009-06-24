#ifndef TTS_SERVER_ESPEAK_HPP_
#define TTS_SERVER_ESPEAK_HPP_

#include <cast/architecture/ManagedComponent.hpp>
#include "AbstractTTSServer.hpp"

class TTSServerESpeak : 
  public AbstractTTSServer {
  
public:

  TTSServerESpeak();
  /**
   * Empty destructor.
   */
  virtual ~TTSServerESpeak(){};

protected:

  virtual void 
  say(const std::string &_message);

};



#endif

//  LocalWords:  ifndef
