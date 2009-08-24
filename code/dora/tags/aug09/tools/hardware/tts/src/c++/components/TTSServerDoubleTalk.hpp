#ifndef TTS_SERVER_DOUBLETALK_HPP_
#define TTS_SERVER_DOUBLETALK_HPP_

#include <cast/architecture/ManagedComponent.hpp>

class TTSServerDoubleTalk : 
  public cast::ManagedComponent {
  
public:

  TTSServerDoubleTalk();
  /**
   * Empty destructor.
   */
  virtual ~TTSServerDoubleTalk(){};

protected:

  virtual 
  void 
  start();

  virtual 
  void 
  stop();


private:

  void 
  handleSpeakCommand(const cast::cdl::WorkingMemoryChange & _wmc);

  void 
  say(const std::string &_message);

  std::string m_serialDevice;

};



#endif
