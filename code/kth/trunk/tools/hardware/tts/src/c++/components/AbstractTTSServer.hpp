#ifndef Abstract_TTS_SERVER_HPP_
#define Abstract_TTS_SERVER_HPP_

#include <cast/architecture/ManagedComponent.hpp>

class AbstractTTSServer : 
  public cast::ManagedComponent {
  
public:

  AbstractTTSServer();
  /**
   * Empty destructor.
   */
  virtual ~AbstractTTSServer(){};

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

  /**
   * Produce this message via TTS.
   */
  virtual void 
  say(const std::string &_message) = 0;

};



#endif
