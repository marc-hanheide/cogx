#include "TTSServerDoubleTalk.hpp"

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <autogen/TTS.hpp>
#include <cstdlib> 

using namespace std;
using namespace cast;
using namespace cast::cdl;
using namespace TTS;


/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new TTSServerDoubleTalk();
  }
}

TTSServerDoubleTalk::TTSServerDoubleTalk() :
  m_serialDevice("/dev/ttyR0")
{}

void 
TTSServerDoubleTalk::handleSpeakCommand(const WorkingMemoryChange & _wmc) {
  SpeakPtr speak(getMemoryEntry<Speak>(_wmc.address));
  log("going to speak: %s", speak->message.c_str());
  say(speak->message);
  deleteFromWorkingMemory(_wmc.address);
}
  
void 
TTSServerDoubleTalk::say(const string & _message) {
  string cmd("echo " + _message + " > " + m_serialDevice);
  system(cmd.c_str()); 
}

void 
TTSServerDoubleTalk::start() {
  say("hello");
  say("my name is mister chips");
  addChangeFilter(createGlobalTypeFilter<TTS::Speak>(cdl::ADD),
		  new MemberFunctionChangeReceiver<TTSServerDoubleTalk>(this,&TTSServerDoubleTalk::handleSpeakCommand));
}

void 
TTSServerDoubleTalk::stop() {
  say("good bye");
}
