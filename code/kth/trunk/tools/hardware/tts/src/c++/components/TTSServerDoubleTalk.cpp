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
TTSServerDoubleTalk::say(const string & _message) {
  string cmd("echo " + _message + " > " + m_serialDevice);
  system(cmd.c_str()); 
}
