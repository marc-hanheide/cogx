#include "TTSServerESpeak.hpp"

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <autogen/TTS.hpp>
#include <cstdlib>
#include <sstream> 

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
    return new TTSServerESpeak();
  }
}

TTSServerESpeak::TTSServerESpeak() {}

void 
TTSServerESpeak::say(const string & _message) {
  println("saying: %s", _message.c_str());
  ostringstream cmd;
  cmd<<"espeak -v en-uk-wmids -k 20 -a 200 -p 44 -s 120 \" "
     <<_message << " \"";
  system(cmd.str().c_str()); 
}
