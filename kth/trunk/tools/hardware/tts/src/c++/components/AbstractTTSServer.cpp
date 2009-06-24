#include "AbstractTTSServer.hpp"

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <autogen/TTS.hpp>
#include <cstdlib> 

using namespace std;
using namespace cast;
using namespace cast::cdl;
using namespace TTS;

AbstractTTSServer::AbstractTTSServer()
{}

void 
AbstractTTSServer::handleSpeakCommand(const WorkingMemoryChange & _wmc) {
  SpeakPtr speak(getMemoryEntry<Speak>(_wmc.address));
  log("going to speak: %s", speak->message.c_str());
  say(speak->message);
  deleteFromWorkingMemory(_wmc.address);
}
  

void 
AbstractTTSServer::start() {
  say("hello");
  say("my name is mister chips");
  addChangeFilter(createGlobalTypeFilter<TTS::Speak>(cdl::ADD),
		  new MemberFunctionChangeReceiver<AbstractTTSServer>(this,&AbstractTTSServer::handleSpeakCommand));
}

void 
AbstractTTSServer::stop() {
  say("good bye");
}
