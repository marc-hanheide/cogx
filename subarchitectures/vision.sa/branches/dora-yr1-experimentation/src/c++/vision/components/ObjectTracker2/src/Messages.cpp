
#include "Messages.h"
#include "Resources.h"

// *** PRIVATE ***
MessageMgr::MessageMgr(){
	
}

// *** PUBLIC ***
MessageMgr::~MessageMgr(){
	
}

void MessageMgr::Message::print(){
	if(type == LOG)
		printf("(log) in %s: %s\n", location, description);
	if(type == WARNING)
		printf("(warning) in %s: %s\n", location, description);
	if(type == ERROR)
		printf("(error) in %s: %s\n", location, description);
}


void MessageMgr::log(const char* desc, const char* loc){
	Message msg;
	msg.type = LOG;
	sprintf(msg.description, "%s", desc);
	sprintf(msg.location, "%s", loc);
	
	m_MessageList.push_back(msg);
}

void MessageMgr::warning(const char* desc, const char* loc){
	Message msg;
	msg.type = WARNING;
	sprintf(msg.description, "%s", desc);
	sprintf(msg.location, "%s", loc);
	
	m_MessageList.push_back(msg);
}

void MessageMgr::error(const char* desc, const char* loc){
	Message msg;
	msg.type = ERROR;
	sprintf(msg.description, "%s", desc);
	sprintf(msg.location, "%s", loc);
	
	m_MessageList.push_back(msg);
	
	delete(g_Resources);
	exit(1);
}