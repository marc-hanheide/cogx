
#ifndef __MESSAGEMGR_H__
#define __MESSAGEMGR_H__

#include "Singleton.h"
#include <vector>
#include <stdio.h>

#ifndef CHAR_LEN
#define CHAR_LEN 256
#endif

#ifndef MSG_TYPES
#define LOG 0
#define WARNING 1
#define ERROR 2
#endif

#define g_Messages MessageMgr::GetInstance()

typedef std::vector<char*> NameList;

class MessageMgr : public Singleton <MessageMgr>
{
friend class Singleton <MessageMgr>;
private:
	MessageMgr();
	
	

public:
	~MessageMgr();
	static MessageMgr* GetInstance(){
       return Singleton <MessageMgr>::GetInstance ();
    }
    struct Message{
    	unsigned int type;
    	char description[CHAR_LEN];
    	char location[CHAR_LEN];
    	void print();
    };
    
    std::vector<Message> m_MessageList;
    
    
    void log(const char* description, const char* location);
    void warning(const char* description, const char* location);
    void error(const char* description, const char* location);
    
    
    
};

#endif
