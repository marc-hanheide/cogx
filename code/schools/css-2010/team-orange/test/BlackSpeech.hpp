/**
* BlackSpeech Component
* Author: Andrzej Pronobis
*/


#ifndef BLACKCONSOLE_HPP
#define BLACKCONSOLE_HPP

#include <ManagedComponent.hpp>
#include <BlackData.hpp>

#include <vector>
#include <map>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>

#include <iostream>
#include <string.h>


/**** Includes for MARY ****/
#include <netinet/in.h>
#include <sys/socket.h>
#include <netdb.h>

// #include <string>
// #include <string.h>


// #include "mary_client.h"

#include <stdint.h> 

#include <stdio.h>
#include <stdlib.h>


namespace black 
{

class BlackSpeech : public cast::ManagedComponent
{
public:

  BlackSpeech();
  virtual ~BlackSpeech();

  virtual void configure(const std::map<std::string, std::string> &config);
  virtual void start();
  virtual void runComponent();
  virtual void stop();

private:

	void MarySpeech(std::string text);
	int mary_query (
						const std::string& server_host,
            int server_port,
            const std::string& inputText,
            const std::string& maryInFormat,
            const std::string& maryOutFormat,
            const std::string& audioType,
            std::string& result);

   void newSpeechCommand(const cast::cdl::WorkingMemoryChange &objID);

// 	void PlaySound(char *file);
// 	void mixaudio(void *unused, Uint8 *stream, int len);
};

}; // namespace black

#endif // BLACKCONSOLE_HPP
