/**
* BlackSpeech Component
* Author: Andrzej Pronobis
*/


#include "BlackSpeech.hpp"
#include "ConfigFile.h"
#include <ChangeFilterFactory.hpp>

#include <boost/lexical_cast.hpp>
#include <fcntl.h>
#include <errno.h>
#include <sstream>
#include <iterator>
#include <sstream>
#include <stdexcept>



#include <netinet/in.h>
#include <sys/socket.h>
#include <netdb.h>

#include <string>
#include <iostream>

// #include "mary_client.h"
#include <string.h>
// #include "unicode.h"


using namespace std;
using namespace black;
using namespace cast;
using namespace boost;

extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new BlackSpeech();
  }
}


// -----------------------------------------------------------------
BlackSpeech::BlackSpeech()
{
}


// -----------------------------------------------------------------
BlackSpeech::~BlackSpeech()
{
}


// -----------------------------------------------------------------
void BlackSpeech::configure(const std::map<std::string, std::string> &config)
{
  string cfgGroup = "BlackSpeech";

  // Read cmd line options
  map<string,string>::const_iterator it = config.find("--config");
  string configFile;
  if (it!= config.end()) 
  {
    configFile = it->second;
  }

  // Read config file
  ConfigFile cf;
  if (configFile.empty())
    println("No configuration file provided. Using defaults instead.");
  else
  {
    if (!cf.read(configFile))
      throw runtime_error(exceptionMessage(__HERE__, "Unable to load configuration file '%s'!", configFile.c_str()));
  }

  // Get configuration from the file
  try
  {
   //   _port=cf.getIntValue(cfgGroup, "Port", 2222);
  }
  catch(bad_lexical_cast &)
  {
    throw runtime_error(exceptionMessage(__HERE__, "Incorrect item value in the config file '%s'!", configFile.c_str() ));
  }

  // Print the configuration
  //log("Configuration:");
  //log("-> Port: %d", _port);
}


// -----------------------------------------------------------------
void BlackSpeech::start()
{
  addChangeFilter(createLocalTypeFilter<BlackData::SpeechCommand>(cdl::ADD),
    new MemberFunctionChangeReceiver<BlackSpeech>(this, &BlackSpeech::newSpeechCommand));
}


// -----------------------------------------------------------------
void BlackSpeech::stop()
{
}


// -----------------------------------------------------------------
void BlackSpeech::runComponent()
{
  while (isRunning())
  {
    usleep(1000000);
  } // while
}


// -----------------------------------------------------------------
void black::BlackSpeech::newSpeechCommand(const cast::cdl::WorkingMemoryChange & objID)
{

 shared_ptr<CASTData<BlackData::SpeechCommand> > obj =
    getWorkingMemoryEntry<BlackData::SpeechCommand>(objID.address);

  if (obj != 0)
  {
    BlackData::SpeechCommandType cmd = obj->getData()->cmd;
    debug("Received SpeechCommand.");

		std::string say = "";
		char say2[100];
    // Start new
    switch(cmd)
    {
      case BlackData::GOINGTO:
        log("Speech: I'm going to %s now!", obj->getData()->strParam.c_str());

				sprintf(say2, "I'm going to %s now!", obj->getData()->strParam.c_str());
				say = say2;
				MarySpeech(say);
// 				MarySpeech("I'm going to the next room.");
/*	      sleepComponent(3000);
				MarySpeech("Come with me, please.");
	      sleepComponent(3000);
				MarySpeech("Come on, follow me!");*/
        break;
      case BlackData::SEARCHING:
        log("Speech: I'm searching for objects!");
				MarySpeech("I'm searching for objects in this room!");
/*	      sleepComponent(3000);
				MarySpeech("Where are the objects?");
	      sleepComponent(5000);
				MarySpeech("Come on!");*/
// 	      sleepComponent(1000);
				MarySpeech("Help me, searching the objects!");
        break;
      case BlackData::SHAKING:
        log("Speech: I'm confused, I will have a look around!");
				MarySpeech("I'm confused, I will have a look around!");
//         sleepComponent(1000);
				MarySpeech("o. k. Now I know what to do!");
        break;
      case BlackData::RETURNING:
        log("Speech: Going to the target point!");
				MarySpeech("I'm going to the target point!");
        sleepComponent(1000);
				MarySpeech("Who wants to carry me?");
        break;
      case BlackData::GRABBING:
        log("Speech: Grabbing object!");
//				MarySpeech("I'm grabbing the object now!");
				MarySpeech("I'm going to grab the object.");
        break;
      case BlackData::RELEASING:
        log("Speech: Releasing object!");
				MarySpeech("Please help me releasing the object now!");
        break;
      case BlackData::APPROACHING:
        log("Speech: Approaching !");
				MarySpeech("Yeah.");
// 				sleepComponent(1000);
				MarySpeech("I can see the object. I'm approaching!");
        break;
      default:
        println("Unknown SpeechCommand found in the WM!");
				MarySpeech("I dont know what to do!");
        break;
    }

    deleteFromWorkingMemory(objID.address);
  }
}


/**
 * @brief MARY Text to speech converter
 */
void BlackSpeech::MarySpeech(std::string text)
{
//   std::string server_host = "cling.dfki.uni-sb.de";
  std::string server_host = "localhost";
  int server_port = 59125;
//   std::string text = "Welcome to the black planner! I am fine!";

// 	text = "Welcome to the black planner!";

  // To send properly encoded UTF-8 text, you need to convert the text to UTF-8
  // (e.g. by means of the ICU library used by the accompanying unicode.c --
  // get it at http://ibm.com/software/globalization/icu)


#ifdef WITH_UNICODE
  // convert request from ISO-8859-1 to UTF-8
  text = convert_encoding (text, "LATIN-1", "UTF-8");
#endif

  std::string inputType = "TEXT_EN";
  //std::string outputType = "ACOUSTPARAMS";
  std::string outputType = "AUDIO";
//   std::string audioType = "WAVE";
	std::string audioType = "WAVE";

  std::string result;

	if (mary_query(server_host, server_port, text, inputType, outputType, audioType, result) != 0)
	std::cout << "BLACK SPEECH: mery_query unsuccessful finished!" << std::endl;


	FILE *file = popen("play -q -t wav -", "w");
	fwrite(result.c_str(), 1, result.size(), file);
	pclose(file);


// 	std::istream in(&ib);
	
 //cat test.wav | play -t wav -
}




/**
 * @brief MARY: Create mary query
 */
int BlackSpeech::mary_query (
						const std::string& server_host,
            int server_port,
            const std::string& inputText,
            const std::string& maryInFormat,
            const std::string& maryOutFormat,
            const std::string& audioType,
            std::string& result)
{
  // declare global connection stuff
  struct sockaddr_in maryServer;
  struct sockaddr_in maryClient;
  struct hostent* hostInfo;

  // declare global variables
  int maryInfoSocket;
  int maryDataSocket;

  // set configuration parameters

  // get host information
  hostInfo = gethostbyname (server_host.c_str());

  if (hostInfo == NULL)
  {
    return -2;
  }


  // create a tcp connection to the mary server
  maryInfoSocket = socket (AF_INET, SOCK_STREAM, 0);

  // verify that the socket could be opened successfully
  if (maryInfoSocket == -1)
  {
    return -2;
  }
  else
  // autoflush stdout, bind and connect
  {
    maryClient.sin_family = AF_INET;
    maryClient.sin_port = htons (0);
    maryClient.sin_addr.s_addr = INADDR_ANY;

    int status = bind (maryInfoSocket, (struct sockaddr*) &maryClient, sizeof (maryClient));

    if (status != 0)
    {
      return -2;
    }

    maryServer.sin_family = AF_INET;
    maryServer.sin_port = htons (server_port);
    memcpy ((char*) &maryServer.sin_addr.s_addr, hostInfo->h_addr_list [0], hostInfo->h_length);

    status = connect (maryInfoSocket, (struct sockaddr*) &maryServer, sizeof (maryServer));

    if (status != 0)
    {
      return -2;
    }
  }

  // prepare the request
  std::string query = "MARY IN=" + maryInFormat + " OUT=" + maryOutFormat;
  if (maryOutFormat == "AUDIO") query += " AUDIO="+audioType;
  query += "\012\015";

//   std::cerr << "BLACK SPEECH: Constructed query: "<<query;

  // send request to the Mary server
  if (send (maryInfoSocket, query.c_str (), query.size (), 0) == -1)
  {
    return -2;
  }


  // receive the request id
  char id [32] = "";

  if (recv (maryInfoSocket, id, 32, 0) == -1)
  {
    return -2;
  }

//   std::cerr << "BLACK SPEECH: Read id: " << id;

  // create a tcp connection to the mary server
  maryDataSocket = socket (AF_INET, SOCK_STREAM, 0);

  // verify that the socket could be opened successfully
  if (maryDataSocket == -1)
  {
    return -2;
  }
  else
  // autoflush stdout, bind and connect
  {
    maryClient.sin_family = AF_INET;
    maryClient.sin_port = htons (0);
    maryClient.sin_addr.s_addr = INADDR_ANY;

    int status = bind (maryDataSocket, (struct sockaddr*) &maryClient, sizeof (maryClient));

    if (status != 0)
    {
      return -2;
    }

    maryServer.sin_family = AF_INET;
    maryServer.sin_port = htons (server_port);
    memcpy ((char*) &maryServer.sin_addr.s_addr, hostInfo->h_addr_list [0], hostInfo->h_length);

    status = connect (maryDataSocket, (struct sockaddr*) &maryServer, sizeof (maryServer));

    if (status != 0)
    {
      return -2;
    }
  }


  // send the request id to the Mary server
  if (send (maryDataSocket, id, strlen (id), 0) == -1)
  {
    return -2;
  }

//   std::cerr << "BLACK SPEECH: Sending request:" << inputText;

  // send the query to the Mary server
  if (send (maryDataSocket, inputText.c_str (), inputText.size (), 0) == -1)
  {
    return -2;
  }

  if (send (maryDataSocket, "\012\015", 2, 0) == -1)
  {
    return -2;
  }

  // shutdown data socket
  shutdown (maryDataSocket, 1);

//   std::cerr << "BLACK SPEECH: Reading result" << std::endl;

  unsigned int total_bytes = 0;
  int recv_bytes = 0;
  char data [1024] = "";

  result [0] = '\0';

  // receive the request result
  do
  {
    data [0] = '\0';

    recv_bytes = recv (maryDataSocket, data, 1024, 0);

    if (recv_bytes == -1)
    {
      return -2;
    }
    else if (recv_bytes > 0)
    {
      //std::cerr << "("<<recv_bytes<<")";
      total_bytes += recv_bytes;
      data [recv_bytes] = '\0';

      if (maryOutFormat == "AUDIO")
      {
        for (int i=0; i<recv_bytes; i++)
        {
          result += data [i];
        }
      }
      else
      {
        result += data;
      }
    }
  } while (recv_bytes != 0);

  if (result.size () != total_bytes)
  {
    std::cerr << "BLACK SPEECH: error: total bytes received != result bytes!" << std::endl;
    std::cerr << "       total bytes received = " << total_bytes << std::endl;
    std::cerr << "       result bytes = " << result.size () << std::endl;
  }

  // receive the request error
  do
  {
    data [0] = '\0';

    recv_bytes = recv (maryInfoSocket, data, 1024, 0);

    if (recv_bytes == -1)
    {
      return -2;
    }
    else if (recv_bytes > 0)
    {
      std::cerr << std::endl << "BLACK SPEECH: Mary error code: " << data << std::endl;
      return -3;
    }
  } while (recv_bytes != 0);

  // close all open sockets
  close (maryInfoSocket);
  close (maryDataSocket);

  return 0;
}

