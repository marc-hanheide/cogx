#ifndef HELLO_READER_HPP_
#define HELLO_READER_HPP_
#include <HelloWorldData.hpp>

#include <cast/architecture.hpp>

/*
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "MaryClient.h"


using namespace std;
*/


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

 #include <string>
 #include <string.h>


 #include "MaryClient.h"
using namespace std;


#include <stdint.h> 

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <stdlib.h>




class HelloReader :
  public cast::ManagedComponent {
protected:
virtual void start();
void makeAnnouncement(const cast::cdl::WorkingMemoryChange & _wmc) ;
void MarySpeech(std::string inputText);


};


#endif



