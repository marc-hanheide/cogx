/*
 *  StandaloneDk.cpp
 *  DoraYr2
 *
 *  Created by Nick Hawes on 09/09/2010.
 *  Copyright 2010 University of Birmingham. All rights reserved.
 *
 */

#include <Ice/Ice.h>
#include "CDisplayServer.hpp"

using namespace std;
using namespace Ice;
using namespace cast;
using namespace cast::interfaces;

int main(int argc, char* argv[]) {
  
  Ice::CommunicatorPtr ic = Ice::initialize(argc, argv);
  char buf[50];
  snprintf(buf,50,"default -p %d", Visualization::V11NSTANDALONEPORT);
  ObjectAdapterPtr adapter = ic->createObjectAdapterWithEndpoints("DisplayServerAdapter", buf);

  
  IceInternal::Handle<cogx::display::CDisplayServer> server = new cogx::display::CDisplayServer();        
  server->_setObjectAdapter(adapter);
  

  map<string,string> config;
  
  
  
  CASTComponentPrx serverProxy 
    = CASTComponentPrx::uncheckedCast(adapter->addWithUUID(server));

  adapter->activate();
  
  
  serverProxy->setID(Visualization::V11NSTANDALONENAME);

  config[cdl::COMPONENTNUMBERKEY] = "0";
  config[cdl::SUBARCHIDKEY] = "v11n.sa";  
  serverProxy->configure(config);

  serverProxy->start();

  server->run(); 
 
 
  ic->waitForShutdown();
  
  return 0;    
}
