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
    
    IceInternal::Handle<cogx::display::CDisplayServer> server = new cogx::display::CDisplayServer();        
    map<string,string> config;
    
    char buf[50];
   snprintf(buf,50,"default -p %d", 20202);
   
   ObjectAdapterPtr adapter = ic->createObjectAdapterWithEndpoints("DisplayServerAdapter", buf);
   
   server->_setObjectAdapter(adapter);
   
    Identity id;
   id.name = "StandaloneDisplayServer";
   id.category = "StandaloneDisplayServer";
   adapter->add(server, id);
   adapter->activate();
   
   
   CASTComponentPrx serverProxy = CASTComponentPrx::uncheckedCast(adapter->addWithUUID(server));
    
    server->setID("display.srv", Ice::Current());
    
    config[cdl::COMPONENTNUMBERKEY] = "0";
    config[cdl::SUBARCHIDKEY] = "v11n.sa";
    serverProxy->configure(config);
    serverProxy->start();
    server->run(); 
    ic->waitForShutdown();
     
    return 0;    
}
