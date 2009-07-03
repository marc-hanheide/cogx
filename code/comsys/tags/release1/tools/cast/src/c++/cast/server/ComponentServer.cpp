
#include <CASTComponentFactory.hpp> 
#include <CASTTimeServer.hpp> 

#include <CDL.hpp> 
#include <Ice/Ice.h> 
 
using namespace Ice;
using namespace std;

namespace cast {
  
  class ComponentServer : virtual public Ice::Application { 
  public: 
    virtual int run(int, char*[]) { 
     
      CommunicatorPtr ic = communicator();
      
      char buf[50];
      snprintf(buf,50,"default -p %d", cast::cdl::CPPSERVERPORT);
      
      ObjectAdapterPtr adapter 
	= ic->createObjectAdapterWithEndpoints("ComponentServer1", buf);
      
      //add component factory to server
      Identity factoryID;
      factoryID.name = "ComponentFactory";
      factoryID.category = "ComponentFactory";
      adapter->add(new cast::CASTComponentFactory(), factoryID);
      
      //add timeserver to server
      
      Identity tsID;
      tsID.name = "TimeServer";
      tsID.category = "TimeServer";
      adapter->add(new cast::CASTTimeServer(), tsID);
      
      
      adapter->activate();
      
      ic->waitForShutdown();
      
      return 0; 
    }

  };

}; 
  
int 
main(int argc, char* argv[]) {
  cast::ComponentServer app; 
  try {
    return app.main(argc, argv ); 
  }
  catch(...) {
    cout<<"Exception in ComponentServer"<<endl;
    return 1;
  }
}
