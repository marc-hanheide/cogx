#include "planner_client.hh"


extern "C" {
  cast::CASTComponentPtr
  newComponent() {
    return new Planner_Client();
  }
}

Planner_Client::Planner_Client(Designator&& name)
    :Implement(name)
{
}

void Planner_Client::start()
{   
    
}

void Planner_Client::runComponent()
{
    
    
}
