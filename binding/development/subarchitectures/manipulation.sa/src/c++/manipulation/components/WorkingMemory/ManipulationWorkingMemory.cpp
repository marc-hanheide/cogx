#include <manipulation/idl/Manipulation.hh>
#include <vision/idl/Vision.hh>

#include "ManipulationWorkingMemory.h"

using namespace Manipulation;
using namespace Vision;
using namespace cast;
using namespace std;
using namespace boost;

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new ManipulationWorkingMemory(_id);
  }
}

 
ManipulationWorkingMemory::ManipulationWorkingMemory(const string & _id)
:  SubarchitectureWorkingMemory(_id)
{
  // manipulation wm should broadcast to other sub-architectures
  setSendXarchChangeNotifications(true);

}

ManipulationWorkingMemory::~ManipulationWorkingMemory()
{
}
  
void ManipulationWorkingMemory::redrawGraphicsText()
{
  vector< shared_ptr< WorkingMemoryItem<Head> > > items;
  getItems<Head>(typeName<Head>(), items);


  printText("%d heads poses:\n", items.size());
  for(unsigned i = 0; i < items.size(); i++)
  {
    shared_ptr<const Head> h = items[i]->getData();
    printText("%6d:%06d: head %d pose %f %f %f  %f %f %f\n",
        h->m_time.m_s, h->m_time.m_us, h->m_num,
        h->m_pose.m_position.m_x, h->m_pose.m_position.m_y,
        h->m_pose.m_position.m_z, h->m_pose.m_orientation.m_x,
        h->m_pose.m_orientation.m_y, h->m_pose.m_orientation.m_z);
  }
}

