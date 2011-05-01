/**
 * @author Andreas Richtsfeld
 * @date April 2011
 * @brief Just receives point clouds and displays them on the TomGine.
 */

#include "ManipulationPlanner.h"


/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ManipulationPlanner();
  }
}

namespace cast
{

using namespace std;
using namespace cogx;
using namespace cogx::Math;



/**
 * @brief Configure
 * @param _config Configuration
 */
void ManipulationPlanner::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;


}


/**
 * @brief start component
 */
void ManipulationPlanner::start()
{
}


/**
 * @brief runComponent
 */
void ManipulationPlanner::runComponent()
{

}

}

