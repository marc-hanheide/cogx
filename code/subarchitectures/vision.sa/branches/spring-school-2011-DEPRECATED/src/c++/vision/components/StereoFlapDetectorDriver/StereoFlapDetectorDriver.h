/**
 * @author Andreas Richtsfeld
 * @date September 2009
 *
 * Just a dummy component to drive some vision functionality.
 */

#ifndef DUMMY_DRIVER_H
#define DUMMY_DRIVER_H

#include <vector>
#include <string>
#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>

namespace cast
{

class StereoFlapDetectorDriver : public ManagedComponent
{
private:

  /**
   * @brief callback function called whenever a new object appears or an object changes
   */
	void receiveVisualObject(const cdl::WorkingMemoryChange & _wmc);

	bool tracking;
	bool detecting;
	bool running;
	bool not_found;

protected:
  /**
   * @brief called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);

  /**
   * @brief called by the framework after configuration, before run loop
   */
  virtual void start();

  /**
   * @brief called by the framework to start compnent run loop
   */
  virtual void runComponent();
 
public:
  virtual ~StereoFlapDetectorDriver() {}
};

}

#endif

