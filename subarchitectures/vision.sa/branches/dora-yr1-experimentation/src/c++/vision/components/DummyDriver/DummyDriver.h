/**
 * @author Michael Zillich
 * @date February 2009
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

class DummyDriver : public ManagedComponent
{
private:
  /**
   * list of objects we want to have detected
   */
  std::vector<std::string> labels;

  /**
   * callback function called whenever a new object appears ore an object
   * changes
   */
  void receiveVisualObject(const cdl::WorkingMemoryChange & _wmc);

protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start();
  /**
   * called by the framework to start compnent run loop
   */
  virtual void runComponent();
 
public:
  virtual ~DummyDriver() {}
};

}

#endif

