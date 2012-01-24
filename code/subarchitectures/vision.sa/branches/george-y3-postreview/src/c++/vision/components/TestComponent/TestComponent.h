/**
 * @file TestComponent.h
 * @author Andreas Richtsfeld
 * @date March 2010
 * @version 0.1
 * @brief Testing WM problems
 */

#ifndef TEST_COMPONENT_H
#define TEST_COMPONENT_H

#include <cast/architecture/ManagedComponent.hpp>

#include <VideoClient.h>
#include <StereoClient.h>
#include <opencv2/highgui/highgui.hpp>


namespace cast
{

/**
	* @class TestComponent
	* @brief Test WM problems.
	*/
class TestComponent : public ManagedComponent,
                      public VideoClient,
                      public StereoClient
{
private:

	std::vector<std::string> objectIDs;			///< IDs of the currently stored visual objects

	void WriteNewVisualObject();
	bool CreateVisualObject(VisionData::VisualObjectPtr &obj, int id);
	void DeleteVisualObjectsFromWM();

protected:
  virtual void configure(const std::map<std::string,std::string> & _config);
  virtual void start();
  virtual void runComponent();

public:
  TestComponent() {}
  virtual ~TestComponent();
};

}

#endif



