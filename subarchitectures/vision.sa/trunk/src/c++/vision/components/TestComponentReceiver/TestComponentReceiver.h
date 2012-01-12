/**
 * @file TestComponentReceiver.h
 * @author Andreas Richtsfeld
 * @date March 2010
 * @version 0.1
 * @brief Testing WM problems
 */

#ifndef TEST_COMPONENT_RECEIVER_H
#define TEST_COMPONENT_RECEIVER_H

#include <cast/architecture/ManagedComponent.hpp>

#include <VideoClient.h>
// #include <VisionData.hpp>
// #include <VideoUtils.h>
// #include <vector>
// 
// #include <VisionData.hpp>
#include <StereoClient.h>
// #include <../../VisionUtils.h>
#include <opencv2/highgui/highgui.hpp>

namespace cast
{

/**
	* @class TestComponentReceiver
	* @brief Test WM problems.
	*/
class TestComponentReceiver : public ManagedComponent,
                       public VideoClient,
                       public StereoClient
{
private:

	void receiveVisualObject(const cdl::WorkingMemoryChange & _wmc);

protected:
  virtual void configure(const std::map<std::string,std::string> & _config);
  virtual void start();
  virtual void runComponent();

public:
  TestComponentReceiver() {}
  virtual ~TestComponentReceiver();
};

}

#endif



