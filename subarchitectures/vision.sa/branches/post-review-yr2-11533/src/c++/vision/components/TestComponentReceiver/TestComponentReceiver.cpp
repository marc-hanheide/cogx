/**
 * @file TestComponentReceiver.cpp
 * @author Andreas Richtsfeld
 * @date March 2010
 * @version 0.1
 * @brief Testing WM problems
 */


#include <cast/architecture/ChangeFilterFactory.hpp>

#include <opencv/highgui.h>
#include "TestComponentReceiver.h"


using namespace std;
using namespace VisionData;

/**
 * @brief The function called to create a new instance of our component.
 */
extern "C" {
	cast::CASTComponentPtr newComponent() {
    return new cast::TestComponentReceiver();
  }
}

namespace cast
{

/**
 *	@brief Destructor of class StereoDetector
 */
TestComponentReceiver::~TestComponentReceiver()
{}


/**
 *	@brief Called by the framework to configure the component.
 *	@param _config Config
 */
void TestComponentReceiver::configure(const map<string,string> & _config)
{}

/**
 *	@brief Called by the framework after configuration, before run loop.
 */
void TestComponentReceiver::start()
{
	// add change filter for visual objects
  addChangeFilter(createLocalTypeFilter<VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<TestComponentReceiver>(this, &TestComponentReceiver::receiveVisualObject));
}


/**
 *	@brief Called by the framework to start component run loop.
 */
void TestComponentReceiver::runComponent()
{}

/**
 *	@brief Receive a new created VisualObject.
 *	@param wmc Working memory change.
 */
void TestComponentReceiver::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
	VisualObjectPtr oPtr = getMemoryEntry<VisionData::VisualObject>(_wmc.address);
	log("received visual object with nr:  %s - %s", oPtr->identLabels[0].c_str() , _wmc.address.id.c_str());
}

}








