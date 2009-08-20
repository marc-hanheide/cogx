/**
 * @file Vs3Interface.cpp
 * @author Andreas Richtsfeld
 * @date April 2008
 * @version 0.1
 * @brief Calculating with vision system 3 (vs3)
 **/

#include <Vs3Interface.h>
#include <stdio.h>
#include <assert.h>
// #include "VisionCore.hh"


namespace cast
{

/**
 * @brief Constructor of class VisionSystem3
 */
Vs3Interface::Vs3Interface()
{
	config = "subarchitectures/vision.sa/src/c++/vision/components/VisionSystem3/cfg/vs3.cfg";
  vcore = new Z::VisionCore(Z::VIDEO_TYPE_BUFFER, config);
  assert(vcore != 0);	
}

/**
 * @brief Constructor of class VisionSystem3
 */
Vs3Interface::~Vs3Interface()
{
  delete vcore;
}


int Vs3Interface::ProcessSingleImage(IplImage *image)
{
	printf("Vs3Interface::ProcessSingleImage\n");

	static bool firstCall = true;
	if(firstCall) vcore->InitBuffer(image, false);
	else vcore->SetBuffer(image);
	firstCall = false;

// 	vcore->NewImage(image);

  vcore->ProcessImage(400, 600, 1, false);
	return 0;
}


}

