/**
 * @file Vs3Interface.h
 * @author Andreas Richtsfeld
 * @date April 2008
 * @version 0.1
 * @brief Calculating with vision system 3 (vs3)
 **/

#ifndef VS3_INTERFACE_H
#define VS3_INTERFACE_H

#include <stdio.h>
#include <assert.h>
#include "VisionCore.hh"
#include "Image.hh"
#include <opencv/highgui.h>

// using namespace Z;

namespace cast
{

class Vs3Interface
{
private:
	char *config;										///< path and name of config-file
	Z::VisionCore *vcore;						///< vision core of vs3

public:
	Vs3Interface();
	~Vs3Interface();
	int ProcessSingleImage(IplImage *image);				// Process a single image with vs3


};



}

#endif