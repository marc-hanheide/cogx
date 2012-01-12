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
#include <opencv2/highgui/highgui.hpp>

#include "VisionCore.hh"
#include "Image.hh"
#include "CubeDefinition.hh"

// using namespace Z;

namespace cast
{

class Vs3Interface
{
private:
	char *config;										///< path and name of config-file
	Z::VisionCore *vcore;						///< vision core of vs3

	int processingTime;							///< Processing time of vs3 [ms]
	int cannyAlpha;									///< Canny parameter: alpha*1000
	int cannyOmega;									///< Canny parameter: omega*1000

//	bool drawGestalts;							///< Drawing Gestalts to the OpenCv IplImage. (default = false)
//	IplImage *iplImage; 						///< OpenCv IplImage for drawing Gestalts (Objects)


public:
	Vs3Interface();
	~Vs3Interface();
	int ProcessSingleImage(IplImage *image);
	bool GetCube(unsigned number, Z::CubeDef &cd, bool &masked);
	bool GetCylinder(unsigned number, Z::CylDef &cd, bool &masked);
	bool GetFlap(unsigned number, Z::FlapDef &fd, bool &masked);

	void SetActiveDrawArea(IplImage *iI);
	void DrawGestalts(int type, int detail);
	void DrawUnmaskedGestalts(int type, int detail);

	void SetCamParameters(double *intrinsic, double *dist, double *extrinsic);
};



}

#endif
