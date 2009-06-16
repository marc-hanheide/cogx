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

#include "CubeDefinition.hh"
#include "Gestalt.hh"

namespace cast
{

/**
 * @brief Constructor of class VisionSystem3
 */
Vs3Interface::Vs3Interface()
{
	config = "subarchitectures/vision.sa/src/c++/vision/components/ObjectDetector/cfg/vs3.cfg";
  vcore = new Z::VisionCore(Z::VIDEO_TYPE_BUFFER, config);
  assert(vcore != 0);	

	// set processing time and canny parameters
	processingTime = 400;
	cannyAlpha = 600;
	cannyOmega = 1;
}

/**
 * @brief Destructor of class VisionSystem3
 */
Vs3Interface::~Vs3Interface()
{
  delete vcore;
}


/**
 * @brief Constructor of class VisionSystem3
 * @image IplImage Image to process.
 */
int Vs3Interface::ProcessSingleImage(IplImage *image)
{
// 	printf("Vs3Interface::ProcessSingleImage\n");

	static bool firstCall = true;
	if(firstCall) vcore->InitBuffer(image, true);
	else vcore->SetBuffer(image);
	firstCall = false;

// 	vcore->NewImage(image);

  vcore->ProcessImage(processingTime, cannyAlpha, cannyOmega, false);
	return 0;
}


/**
 * @brief Get all objects from the vision system (vs3).
 * @param number Number of requested cube
 * @param cd Cube parameter
 * @param masked True, if cube is masked.
 * @return Return true, if cube exists.
 */
bool Vs3Interface::GetCube(unsigned number, Z::CubeDef &cd, bool &masked)
{
	if(vcore->GetCube(number, cd, masked)) return true;
	else return false;
}

/**
 * @brief Draw Gestalts into OpenCv IplImage
 * @param iI IplImage
 */
void Vs3Interface::Draw(IplImage *iI)
{
	int detail = 0; 	// Degree of detail for drawing Gestalts.
	vcore->DrawToIplImage(iI);
}

/**
 * @brief Draw Gestalts into OpenCv IplImage
 * @param iI IplImage
 */
void Vs3Interface::DrawGestalt(int type, int detail)
{
	vcore->DrawGestalts((Z::Gestalt::Type) type, detail);
}

/**
 * @brief Set parameters of the camera for the object detector
 * @param intrinsic Intrinsic parameters fx, fy, cx, cy
 * @param distortion Radial and tangential distortion: k1, k2, p1, p2
 */
void Vs3Interface::SetCamParameters(double *intrinsic, double *distortion, double *extrinsic)
{
	vcore->SetCamParameters(intrinsic, distortion, extrinsic);
}

}

