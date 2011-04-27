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
	processingTime = 600;
	cannyAlpha = 800;
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
	static bool firstCall = true;
	if(firstCall) vcore->InitBuffer(image, true);
	else vcore->SetBuffer(image);
	firstCall = false;

  vcore->ProcessImage(processingTime, cannyAlpha, cannyOmega, false);
	return 0;
}


/**
 * @brief Get all cubes from the vision system (vs3).
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
 * @brief Get all cylinders from the vision system (vs3).
 * @param number Number of requested cylinder
 * @param cd Cylinder parameter
 * @param masked True, if cylinder is masked.
 * @return Return true, if cylinder exists.
 */
bool Vs3Interface::GetCylinder(unsigned number, Z::CylDef &cd, bool &masked)
{
	if(vcore->GetCylinder(number, cd, masked)) return true;
	else return false;
}


/**
 * @brief Get all flaps from the vision system (vs3).
 * @param number Number of requested flap
 * @param cd Flap parameter
 * @param masked True, if flap is masked.
 * @return Return true, if flap exists.
 */
bool Vs3Interface::GetFlap(unsigned number, Z::FlapDef &fd, bool &masked)
{
	if(vcore->GetFlap(number, fd, masked)) return true;
	else return false;
}

/**
 * @brief Set active draw area as the OpenCv IplImage
 * @param iI IplImage
 */
void Vs3Interface::SetActiveDrawArea(IplImage *iI)
{
	int detail = 0; 	// Degree of detail for drawing Gestalts.
	vcore->DrawArea(iI);
}

/**
 * @brief Draw Gestalts into OpenCv IplImage
 * @param type Type of Gestalt to draw
 * @param detail Degree of detail for drawing
 */
void Vs3Interface::DrawGestalts(int type, int detail)
{
	vcore->DrawGestalts((Z::Gestalt::Type) type, detail);
}

/**
 * @brief Draw Gestalts into OpenCv IplImage
 * @param type Type of Gestalt to draw
 * @param detail Degree of detail for drawing
 */
void Vs3Interface::DrawUnmaskedGestalts(int type, int detail)
{
	vcore->DrawUnmaskedGestalts((Z::Gestalt::Type) type, detail);
}

/**
 * @brief Set parameters of the camera for the object detector
 * @param intrinsic Intrinsic parameters fx, fy, cx, cy
 * @param distortion Radial and tangential distortion: k1, k2, p1, p2
 */
void Vs3Interface::SetCamParameters(double *intrinsic, double *dist, double *extrinsic)
{
	vcore->SetCamParameters(intrinsic, dist, extrinsic);
}

}

