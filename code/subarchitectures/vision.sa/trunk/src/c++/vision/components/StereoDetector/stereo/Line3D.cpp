/**
 * @file Line3D.cpp
 * @author Andreas Richtsfeld
 * @date February 2011
 * @version 0.1
 * @brief Base class for stereo calculated line in 3D.
 */

#include "Line3D.h"
#include "Line.hh"

namespace Z
{

/**
 * @brief Constructor of class Line3D.
 */
Line3D::Line3D(unsigned vs3IDleft, unsigned vs3IDright) : Gestalt3D(Gestalt3D::LINE)
{
  vs3IDs[0] = vs3IDleft;
  vs3IDs[1] = vs3IDright;
}


/**
 * @brief Calculate significance for the stereo Gestalt.
 * 2D horizontal lines are worser to match in 3D, therefore lower significance.
 * 3D Lines with main direction in z-coordinte.
 * @param angle2Dleft
 * @param angle2Dright 
 * @param angle3Dz Opening angle between z-axis and Line in 3D
 */
void Line3D::CalculateSignificance(double angle2Dleft, double angle2Dright, double angle3Dz)
{
  // Significance is between 
  double sig2Dleft = fabs(angle2Dleft/(M_PI/2.));
  double sig2Dright = fabs(angle2Dright/(M_PI/2.));
  double sig3D = 1- angle3Dz/(M_PI/2.);	
  
  sig = sig2Dleft*sig2Dright*sig3D;	

  //   printf("Line3D::CalculateSignificance: %4.3f / %4.3f => %4.3f / %4.3f\n", angle2Dleft, angle2Dright, sig2Dleft, sig2Dright);
//   printf("Line3D::CalculateSignificance: %4.3f => %4.3f\n", angle3Dz, sig3D);
//   printf("Line3D::CalculateSignificance: sig = %4.3f\n", sig);
}

}


