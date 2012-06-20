/**
 * @file Closure3D.cpp
 * @author Andreas Richtsfeld
 * @date January 2011
 * @version 0.1
 * @brief Base class for stereo calculated Closures in 3D.
 */

#include "Closure3D.h"

namespace Z
{

/**
 * @brief Constructor of Class Closure3D
 */
Closure3D::Closure3D() : Gestalt3D(Gestalt3D::CLOSURE)
{
}

/**
 * @brief Constructor of Class Closure3D for KinectClosure
 */
Closure3D::Closure3D(std::vector<cv::Vec4f> _plane, std::vector<cv::Vec4f> _hull) : Gestalt3D(Gestalt3D::CLOSURE)
{
  plane = _plane;
  hull = _hull;
    
  int nr_bins = 10;
  hist = new ColorHistogram(nr_bins, plane);
}

/**
 * @brief Compare histogram of closure with another closure's histogram.
 * @param p Patch to compare with
 */
double Closure3D::Compare(Closure3D *c)
{
  return hist->Compare(c->hist);
}

void Closure3D::CalculateSignificance(double sigLeft, double sigRight)
{
  sig = sigLeft * sigRight;
//   printf(" Sigs: %4.1f/%4.1f => sig: %4.1f\n", sigLeft, sigRight, sig);
}

/**
 * @brief Draw this 3D Gestalt to the TomGine render engine.
 * @param tgRenderer Render engine
 */
void Closure3D::DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool randomColor)
{
  RGBValue color;
  color.float_value = hull[0][3];
  for(unsigned i=0; i < hull.size()-1; i++)
    tgRenderer->AddLine3D(hull[i][0], hull[i][1], hull[i][2], hull[i+1][0], hull[i+1][1], hull[i+1][2], color.r, color.g, color.b);
  tgRenderer->AddLine3D(hull[0][0], hull[0][1], hull[0][2], hull[hull.size()-1][0], hull[hull.size()-1][1], hull[hull.size()-1][2], color.r, color.g, color.b);
}

}


