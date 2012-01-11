/**
 * @file Closure3D.cpp
 * @author Andreas Richtsfeld
 * @date January 2011
 * @version 0.1
 * @brief Base class for stereo calculated Closures in 3D.
 */

#ifndef Z_CLOSURE3D_HH
#define Z_CLOSURE3D_HH

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Gestalt3D.h"
#include "StereoTypes.h"
#include "StereoCore.h"
#include "ColorHistogram.h"

namespace Z
{

/**
 * @brief Class Closure3D
 */
class Closure3D : public Gestalt3D
{
public:
  std::vector<cv::Vec4f> plane;       /// points of the closure on the fitted plane
  std::vector<cv::Vec4f> hull;        /// hull points of the closure (kinect)
  Surf3D surf;                        ///< 3D surface

  ColorHistogram *hist;               ///< ColorHistogram of closure points (plane)

public:
  Closure3D();
  Closure3D(std::vector<cv::Vec4f> _plane, std::vector<cv::Vec4f> _hull);
  
  double Compare(Closure3D *c);

  void CalculateSignificance(double sigLeft, double sigRight);
  void DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool randomColor);

};


inline Array<Gestalt3D*>& Closures3D(StereoCore *score)
{
  return score->Gestalts3D(Gestalt3D::CLOSURE);
}
inline Closure3D* Closures3D(StereoCore *score, unsigned id)
{
  return (Closure3D*)score->Gestalts3D(Gestalt3D::CLOSURE, id);
}
inline unsigned NumClosures3D(StereoCore *score)
{
  return score->NumGestalts3D(Gestalt3D::CLOSURE);
}


}

#endif
