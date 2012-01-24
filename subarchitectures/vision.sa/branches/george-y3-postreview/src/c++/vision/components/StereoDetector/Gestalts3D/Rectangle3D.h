/**
 * @file Rectangle3D.h
 * @author Andreas Richtsfeld
 * @date Januray 2011
 * @version 0.1
 * @brief Base class for stereo calculated Rectangles in 3D.
 */

#ifndef Z_RECTANGLE3D_HH
#define Z_RECTANGLE3D_HH

#include "Gestalt3D.h"
#include "StereoTypes.h"
#include "StereoCore.h"
#include "ColorHistogram.h"

namespace Z
{

/**
 * @brief Class Rectangle3D
 */
class Rectangle3D : public Gestalt3D
{
public:
  std::vector<cv::Vec4f> plane;       /// points of the rectangle on the fitted plane
  std::vector<cv::Vec4f> hull;        /// hull points of the rectangle (kinect)
  Surf3D surf;                        ///< 3D surface

  ColorHistogram *hist;               ///< ColorHistogram of rectangle points (plane)

public:
  Rectangle3D();
  Rectangle3D(std::vector<cv::Vec4f> _plane, std::vector<cv::Vec4f> _hull);

  double Compare(Rectangle3D *r);

  void CalculateSignificance(double sigLeft, double sigRight);
  unsigned GetSurfaceSize() {return surf.vertices.Size();}
  bool GetLinks(vector<GraphLink> &links);
  void DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool randomColor);
};


inline Array<Gestalt3D*>& Rectangles3D(StereoCore *score)
{
  return score->Gestalts3D(Gestalt3D::RECTANGLE);
}
inline Rectangle3D* Rectangles3D(StereoCore *score, unsigned id)
{
  return (Rectangle3D*)score->Gestalts3D(Gestalt3D::RECTANGLE, id);
}
inline unsigned NumRectangles3D(StereoCore *score)
{
  return score->NumGestalts3D(Gestalt3D::RECTANGLE);
}


}

#endif
