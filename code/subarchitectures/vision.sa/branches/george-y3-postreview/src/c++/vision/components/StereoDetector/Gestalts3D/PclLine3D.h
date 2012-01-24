/**
 * @file PclLine3D.h
 * @author Andreas Richtsfeld
 * @date September 2011
 * @version 0.1
 * @brief Base class for pcl-calculated lines in 3D.
 */

#ifndef Z_PCL_LINE3D_HH
#define Z_PCL_LINE3D_HH

#include "Gestalt3D.h"
#include "StereoTypes.h"
#include "StereoCore.h"

namespace Z
{

/**
 * @brief Class PclLine3D
 */
class PclLine3D : public Gestalt3D
{
public:
  std::vector<cv::Vec4f> points;       /// points of the rectangle on the fitted plane


public:
  PclLine3D();
  PclLine3D(std::vector<cv::Vec4f> _points);

  void CalculateSignificance(double sigLeft, double sigRight);
  void DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool randomColor, bool use_color = false, float color = 0.0);
};


inline Array<Gestalt3D*>& PclLines3D(StereoCore *score)
{
  return score->Gestalts3D(Gestalt3D::PCL_LINE);
}
inline PclLine3D* PclLines3D(StereoCore *score, unsigned id)
{
  return (PclLine3D*)score->Gestalts3D(Gestalt3D::PCL_LINE, id);
}
inline unsigned NumPclLines3D(StereoCore *score)
{
  return score->NumGestalts3D(Gestalt3D::PCL_LINE);
}


}

#endif
