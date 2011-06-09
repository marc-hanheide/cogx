/**
 * @file Patch3D.h
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Base class for plane patches in 3D.
 */

#ifndef Z_PATCH3D_HH
#define Z_PATCH3D_HH

#include <vector>
#include "Gestalt3D.h"
#include "StereoTypes.h"
#include "StereoCore.h"
// #include "VisionUtils.h"
#include "ColorHistogram.h"

namespace Z
{

/**
 * @brief Class Patch3D
 */
class Patch3D : public Gestalt3D
{
private:
 std::vector<cv::Vec4f> points;           ///< All (projected) points of the plane patch
 std::vector<cv::Vec4f> hull_points;      ///< Hull points of the plane

public:
  ColorHistogram *hist;                   ///< ColorHistogram of patch
  
public:
//   Patch3D(cv::Mat_<cv::Vec4f> _p, cv::Mat_<cv::Vec4f> _h_p);
  Patch3D(std::vector<cv::Vec4f> _p, std::vector<cv::Vec4f> _h_p);

  void CalculateSignificance(double angle2Dleft, double angle2Dright, double angle3Dz);
  bool GetLinks(vector<GraphLink> &links);
  
  // Learning functions
  double CompareColor(Patch3D *p);
  double IsClose(Patch3D *p);
  
  void DrawGestalt3D(TGThread::TomGineThread *tgRenderer, bool randomColor = true);
  void PrintGestalt3D();
};


inline Array<Gestalt3D*>& Patches3D(StereoCore *score)
{
  return score->Gestalts3D(Gestalt3D::PATCH);
}
inline Patch3D* Patches3D(StereoCore *score, unsigned id)
{
  return (Patch3D*)score->Gestalts3D(Gestalt3D::PATCH, id);
}
inline unsigned NumPatches3D(StereoCore *score)
{
  return score->NumGestalts3D(Gestalt3D::PATCH);
}


}

#endif
