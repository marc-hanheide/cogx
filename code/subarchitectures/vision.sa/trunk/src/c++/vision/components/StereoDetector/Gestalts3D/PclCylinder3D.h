/**
 * @file PclCylinder3D.h
 * @author Andreas Richtsfeld
 * @date September 2011
 * @version 0.1
 * @brief Base class for pcl-calculated cylinders in 3D.
 */

#ifndef Z_PCL_CYLINDER3D_H
#define Z_PCL_CYLINDER3D_H

#include "Gestalt3D.h"
#include "StereoTypes.h"
#include "StereoCore.h"

namespace Z
{

/**
 * @brief Class PclCylinder3D
 */
class PclCylinder3D : public Gestalt3D
{
public:
  std::vector<cv::Vec4f> points;        ///< points of the rectangle on the fitted plane
  std::vector<int> indices;             ///< Indices of the points refering to 2D image plane
  std::vector<int> mask_hull_indices;             ///< 2D indexes of the points (in image space)
  bool calcCenter3D;                    ///< True, if center3D is already calculated
  cv::Vec3f center3D;                   ///< 3D center point

  void CalcCenter3D();

public:
  PclCylinder3D();
  PclCylinder3D(std::vector<cv::Vec4f> _points, 
                std::vector<int> _indices,
                std::vector<int> _mask_hull_indices);

  void GetPoints(std::vector<cv::Vec4f> &p) {p = points;}
  void GetIndices(std::vector<int> &i) {i = indices;}

  void CalculateSignificance(double sigLeft, double sigRight);
  void GetCenter3D(cv::Vec3f &c) {CalcCenter3D(); c = center3D;}
  cv::Vec3f GetCenter3D() {CalcCenter3D(); return center3D;}

  void DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool randomColor, bool use_color = false, float color = 0.0);
};


inline Array<Gestalt3D*>& PclCylinders3D(StereoCore *score)
{
  return score->Gestalts3D(Gestalt3D::PCL_CYLINDER);
}
inline PclCylinder3D* PclCylinders3D(StereoCore *score, unsigned id)
{
  return (PclCylinder3D*)score->Gestalts3D(Gestalt3D::PCL_CYLINDER, id);
}
inline unsigned NumPclCylinders3D(StereoCore *score)
{
  return score->NumGestalts3D(Gestalt3D::PCL_CYLINDER);
}


}

#endif
