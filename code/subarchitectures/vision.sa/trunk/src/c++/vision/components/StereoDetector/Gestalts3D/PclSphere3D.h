/**
 * @file PclSphere3D.h
 * @author Andreas Richtsfeld
 * @date September 2011
 * @version 0.1
 * @brief Base class for pcl-calculated spheres in 3D.
 */

#ifndef Z_PCL_SPHERE3D_H
#define Z_PCL_SPHERE3D_H

#include "Gestalt3D.h"
#include "StereoTypes.h"
#include "StereoCore.h"

namespace Z
{

/**
 * @brief Class PclSphere3D
 */
class PclSphere3D : public Gestalt3D
{
public:
  std::vector<cv::Vec4f> points;            /// points of the sphere hull
  std::vector<int> indices;                 ///< Indices of the points refering to 2D image plane
  std::vector<int> mask_hull_indices;       ///< 2D indexes of the points (in image space)
  cv::Vec3f center3D;                       ///< 3D center point of the sphere // TODO TODO not done!!!

public:
  PclSphere3D(std::vector<cv::Vec4f> _points, 
              std::vector<int> _indices,
              std::vector<int> _mask_hull_indices);

  void GetPoints(std::vector<cv::Vec4f> &p) {p = points;}
  void GetIndices(std::vector<int> &i) {i = indices;}

  cv::Vec3f GetCenter3D() {return center3D;}
  void GetCenter3D(cv::Vec3f &c) {c = center3D;}

  void CalculateSignificance(double sigLeft, double sigRight);
  void DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool randomColor, bool use_color = false, float color = 0.0);
};


inline Array<Gestalt3D*>& PclSpheres3D(StereoCore *score)
{
  return score->Gestalts3D(Gestalt3D::PCL_SPHERE);
}
inline PclSphere3D* PclSpheres3D(StereoCore *score, unsigned id)
{
  return (PclSphere3D*)score->Gestalts3D(Gestalt3D::PCL_SPHERE, id);
}
inline unsigned NumPclSpheres3D(StereoCore *score)
{
  return score->NumGestalts3D(Gestalt3D::PCL_SPHERE);
}


}

#endif
