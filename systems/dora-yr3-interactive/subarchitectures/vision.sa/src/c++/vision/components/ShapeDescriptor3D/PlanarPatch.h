/**
 * @author Michael Zillich
 * @date September 2010
 */

#ifndef PLANAR_PATCH_H
#define PLANAR_PATCH_H

#include <vector>
#include <cogxmath.h>
#include <VisionData.hpp>

class PlanarPatch
{
public:
  cogx::Math::Plane3 plane;
  cogx::Math::Pose3 pose;
  std::vector<cogx::Math::Vector3> convexHull;
  std::vector<cogx::Math::Vector2> projectedConvexHull;
  std::vector<VisionData::SurfacePoint> surfPoints;

  /**
   * returns whether a point, projected onto the plane lies inside the projected convex hull
   */
  bool isPointInside(cogx::Math::Vector3 &p) const;
};

#endif
