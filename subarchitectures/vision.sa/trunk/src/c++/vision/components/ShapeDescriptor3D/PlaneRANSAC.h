/**
 * @author Michael Zillich
 * @date February 2010
 */

#ifndef PLANE_RANSAC_H
#define PLANE_RANSAC_H

#include <vector>
#include <VisionData.hpp>

namespace cast
{

using namespace std;
using namespace cogx;
using namespace cogx::Math;
using namespace VisionData;

class PlaneRANSAC
{
private:
  double inlierThr;
  /**
   * max allowed probability of failure, i.e. that we do not find a solution
   */
  double eta0;
  /**
   * absolute max number of iterations (more just gets too slow)
   */
  int maxIter;
  /**
   * need at least this many points to accept a plane hypothesis
   */
  int minSupportPoints;

  bool selectSample(const SurfacePointSeq &points, int n,
    const Vector3 &up, bool horizontal, Vector3 &p, Vector3 &q, Vector3 &r);
  bool selectSampleNear(const SurfacePointSeq &points, int nPoints,
      Vector3 &p, Vector3 &q, Vector3 &r);
  bool computeModel(const Vector3 &p, const Vector3 &q,
    const Vector3 &r, Plane3 &plane);
  double fitError(const SurfacePointSeq &points, int n,
    const Plane3 &plane, int &nInliers);

public:
  /**
   * @param thr inlier threshold
   * @param _minPoints minimum number of points required for a plane,
   *        a very nasty threshold parameter ...
   * @param _maxIter maximum number of iterations before we give up
   */
  PlaneRANSAC(double _thr, int _minPoints = 1000, int _maxIter = 1000);
  bool detectPlane(SurfacePointSeq &points, int &n, const Vector3 &up,
    bool horizontal, Plane3 &plane);
};

}

#endif
