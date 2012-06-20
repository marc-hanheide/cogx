/**
 * @author Michael Zillich
 * @date February 2010
 */

#ifndef PLANE_RANSAC_H
#define PLANE_RANSAC_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <VisionData.hpp>
#include "PlanarPatch.h"

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
  bool findConnectedRegions;
  IplImage *voteImg;
  IplImage *maskImg;
  float gridSize;

  bool selectSample(const SurfacePointSeq &points, int n,
    const Vector3 &up, bool horizontal, Vector3 &p, Vector3 &q, Vector3 &r);
  bool selectSampleNear(const SurfacePointSeq &points, int nPoints,
      Vector3 &p, Vector3 &q, Vector3 &r);
  bool computeModel(const Vector3 &p, const Vector3 &q,
    const Vector3 &r, PlanarPatch &plane);
  double fitError(const SurfacePointSeq &points, int n,
    const PlanarPatch &plane, int &nInliers);
  double fitErrorConnectedRegion(const SurfacePointSeq &points, int nPoints,
    const PlanarPatch &plane, int &nInliers);
  double vote(const SurfacePointSeq &points, int nPoints,
    const PlanarPatch &plane, int &nInliers, bool saveImages = false);

public:
  /**
   * @param thr inlier threshold
   * @param _minPoints minimum number of points required for a plane,
   *        a very nasty threshold parameter ...
   * @param _maxIter maximum number of iterations before we give up
   */
  PlaneRANSAC(double _thr, bool connectedRegions, int _minPoints = 1000, int _maxIter = 1000);
  ~PlaneRANSAC();
  bool detectPlane(SurfacePointSeq &points, int &n, const Vector3 &up,
    bool horizontal, PlanarPatch &plane);
  IplImage* getVoteImage() {return voteImg;}
  void findConvexHull(const SurfacePointSeq &points, int nPoints,
    PlanarPatch &plane, double thrFactor = 1.);
  void findConvexHullConnectedRegion(const SurfacePointSeq &points, int nPoints,
    PlanarPatch &plane);
  void removeInliers(SurfacePointSeq &points, int &n,
    const PlanarPatch &plane, double thrFactor = 1.);
};

}

#endif
