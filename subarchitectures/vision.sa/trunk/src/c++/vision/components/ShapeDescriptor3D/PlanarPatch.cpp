/**
 * @author Michael Zillich
 * @date September 2010
 */

#include <ios>
#include <cv.h>
#include "PlanarPatch.h"

using namespace cogx::Math;

PlanarPatch::PlanarPatch(Plane3 &p)
{
  plane = p;
  definePlaneCoordSys(plane, pose);
}

void PlanarPatch::findConvexHull(std::vector<VisionData::SurfacePoint> &points, int nPoints, double distThr)
{
  std::vector<CvPoint2D32f> planePoints;
  for(size_t j = 0; j < nPoints; j++)
  {
    // if inlier
    if(fabs(distPointToPlane(points[j].p, plane)) < distThr)
    {
      Vector3 q = transformInverse(pose, points[j].p);
      planePoints.push_back(cvPoint2D32f(q.x, q.y));
    }
  }
  int *hull = (int*)malloc(planePoints.size() * sizeof(hull[0]));
  CvMat hullMat = cvMat(1, planePoints.size(), CV_32SC1, hull);
  CvMat pointMat = cvMat(1, planePoints.size(), CV_32FC2, &planePoints[0]);
  cvConvexHull2(&pointMat, &hullMat, CV_COUNTER_CLOCKWISE, 0);

  for(int j = 0; j < hullMat.cols; j++)
  {
    Vector3 q = vector3(planePoints[hull[j]].x, planePoints[hull[j]].y, 0.);
    convexHull.push_back(transform(pose, q));
    projectedConvexHull.push_back(vector2(q.x, q.y));
  }

  free(hull);

  // HACK test
  Vector3 m = vector3(0,0,0);
  for(size_t i = 0; i < convexHull.size(); i++)
  {
    m += convexHull[i];
  }
  m /= (double)convexHull.size();
  std::cout << "PlanarPatch::findConvexHull, check center inside? " << isPointInside(m) << endl;
}

bool PlanarPatch::isPointInside(Vector3 &q) const
{
  int i, j;
  const std::vector<Vector2> &h = projectedConvexHull;
  Vector3 pp = transformInverse(pose, q);
  Vector2 p = vector2(pp.x, pp.y);
  for(i = 0, j = h.size() - 1; i < h.size(); j = i++)
  {
    Vector2 a = h[i] - h[j];
    Vector2 b = p - h[j];
    if(!leftOf(a, b))
      return false;
  }
  return true;

  /*int i, j;
  bool c = false;
  const std::vector<Vector2> &h = projectedConvexHull;
  Vector3 p = transformInverse(pose, q);
  for(i = 0, j = h.size() - 1; i < h.size(); j = i++)
  {
    if((h[i].y<=p.y) && (p.y< (h[j].x - h[i].x) * (p.y - h[i].y) / (h[j].y - h[i].y) + h[i].x))
      c = !c;
  }
  return c;*/
}
