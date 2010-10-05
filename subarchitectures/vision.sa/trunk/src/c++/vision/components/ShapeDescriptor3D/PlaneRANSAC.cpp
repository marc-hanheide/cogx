/**
 * @author Michael Zillich
 * @date February 2010
 */

#include <cassert>
#include <fstream>
#include <highgui.h>
#include "cogxmath.h"
#include "AccessImage.h"
#include "PlaneRANSAC.h"

namespace cast
{

//#define PLANE_RANSAC_DEBUG

using namespace std;
using namespace cogx;
using namespace cogx::Math;
using namespace VisionData;

PlaneRANSAC::PlaneRANSAC(double _thr, bool connectedRegions, int _minPoints, int _maxIter)
{
  findConnectedRegions = connectedRegions;
  inlierThr = _thr;
  // TODO: get rid of this arbitrary tuning parameter
  maxIter = _maxIter;
  eta0 = 0.01;
  // TODO: get rid of this arbitrary tuning parameter
  minSupportPoints = _minPoints;
  if(findConnectedRegions)
  {
    // HACK: this should be set from nearest neighbour distance, e.g. mean + 3 sigma
    gridSize = 0.005;
    voteImg = cvCreateImage(cvSize(400, 400), IPL_DEPTH_32F, 1);
    maskImg = cvCreateImage(cvSize(voteImg->width+2, voteImg->height+2), IPL_DEPTH_8U, 1);
  }

  assert(inlierThr > 0.);
  assert(maxIter > 0);
  assert(eta0 > 0.);
  assert(minSupportPoints > 0);
}

PlaneRANSAC::~PlaneRANSAC()
{
  if(findConnectedRegions)
  {
    cvReleaseImage(&voteImg);
    cvReleaseImage(&maskImg);
  }
}

// HACK: collect statistics for samples
/*static int distCnt = 0;
static double distMean = 0.;*/

/**
 * Randomly select a minimum sample set of 3 points.
 * If horizontal is true then only (roughly) horizonal plane hypotheses will
 * be accepted, where horizonal is defined via the up vector.
 * @param points (in) array of data points
 * @param nPoints (in) number of data points to use, the array might be longer
 *                but the remaining points are irrelevant (e.g. because they
 *                have already been assigned to a hypothesis)
 * @param up (in) vector defining the up direction and hence horizontal
 * @param horizonal (in) if true, accept only (roughly) horizontal plane hypotheses
 * @param p (out) first sample point
 * @param q (out) second sample point
 * @param r (out) third sample point
 * @returns true if a sample could be found (Note that there simply might be no
 *          horizontal plane)
 */
bool PlaneRANSAC::selectSample(const SurfacePointSeq &points, int nPoints,
    const Vector3 &up, bool horizontal, Vector3 &p, Vector3 &q, Vector3 &r)
{
  int i, j, k, cnt;
  i = rand()%nPoints;
  j = i;
  k = i;
  p = points[i].p;
  cnt = 0;
  // Checking for cnt ensures that we do not loop forever (which could
  // happen if no sample of points meets the horizontal condition). So
  // essentially we want to stop after we tried all points. Checking for
  // cnt < nPoints does not really ensure that as rand() might pick the same number
  // twice, but more or less gets us there.
  while(cnt < nPoints && j == i)
  {
    j = rand()%nPoints;
    q = points[j].p;
    if(horizontal)
    {
      Vector3 pq = q - p;
      // if vector p-q is not normal to up vector then reject choice of j
      // HACK: get rid of stupid threshold 0.01!
      if(isZero(pq) || dot(up, pq) > 0.01)
        j = i;
    }
    cnt++;
  }
  while(cnt < nPoints && (k == i || k == j))
  {
    k = rand()%nPoints;
    r = points[k].p;
    if(horizontal)
    {
      Vector3 pr = r - p;
      // if vector p-r is not normal to up vector then reject choice of k
      if(isZero(pr) || dot(up, pr) > 0.01)
        k = i;
    }
    cnt++;
  }
  // HACK
  /*if(cnt < nPoints)
  {
    distMean += dist(p, q);
    distMean += dist(q, r);
    distMean += dist(r, p);
    distCnt += 3;
  }*/
  return cnt < nPoints;
}

/**
 * Select a minimum sample set of 3 points, where the second and third point
 * are chosen "near" the first (and the 3rd should be chosen such that the 3 points are
 * not too collinear).
 * NOTE: does not noticably improve results. especially choosing sample points to be too
 * near to each other degrades performance.
 */
bool PlaneRANSAC::selectSampleNear(const SurfacePointSeq &points, int nPoints,
    Vector3 &p, Vector3 &q, Vector3 &r)
{
  // note: we don't want the sample points to be "too near", as then the estimated
  // plane hypothesis is unstable. therefore chose a "good" distance (which should
  // evantually be estimated from the data, e.g. from average distance)
  double d_ideal = 0.02;
  int i, j, k, cnt;
  i = rand()%nPoints;
  j = i;
  k = i;
  p = points[i].p;
  cnt = 0;
  while(cnt < nPoints && j == i)
  {
    double d, d_min = HUGE_VAL;
    int j_min = j;
    for(int l = 0; l < 10; l++)
    {
      j = rand()%nPoints;
      if(j != i)
      {
        q = points[j].p;
        d = dist(p, q);
        if(fabs(d - d_ideal) < d_min)
        {
          d_min = d;
          j_min = j;
        }
      }
    }
    cnt++;
    j = j_min;
    q = points[j_min].p;
  }
  while(cnt < nPoints && (k == i || k == j))
  {
    double d, d_min = HUGE_VAL;
    int k_min = k;
    for(int l = 0; l < 10; l++)
    {
      k = rand()%nPoints;
      if(k != i && k != j)
      {
        r = points[k].p;
        d = dist(p, r);
        if(fabs(d - d_ideal) < d_min)
        {
          d_min = d;
          k_min = k;
        }
      }
    }
    cnt++;
    k = k_min;
    r = points[k_min].p;
  }
  // HACK
  /*if(cnt < nPoints)
  {
    distMean += dist(p, q);
    distMean += dist(q, r);
    distMean += dist(r, p);
    distCnt += 3;
  }*/
  return cnt < nPoints;
}

/**
 * Given a minimum sample set, compute model hypothesis.
 * @returns true if model could be computed (point were not in a degenerate position)
 */
bool PlaneRANSAC::computeModel(const Vector3 &p, const Vector3 &q,
    const Vector3 &r, PlanarPatch &plane)
{
  if(planeFromPoints(plane.plane, p, q, r))
  {
    // any point can be origin, take p
    plane.pose.pos = p;
    definePlaneAxes(plane.plane, plane.pose.rot);
    return true;
  }
  else
    return false;
}

/**
 * Returns the mean relative error.
 * Relative means the error relative to the outlier threshold, so an outlier has
 * error 1, an inlier < 1.
 * Mean is over all n points.
 * @param points (in) array of data points
 * @param nPoints (in) number of data points to use, the array might be longer
 *                but the remaining points are irrelevant (e.g. because they
 *                have already been assigned to a hypothesis)
 * @param plane (in) the plane hypothesis
 * @param nInliers (out) number of inliers
 * @return mean relative fit error
 */
double PlaneRANSAC::fitError(const SurfacePointSeq &points, int nPoints,
    const PlanarPatch &plane, int &nInliers)
{ 
  double sumErr = 0.;
  nInliers = 0;
  // note: using MSAC, i.e. summing errors instead of counting inliers
  for(int i = 0; i < nPoints; i++)
  {
    double err = fabs(distPointToPlane(points[i].p, plane.plane));
    if(err >= inlierThr)
      err = inlierThr;
    else
      nInliers++;
    sumErr += err/inlierThr;
  }
  sumErr /= (double)nPoints;
  return sumErr;
}

/**
 * Fill vote image with 3D points projected on the plane and find connected
 * region centered around projected plane origin.
 * @return average error of points inside the connected region
 */
double PlaneRANSAC::vote(const SurfacePointSeq &points, int nPoints,
    const PlanarPatch &plane, int &nInliers, bool saveImages)
{
  cvSet(voteImg, cvScalarAll(0));
  cvSet(maskImg, cvScalarAll(0));
  Video::AccessBwImageFloat grid(voteImg);
  CvPoint seed = cvPoint(voteImg->width/2,
                         voteImg->height/2);
  for(int i = 0; i < nPoints; i++)
  {
    Vector3 q = transformInverse(plane.pose, points[i].p);
    double err = fabs(distPointToPlane(points[i].p, plane.plane))/inlierThr;
    int x = voteImg->width/2 + lrintf(q.x/gridSize);
    int y = voteImg->height/2 + lrintf(q.y/gridSize);
    // inliers (err < 1) have votes > 1 and <= 2, outliers (err >= 1) don't vote
    if(err < 1.)
      if(x >= 0 && x < voteImg->width && y >= 0 && y < voteImg->height)
        grid[y][x] = 2. - err;
  }
#ifdef PLANE_RANSAC_DEBUG
  if(saveImages)
    cvSaveImage("ransac-vote-1.png", voteImg);
#endif
  CvConnectedComp connRegion;
  cvFloodFill(
      voteImg,
      seed,
      cvScalarAll(2.),  // fill value does not matter actually
      cvScalarAll(1.),  // low diff
      cvScalarAll(1.),  // up diff
      &connRegion,
      8 | CV_FLOODFILL_FIXED_RANGE | CV_FLOODFILL_MASK_ONLY,  // flags: 8 connectivity plus flags
      maskImg
      );
  nInliers = (int)connRegion.area;
  
#ifdef PLANE_RANSAC_DEBUG
  if(saveImages)
  {
    cvSaveImage("ransac-vote-2.png", voteImg);
    cvSaveImage("ransac-mask.png", maskImg);
    Video::AccessBwImage maskGrid(maskImg);
    // note: mask image has a border of 1 pixel
    printf("seed: %d %d = %lf / %d, ransc conn region area %lf, avg val %lf\n",
      seed.x, seed.y, grid[seed.y][seed.x], maskGrid[seed.y+1][seed.x+1],
      connRegion.area, 2. - connRegion.value.val[0]);
  }
#endif
  return 2. - connRegion.value.val[0];
}

/**
 * Returns the fit error of a connected region. This is meant to avoid accidental
 * support from isolated points.
 */
double PlaneRANSAC::fitErrorConnectedRegion(const SurfacePointSeq &points, int nPoints,
    const PlanarPatch &plane, int &nInliers)
{
  double avgErr = vote(points, nPoints, plane, nInliers);
  return (nPoints - nInliers*avgErr)/nPoints;
}

void PlaneRANSAC::findConvexHull(const SurfacePointSeq &points, int nPoints,
    PlanarPatch &plane, double thrFactor)
{
  std::vector<CvPoint2D32f> planePoints;
  for(int j = 0; j < nPoints; j++)
  {
    if(fabs(distPointToPlane(points[j].p, plane.plane)) < inlierThr*thrFactor)
    {
      plane.surfPoints.push_back(points[j]);
      Vector3 q = transformInverse(plane.pose, points[j].p);
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
    plane.convexHull.push_back(transform(plane.pose, q));
    plane.projectedConvexHull.push_back(vector2(q.x, q.y));
  }

  free(hull);
}

void PlaneRANSAC::findConvexHullConnectedRegion(const SurfacePointSeq &points, int nPoints,
    PlanarPatch &plane)
{
  Video::AccessBwImage maskGrid(maskImg);
  std::vector<CvPoint2D32f> planePoints;
  int nInliers;

  vote(points, nPoints, plane, nInliers);

  for(int i = 0; i < nPoints; i++)
  {
    if(fabs(distPointToPlane(points[i].p, plane.plane)) < inlierThr)
    {
      Vector3 q = transformInverse(plane.pose, points[i].p);
      int x = voteImg->width/2 + lrintf(q.x/gridSize);
      int y = voteImg->height/2 + lrintf(q.y/gridSize);
      // note: mask image has a border of 1 pixel
      if(maskGrid[y+1][x+1] == 1)
      {
        plane.surfPoints.push_back(points[i]);
        Vector3 q = transformInverse(plane.pose, points[i].p);
        planePoints.push_back(cvPoint2D32f(q.x, q.y));
      }
    }
  }

  int *hull = (int*)malloc(planePoints.size() * sizeof(hull[0]));
  CvMat hullMat = cvMat(1, planePoints.size(), CV_32SC1, hull);
  CvMat pointMat = cvMat(1, planePoints.size(), CV_32FC2, &planePoints[0]);
  cvConvexHull2(&pointMat, &hullMat, CV_COUNTER_CLOCKWISE, 0);

  for(int i = 0; i < hullMat.cols; i++)
  {
    Vector3 q = vector3(planePoints[hull[i]].x, planePoints[hull[i]].y, 0.);
    plane.convexHull.push_back(transform(plane.pose, q));
    plane.projectedConvexHull.push_back(vector2(q.x, q.y));
  }

  free(hull);
}

/**
 * Removes inliers from points.
 * To pe precise: inliers are moved to end of points array and n decreased
 * accordingly. So after call to this function points[0..n-1] are all outliers.
 */
void PlaneRANSAC::removeInliers(SurfacePointSeq &points, int &n,
    const PlanarPatch &plane, double thrFactor)
{
  SurfacePoint t;
  for(int i = 0; i < n; )
  {
    // if inlier
    if(plane.isPointInside(points[i].p) &&
       fabs(distPointToPlane(points[i].p, plane.plane)) < inlierThr*thrFactor)
    {
      // move point i to end of points and decrease n
      t = points[i];
      points[i] = points[n - 1];
      points[n - 1] = t;
      n--;
    }
    else
    {
      i++;
    }
  }
}

/**
 * The actual RANSAC iteration for plane detection.
 * If horizontal is true then only (roughly) horizonal plane hypotheses will
 * be accepted, where horizonal is defined via the up vector.
 * @param points (in) array of data points
 * @param nPoints (in) number of data points to use, the array might be longer
 *                but the remaining points are irrelevant (e.g. because they
 *                have already been assigned to a hypothesis)
 * @param up (in) vector defining the up direction and hence horizontal
 * @param horizonal (in) if true, accept only (roughly) horizontal plane hypotheses
 * @param plane (out) the estimated plane
 * @returns true if any plane could be found
 */
bool PlaneRANSAC::detectPlane(SurfacePointSeq &points, int &nPoints, const Vector3 &up,
    bool horizontal, PlanarPatch &plane)
{
  // current estimate of inlier ratio, initial value
  double eps = 2./(double)nPoints;
  int nInliersOpt = 0;
  double minError = HUGE_VAL;
  int k = 0;

  // need at least 3 points
  if(nPoints < 3)
    return false;

  // HACK
  /*distMean = 0.;
  distCnt = 0;*/

  // while the probability of failure is still too high (and - as a safeguard -
  // we haven't yet reached the absolute maximum number of iteratrions)
  while(pow(1. - pow(eps, 2.), (double)k) >= eta0 && k < maxIter)
  {
    Vector3 p, q, r;
    bool gotSample;
    if(findConnectedRegions)
      gotSample = selectSampleNear(points, nPoints, p, q, r);
    else
      gotSample = selectSample(points, nPoints, up, horizontal, p, q, r);
    if(gotSample)
    {
      PlanarPatch planeHyp;
      if(computeModel(p, q, r, planeHyp))
      {
        int nInliers = 0;
        double error;
        if(findConnectedRegions)
          error = fitErrorConnectedRegion(points, nPoints, planeHyp, nInliers);
        else
          error = fitError(points, nPoints, planeHyp, nInliers);
        if(error < minError)
        {
          minError = error;
          nInliersOpt = nInliers;
          plane = planeHyp;
          eps = (double)nInliersOpt/(double)nPoints;
        }
      }
    }
    k = k + 1;
  }
  //printf("RANSAC iterations: %d, inlier ratio: %.2f\n", k, eps);
  // HACK
  /*distMean /= (double)distCnt;
  printf("mean sample point distance of %d: %lf\n", distCnt, distMean);*/

  // if we have a solution
  if(nInliersOpt > minSupportPoints)
  {
    // TODO: find least squares solution (and update inliers)
    // RANSAC_optimiseSolution();
    
    if(findConnectedRegions)
      findConvexHullConnectedRegion(points, nPoints, plane);
    else
      findConvexHull(points, nPoints, plane);
    // HACK
    /*int nIn;
    vote(points, nPoints, plane, nIn, true);*/
    return true;
  }
  else
  {
    return false;
  }
}

}
