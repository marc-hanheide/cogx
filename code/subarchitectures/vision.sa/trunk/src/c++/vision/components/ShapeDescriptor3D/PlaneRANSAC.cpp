/**
 * @author Michael Zillich
 * @date February 2010
 */

#include <cassert>
#include <fstream>
#include "cogxmath.h"
#include "PlaneRANSAC.h"

namespace cast
{

using namespace std;
using namespace cogx;
using namespace cogx::Math;
using namespace VisionData;

PlaneRANSAC::PlaneRANSAC(double _thr, int _minPoints, int _maxIter)
{
  inlierThr = _thr;
  // TODO: get rid of this arbitrary tuning parameter
  maxIter = _maxIter;
  eta0 = 0.01;
  // TODO: get rid of this arbitrary tuning parameter
  minSupportPoints = _minPoints;

  assert(inlierThr > 0.);
  assert(maxIter > 0);
  assert(eta0 > 0.);
  assert(minSupportPoints > 0);
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
    for(int l = 0; l < 100; l++)
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
    for(int l = 0; l < 100; l++)
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
    const Vector3 &r, Plane3 &plane)
{
  return planeFromPoints(plane, p, q, r);
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
    const Plane3 &plane, int &nInliers)
{ 
  double sumErr = 0.;
  nInliers = 0;
  // note: using MSAC, i.e. summing errors instead of counting inliers
  for(int i = 0; i < nPoints; i++)
  {
    double err = fabs(distPointToPlane(points[i].p, plane));
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
    bool horizontal, Plane3 &plane)
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
    if(selectSample(points, nPoints, up, horizontal, p, q, r))
    //if(selectSampleNear(points, nPoints, p, q, r))
    {
      Plane3 planeHyp;
      if(computeModel(p, q, r, planeHyp))
      {
        int nInliers = 0;
        double error = fitError(points, nPoints, planeHyp, nInliers);
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
    return true;
  }
  else
  {
    return false;
  }
}

}
