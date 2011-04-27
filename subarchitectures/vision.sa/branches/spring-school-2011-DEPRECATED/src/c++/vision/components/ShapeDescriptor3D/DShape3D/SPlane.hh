/**
 * $Id$
 *
 * Model selection based plane detection
 *
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 *
 */

#ifndef P_SPLANE_HH
#define P_SPLANE_HH

#include <opencv/cv.h>
//#include "Geometry.hh"
#include "homest.h"
#include "KeypointDescriptor.hh"
#include "KPTree.hh"
#include "Plane.hh"
//#include "SPolygon.hh"
//#include "GPolygon.hh"
#include "DistIdx.hh"
#include "Homography.hh"
#include "SMatrix.hh"

namespace P
{


class SPlane
{
private:
  IplImage *dbg;
  KPTree *dt;
  
  Homography hg;

  static double CONST_ERR1, CONST_ERR2;

  void GetRandIdx(unsigned size, unsigned num, P::Array<unsigned> &idx);
  void GetNapIdx(P::Array<KeypointDescriptor*> &kps, unsigned num, P::Array<unsigned> &idx);
  void GetInlier(Array<KeypointDescriptor*> &kps, CvMat *H, int &inl);
  bool ComputeHomography(P::Array<KeypointDescriptor*> &kps, CvMat *H, double inl_pcent, bool nonl=true, bool outl=true);
  void AccumulateKeys(Array<KeypointDescriptor*> &kps, Plane &plane);
  void AccumulateKeys(Array<KeypointDescriptor*> &kps, Plane &plane, double thrDist);
  void GetRandHypotheses(P::Array<KeypointDescriptor*> &all, P::Array<KeypointDescriptor*> &kps, P::Array<Plane*> &tentPlanes);
  void CreateSegmentationMatrixQ(P::Array<Plane*> &tempPlanes, CvMat *Q);
  void SelectHypotheses(Array<Plane*> &randPlanes, P::Array<Plane*> &planes);
  void ComputeQii(Plane* kg, double &weight);
  void ComputeQij(Plane* ki, Plane* kj, double &weight);
  void GreedyQBP(CvMat *Q, CvMat *m);
  void ReleaseInteractions(Array<Plane*> &planes);
  void SplitPlanes(Array<Plane*> &planes);
  void SplitPlanes2(Array<Plane*> &planes);
  void GetSigma(P::Array<KDVertex*> &kdvs, double &mean, double &sigma);
  KDVertex* GetNextNode(Array<KDVertex*> &kdvs);
  void RenewKps(Array<Plane*> &planes, Array<KeypointDescriptor*> &ks, Array<KeypointDescriptor*> &kps);
  void CountExplainedKps(Array<KeypointDescriptor*> &kps, Array<Plane*> &planes, int &numEx);
  bool NearPoint(Array<KeypointDescriptor*> &kps, Keypoint* k, double &thr);
  void GetErrProbEuc(double m1[2],double m2[2],double H[9], double err[1]);
  void GetMatches(Array<KeypointDescriptor*> &keys, Array<KeypointDescriptor*> &matches);
  void AccumulateKeys(Array<KeypointDescriptor*> &kps, Array<Plane*> &planes);




public:

  Array<KeypointDescriptor*> matches, ks;

  SPlane();
  ~SPlane();
  void DetectPlanes(Array<KeypointDescriptor*> &keys, Array<Plane*> &planes);
  void TrackPlanes(Array<Plane*> &in,  Array<Plane*> &out);

  void SetDebugImage(IplImage *img){dbg = img;}

  void DrawPlane(IplImage *img, Plane &plane, CvScalar col=CV_RGB(0,255,0));
  void DrawPlanes(IplImage *img, Array<Plane*> &planes);

 


};

/*********************** INLINE METHODES **************************/



}

#endif

