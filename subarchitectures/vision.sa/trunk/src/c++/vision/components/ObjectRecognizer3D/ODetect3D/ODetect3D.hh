/**
 * $Id$
 * Simple RANSAC based 3d object detector
 *
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_DOBJECT_3D_HH
#define P_DOBJECT_3D_HH

#include <opencv/cv.h>
#include "KeyClusterPair.hh"
#include "Definitions.hh"
#include "Geometry.hh"
#include "Math.hh"
#include "Object3D.hh"
#include "Pose.hh"

namespace P
{


class ODetect3D
{
private:
  IplImage *dbg;

  Matrix C;
  CvMat *cameraMatrix;
  CvMat *distCoeffs;

  void DeletePairs(Array<KeyClusterPair*> &matches);
  void MatchKeypoints2(Array<KeypointDescriptor *> &keys, Array<CodebookEntry *> &cb, 
                       Array<KeyClusterPair*> &matches);
  void MatchKeypoints(Array<KeypointDescriptor *> &keys, Array<CodebookEntry *> &cb, 
                      Array<KeyClusterPair* > &matches);
  void FitModelRANSAC(Array<KeyClusterPair*> &matches, Pose &pose);
  void GetRandIdx(unsigned size, unsigned num, P::Array<unsigned> &idx);
  void GetInlier(Array<KeyClusterPair*> &matches, Pose &pose, int &inl);
  bool GetBestCorner(Pose &pose, KeypointDescriptor *k, CodebookEntry *cbe, Vector &pos, double &minDist);
  void GetPose(CvMat *modelPoints, CvMat *imgPoints, CvMat *R, CvMat *t);
  void RefinePoseLS(Array<KeyClusterPair*> &matches, Pose &pose, unsigned &inl, double &err);
  void ComputeConfidence(Array<KeypointDescriptor *> &keys, unsigned &numInl, Object3D &object);




  inline void OpenCV2Pose(CvMat *R, CvMat *t, Pose &pose);
  inline void ProjectPoint2Image(double xc, double yc, double zc, Matrix &C, double &xi, double &yi);



public:
  ODetect3D();
  ~ODetect3D();

  bool Detect(Array<KeypointDescriptor *> &keys, Object3D *object);
  void SetCameraParameter(Matrix &_C);

  void SetDebugImage(IplImage *img){dbg = img;}
};

/*********************** INLINE METHODES **************************/
inline void ODetect3D::OpenCV2Pose(CvMat *R, CvMat *t, Pose &pose)
{
  pose.R(1,1)=cvmGet( R, 0, 0);
  pose.R(1,2)=cvmGet( R, 0, 1);
  pose.R(1,3)=cvmGet( R, 0, 2);
  pose.R(2,1)=cvmGet( R, 1, 0);
  pose.R(2,2)=cvmGet( R, 1, 1);
  pose.R(2,3)=cvmGet( R, 1, 2);
  pose.R(3,1)=cvmGet( R, 2, 0);
  pose.R(3,2)=cvmGet( R, 2, 1);
  pose.R(3,3)=cvmGet( R, 2, 2);

  pose.t(1,1)=cvmGet( t, 0, 0);
  pose.t(2,1)=cvmGet( t, 1, 0);
  pose.t(3,1)=cvmGet( t, 2, 0);
}

/**
 * ProjectPoint2Image
 * projects a 3d point to the image
 */
inline void ODetect3D::ProjectPoint2Image(double xc, double yc, double zc, Matrix &C, double &xi, double &yi)
{
  xi = C(1,1) * xc/zc + C(1,3);
  yi = C(2,2) * yc/zc + C(2,3);
}


}

#endif

