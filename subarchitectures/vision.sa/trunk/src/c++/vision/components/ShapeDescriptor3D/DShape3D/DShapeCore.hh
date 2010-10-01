/**
 * $Id$
 *
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 *
 */

#ifndef P_DSHAPE3D_HH
#define P_DSHAPE3D_HH

#include "PNamespace.hh"
#include "Array.hh"
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "KeypointDescriptor.hh"
#include "SPlane.hh"
#include "Plane.hh"
#include "Homography.hh"
#include "Scene3D.hh"
#include "SPlane3D.hh"
#include "Hull2D.hh"
#include "SPolygon3D.hh"
#include "RASDescriptor.hh"
#include "ImgUtils.hh"

namespace P
{

/**
 * Reconstruct a sparse point cloud given two calibrated images (intrinsic and extrinsic), 
 * detect 3d planes and compute a histogram of the angles between the planes
 */
class DShapeCore
{
private:
  static int HALF_WIN_SIZE;
  static int SIZE_POINT_CONTAINER;
  static double QUALITY;
  static double MIN_DISTANCE;
  static int USE_HARRIS;
  static bool REFINE_CORNER_SUBPIX;
  static double MAX_AFF_TPL_ERROR;
  static double SIZE_SURF;
  static double MAX_DIST_SURF;
  static double MAX_DIST_SIFT;
  static int RAS_SIZE_ANGLE;
  static int RAS_SIZE_SCALE;



  IplImage *dbg1, *dbg2;
  CvMat *cameraMatrix1, *cameraMatrix2;
  CvMat *distCoeffs1, *distCoeffs2;
  CvMat *camR12, *camT12;

  IplImage *imgRect1, *imgRect2, *rmask1;

  IplImage *eig, *temp;

  char* status;
  CvPoint2D32f* points[2];
  float *matrices;
  float *error;

  CvSize imgSize;

  SPlane splane;

  Array<Plane*> planes;
  Array<KeypointDescriptor *> keys1, keys2;

  void ReleaseImages();
  void InitImages(IplImage* img1, IplImage *img2, IplImage *mask1);
  void DetectKeypoints(IplImage *img, Array<KeypointDescriptor*> &ks, double minDistance, IplImage *mask=0);
  void TrackKeypoints(IplImage *img1, IplImage *img2, Array<KeypointDescriptor*> &ks1, Array<KeypointDescriptor*> &ks2);
  void TrackKeypoints2(IplImage *img1, IplImage *img2, Array<KeypointDescriptor*> &ks1, Array<KeypointDescriptor*> &ks2);
  void TrackKeypoints3(IplImage *img1, IplImage *img2, Array<KeypointDescriptor*> &ks1, Array<KeypointDescriptor*> &ks2);
  void TrackKeypoints4(IplImage *img1, IplImage *img2, Array<KeypointDescriptor*> &ks1, Array<KeypointDescriptor*> &ks2);
  void ComputeSurfDescriptor(IplImage *img, Array<KeypointDescriptor*> &ks);
  void RefineAffine(IplImage *prevGrey, IplImage *grey, Array<KeypointDescriptor*> &ks2, double maxError=100000);

  void ReconstructPoints(Array<KeypointDescriptor*> &ks);
  void FitPlanes3D(Array<Plane*> &planes);

  inline double CompareSURFDescriptors( const float* d1, const float* d2, double best, int length );

public:
  DShapeCore();
  ~DShapeCore();

  void SetCameraParameter(CvMat *C1, CvMat *C2, CvMat *D1, CvMat *D2, CvMat *R, CvMat *T);
  void Operate(IplImage* img1, IplImage* img2, RASDescriptor &ras, int sizeAngle, int sizeScale, IplImage* mask1=0);
  void ComputeRAShapeDescriptor(Array<Plane*> &planes, RASDescriptor &ras, int sizeAngle, int sizeScale);

  void GetScene(Scene3D &scene);

  void Draw(IplImage *img, Array<KeypointDescriptor*> &keys, unsigned detail);



  void SetDebugImage(IplImage *img1, IplImage *img2);
};


/*********************** INLINE METHODES **************************/


inline double DShapeCore::CompareSURFDescriptors( const float* d1, const float* d2, double best, int length )
{
    double total_cost = 0;
    assert( length % 4 == 0 );
    for( int i = 0; i < length; i += 4 )
    {
        double t0 = d1[i] - d2[i];
        double t1 = d1[i+1] - d2[i+1];
        double t2 = d1[i+2] - d2[i+2];
        double t3 = d1[i+3] - d2[i+3];
        total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
        if( total_cost > best )
            break;
    }
    return total_cost;
}


}

#endif

