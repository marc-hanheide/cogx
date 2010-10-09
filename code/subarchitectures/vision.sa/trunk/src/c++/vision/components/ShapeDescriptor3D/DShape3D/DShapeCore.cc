/**
 * $Id$
 *
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 *
 */


#include "DShapeCore.hh"

//#define DEBUG


namespace P 
{

int DShapeCore::HALF_WIN_SIZE = 10;
int DShapeCore::SIZE_POINT_CONTAINER = 2000;
double DShapeCore::QUALITY = 0.01;  // TUNING KLT feature quality threshold, maybe use looser 0.005
double DShapeCore::MIN_DISTANCE = 5;//10;
int DShapeCore::USE_HARRIS = 0;
bool DShapeCore::REFINE_CORNER_SUBPIX = false;

double DShapeCore::MAX_AFF_TPL_ERROR = 200;
double DShapeCore::SIZE_SURF = 10;
double DShapeCore::MAX_DIST_SURF = .35;
double DShapeCore::MAX_DIST_SIFT = .35;

int DShapeCore::RAS_SIZE_ANGLE = 24; //72;//36;//24;
int DShapeCore::RAS_SIZE_SCALE = 1;





/********************** DShapeCore ************************
 * Constructor/Destructor
 */
DShapeCore::DShapeCore()
 : dbg1(0), dbg2(0), cameraMatrix1(0), cameraMatrix2(0), distCoeffs1(0), distCoeffs2(0), 
   camR12(0), camT12(0),
   imgRect1(0), imgRect2(0), rmask1(0), 
   eig(0), temp(0)
{
  points[0] = (CvPoint2D32f*)cvAlloc(SIZE_POINT_CONTAINER*sizeof(points[0][0]));
  points[1] = (CvPoint2D32f*)cvAlloc(SIZE_POINT_CONTAINER*sizeof(points[0][0]));
  status = (char*)cvAlloc(SIZE_POINT_CONTAINER);
  error = (float*)cvAlloc(SIZE_POINT_CONTAINER*sizeof(error[0]));
  matrices = (float*)cvAlloc(4*SIZE_POINT_CONTAINER*sizeof(matrices[0]));
}

DShapeCore::~DShapeCore()
{
  if (cameraMatrix1!=0) cvReleaseMat(&cameraMatrix1);
  if (cameraMatrix2!=0) cvReleaseMat(&cameraMatrix2);
  if (camR12!=0) cvReleaseMat(&camR12);
  if (camT12!=0) cvReleaseMat(&camT12);
  if (distCoeffs1!=0) cvReleaseMat(&distCoeffs1);
  if (distCoeffs2!=0) cvReleaseMat(&distCoeffs2);

  if (points[0]!=0) cvFree(&points[0]);
  if (points[1]!=0) cvFree(&points[1]);
  if (status!=0) cvFree(&status);
  if (error!=0) cvFree(&error);
  if (matrices!=0) cvFree(&matrices);

  ReleaseImages();

  DeleteKeypoints(keys1);
  DeleteKeypoints(keys2);
  DeletePlanes(planes);
}

/************************** PRIVATE ************************/

/** 
 * release images
 */
void DShapeCore::ReleaseImages()
{
  if (rmask1!=0) cvReleaseImage(&rmask1);
  if (imgRect1!=0) cvReleaseImage(&imgRect1);
  if (imgRect2!=0) cvReleaseImage(&imgRect2);
  if (eig!=0) cvReleaseImage(&eig);
  if (temp!=0) cvReleaseImage(&temp);

  rmask1 = imgRect1 = imgRect2 = 0;
  eig = temp = 0;
}

/**
 * Release images
 */
void DShapeCore::InitImages(IplImage* img1, IplImage *img2, IplImage *mask1)
{
  if (cameraMatrix1==0 || cameraMatrix2==0 || camR12==0 || camT12==0)
    throw Except (__HERE__,"Need camera parameter!");

  if (!IsImage8UC1(img1) || !IsImage8UC1(img2))
    throw Except (__HERE__,"Wrong image format (need 8UC1)!");
  
  if (!IsImageSizeEqual(img1, img2))
    throw Except (__HERE__,"Need equal size of stereo images!");

  if (mask1!=0)
    if(!IsImage8UC1(mask1) || !IsImageSizeEqual(img1, mask1))
      throw Except (__HERE__,"Wrong image format of mask!");

  if (imgRect1!=0 && !IsImageSizeEqual(img1, imgRect1))
    ReleaseImages();

  if (imgRect1==0)
  {
    if (mask1!=0) rmask1 = cvCreateImage ( cvGetSize ( img1 ), 8, 1 );
    imgRect1 = cvCreateImage ( cvGetSize ( img1 ), 8, 1 );
    imgRect2 = cvCreateImage ( cvGetSize ( img1 ), 8, 1 );
    eig = cvCreateImage ( cvGetSize ( img1 ), 32, 1 );
    temp = cvCreateImage ( cvGetSize ( img1 ), 32, 1 );

    imgSize = cvGetSize(img1);
  }

  cvCopy(img2, imgRect2);
  cvCopy(img1, imgRect1);

  if (mask1!=0)
  {
    cvCopy(mask1, rmask1);
    cvThreshold(rmask1,rmask1,128,255,0);
  }

  #ifdef DEBUG
  if (dbg1!=0) cvConvertImage(imgRect1, dbg1);
  if (dbg2!=0) cvConvertImage(imgRect2, dbg2);
  #endif
}

/**
 * Detect corners
 */
void DShapeCore::DetectKeypoints(IplImage *img, Array<KeypointDescriptor*> &ks, double minDistance, IplImage *mask)
{
  DeleteKeypoints(ks);

  int cnt = SIZE_POINT_CONTAINER;
  cvGoodFeaturesToTrack( img, eig, temp, points[0], &cnt, QUALITY, minDistance, mask, 3, USE_HARRIS, 0.04 );

  if (REFINE_CORNER_SUBPIX)
  {
    cvFindCornerSubPix(img, points[0], cnt, cvSize(HALF_WIN_SIZE,HALF_WIN_SIZE), cvSize(-1,-1),
                             cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
  }


  for (int i=0; i<cnt; i++)
  {
    ks.PushBack(new KeypointDescriptor(KeypointDescriptor::DOG_SIFT, points[0][i].x,points[0][i].y, 2, 0));
  }

}

/**
 * Refine the keypoint locaiton (affine model) and filter points
 */
void DShapeCore::RefineAffine(IplImage *prevGrey, IplImage *grey, Array<KeypointDescriptor*> &ks2, double maxError)
{
  int z=0;
  for (int i=0; i<(int)ks2.Size(); i++)
  {
    if (ks2[i]->bw!=0)
    {
      points[0][z].x = ks2[i]->bw->p.x;
      points[0][z].y = ks2[i]->bw->p.y;
      points[1][z].x = ks2[i]->p.x;
      points[1][z].y = ks2[i]->p.y;
      matrices[z*4 + 0] = ks2[i]->mi11;
      matrices[z*4 + 1] = ks2[i]->mi12;
      matrices[z*4 + 2] = ks2[i]->mi21;
      matrices[z*4 + 3] = ks2[i]->mi22;
      z++;
    }
  }

  int flags = 0;
  flags |= CV_LKFLOW_PYR_A_READY;
  flags |= CV_LKFLOW_PYR_B_READY;
  flags |= CV_LKFLOW_INITIAL_GUESSES;

  cvCalcAffineFlowPyrLK(prevGrey, grey, 0,0,
              points[0], points[1], matrices, z, cvSize(HALF_WIN_SIZE, HALF_WIN_SIZE), 0,
              status, error, cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), flags);

  Array<KeypointDescriptor*> temp;

  z=0;
  for (int i=0; i<(int)ks2.Size(); i++)
  {
    if (ks2[i]->bw!=0)
    {
      if (status[z]==1 && error[z] < maxError &&
          points[1][z].x>=0 && points[1][z].y>=0 && 
          points[1][z].x<grey->width && points[1][z].y<grey->height)
      {
        ks2[i]->p.x = points[1][z].x;
        ks2[i]->p.y = points[1][z].y;
        ks2[i]->mi11 = matrices[z*4 + 0];
        ks2[i]->mi12 = matrices[z*4 + 1];
        ks2[i]->mi21 = matrices[z*4 + 2];
        ks2[i]->mi22 = matrices[z*4 + 3];
        temp.PushBack(ks2[i]);
      }
      else
      {
        DeleteKeypoint(ks2[i]);
      }

      z++;
    }
    else
    {
      DeleteKeypoint(ks2[i]);
    }
  }

  ks2 = temp;
}


/**
 * Track corners to second image
 */
void DShapeCore::TrackKeypoints(IplImage *img1, IplImage *img2, Array<KeypointDescriptor*> &ks1, Array<KeypointDescriptor*> &ks2)
{
  DeleteKeypoints(ks2);

  KeypointDescriptor *k1, *k2;
  int winSize = HALF_WIN_SIZE*2+1;
  CvPoint    minloc, maxloc;
  double    minval, maxval;

  IplImage *res = cvCreateImage(cvSize(img2->width  - winSize  + 1, 1), IPL_DEPTH_32F, 1);

  for (unsigned i=0; i<ks1.Size(); i++)
  {
    if (ks1[i]->p.x-HALF_WIN_SIZE >= 0 && ks1[i]->p.y-HALF_WIN_SIZE >= 0 && 
        ks1[i]->p.x+HALF_WIN_SIZE < img1->width && ks1[i]->p.y+HALF_WIN_SIZE < img1->height)
    {
      k1 = ks1[i];
      cvSetImageROI(img1, cvRect(k1->p.x-HALF_WIN_SIZE, k1->p.y-HALF_WIN_SIZE,winSize,winSize));
      cvSetImageROI(img2, cvRect(0, k1->p.y-HALF_WIN_SIZE,img2->width,winSize));

      cvMatchTemplate(img2, img1, res, CV_TM_SQDIFF);

      cvMinMaxLoc(res, &minval, &maxval, &minloc, &maxloc, 0);

      k2 = new KeypointDescriptor(KeypointDescriptor::DOG_SIFT, minloc.x+HALF_WIN_SIZE, k1->p.y, 2, 0);
      k2->bw = k1;
      k1->fw = k2;
      ks2.PushBack(k2);    
    }
  }

  cvResetImageROI(img1);
  cvResetImageROI(img2);
  cvReleaseImage(&res);

  RefineAffine(img1, img2, ks2, MAX_AFF_TPL_ERROR);
}

/**
 * Track corners to second image
 */
void DShapeCore::TrackKeypoints2(IplImage *img1, IplImage *img2, Array<KeypointDescriptor*> &ks1, Array<KeypointDescriptor*> &ks2)
{
  KeypointDescriptor *k1, *k2;
  DetectKeypoints(img2, ks2, 0.);

  IplImage *res = cvCreateImage(cvSize(1, 1), IPL_DEPTH_32F, 1);
  float *d = (float*)res->imageData;
  float dmin;
  unsigned idx;
  double y1, y2;
  int winSize = HALF_WIN_SIZE*2+1;

  for (unsigned i=0; i<ks1.Size(); i++)
  {
    if (ks1[i]->p.x-HALF_WIN_SIZE >= 0 && ks1[i]->p.y-HALF_WIN_SIZE >= 0 && 
        ks1[i]->p.x+HALF_WIN_SIZE < img1->width && ks1[i]->p.y+HALF_WIN_SIZE < img1->height)
    {
      k1 = ks1[i];
      y1 = ((int)(k1->p.y+.5))-1;
      y2 = ((int)(k1->p.y+.5))+1;
      dmin = FLT_MAX;
      for (unsigned j=0; j<ks2.Size(); j++)
      {
        if (ks2[j]->p.x-HALF_WIN_SIZE >= 0 && ks2[j]->p.y-HALF_WIN_SIZE >= 0 && 
            ks2[j]->p.x+HALF_WIN_SIZE < img2->width && ks2[j]->p.y+HALF_WIN_SIZE < img2->height)
        {
          k2 = ks2[j];
          if (k2->p.y >= y1 && k2->p.y <= y2)
          {
            cvSetImageROI(img1, cvRect(k1->p.x-HALF_WIN_SIZE, k1->p.y-HALF_WIN_SIZE,winSize,winSize));
            cvSetImageROI(img2, cvRect(k2->p.x-HALF_WIN_SIZE, k2->p.y-HALF_WIN_SIZE,winSize,winSize));

            cvMatchTemplate(img2, img1, res, CV_TM_SQDIFF);

            if (*d < dmin)
            {
              dmin = *d;
              idx = j;
            }
          }
        }
      }
      if (dmin != FLT_MAX)
      {
        k1->fw = ks2[idx];
        ks2[idx]->bw = k1;
      }
    }
  }

  cvResetImageROI(img1);
  cvResetImageROI(img2);
  cvReleaseImage(&res); 

  RefineAffine(img1, img2, ks2, MAX_AFF_TPL_ERROR);
}

/**
 * compute Surf descriptor to give points
 */
void DShapeCore::ComputeSurfDescriptor(IplImage *img, Array<KeypointDescriptor*> &ks)
{
  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq *keysSurf = 0, *descSurf = 0;
  CvSURFParams params = cvSURFParams(500, 1);
  keysSurf = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvSURFPoint), storage );

  
  for (unsigned i=0; i<ks.Size(); i++)
  {
    CvSURFPoint point = cvSURFPoint( cvPoint2D32f(ks[i]->p.x,ks[i]->p.y), 0, SIZE_SURF, 0, 1000 );  
    cvSeqPush( keysSurf, &point );
  }

  cvExtractSURF( img, 0, &keysSurf, &descSurf, storage, params, 1);

  CvSeqReader reader;
  cvStartReadSeq( descSurf, &reader );

  int length = (int)(descSurf->elem_size/sizeof(float));

  for( int i = 0; i < descSurf->total; i++ )
  {
      float* descriptor = (float*)reader.ptr;
      CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
      ks[i]->AllocVec(length);
      CopyVec(descriptor, ks[i]->vec, length);
  }
}


/**
 * Track corners to second image
 */
void DShapeCore::TrackKeypoints3(IplImage *img1, IplImage *img2, Array<KeypointDescriptor*> &ks1, Array<KeypointDescriptor*> &ks2)
{
  KeypointDescriptor *k1, *k2;
  DetectKeypoints(img2, ks2, 0.);
  ComputeSurfDescriptor(img1, ks1);
  ComputeSurfDescriptor(img2, ks2);

  double dmin, d;
  unsigned idx;
  double y1, y2;

  for (unsigned i=0; i<ks1.Size(); i++)
  {
    if (ks1[i]->size>0 &&
        ks1[i]->p.x-HALF_WIN_SIZE >= 0 && ks1[i]->p.y-HALF_WIN_SIZE >= 0 && 
        ks1[i]->p.x+HALF_WIN_SIZE < img1->width && ks1[i]->p.y+HALF_WIN_SIZE < img1->height)
    {
      k1 = ks1[i];
      y1 = ((int)(k1->p.y+.5))-1;
      y2 = ((int)(k1->p.y+.5))+1;
      dmin = FLT_MAX;
      for (unsigned j=0; j<ks2.Size(); j++)
      {
        if (ks2[j]->size > 0 &&
            ks2[j]->p.x-HALF_WIN_SIZE >= 0 && ks2[j]->p.y-HALF_WIN_SIZE >= 0 && 
            ks2[j]->p.x+HALF_WIN_SIZE < img2->width && ks2[j]->p.y+HALF_WIN_SIZE < img2->height)
        {
          k2 = ks2[j];
          if (k2->p.y >= y1 && k2->p.y <= y2)
          {
            d = CompareSURFDescriptors( ks2[j]->vec, ks1[i]->vec, dmin, ks2[j]->size ); 
            if (d < dmin)
            {
              dmin = d;
              idx = j;
            }
          }
        }
      }
      if (dmin < MAX_DIST_SURF)
      {
        k1->fw = ks2[idx];
        ks2[idx]->bw = k1;
      }
    }
  }

  RefineAffine(img1, img2, ks2, 100000);
}

/**
 * Track corners to second image
 */
/*void DShapeCore::TrackKeypoints4(IplImage *img1, IplImage *img2, Array<KeypointDescriptor*> &ks1, Array<KeypointDescriptor*> &ks2)
{
  KeypointDescriptor *k1, *k2;
  DetectKeypoints(img2, ks2, 0.);
  sift.SetDescriptor(img1, ks1);
  sift.SetDescriptor(img2, ks2);

  double dmin, d;
  unsigned idx;
  double y1, y2;

  for (unsigned i=0; i<ks1.Size(); i++)
  {
    if (ks1[i]->size>0 &&
        ks1[i]->p.x-HALF_WIN_SIZE >= 0 && ks1[i]->p.y-HALF_WIN_SIZE >= 0 && 
        ks1[i]->p.x+HALF_WIN_SIZE < img1->width && ks1[i]->p.y+HALF_WIN_SIZE < img1->height)
    {
      k1 = ks1[i];
      y1 = ((int)(k1->p.y+.5))-1;
      y2 = ((int)(k1->p.y+.5))+1;
      dmin = FLT_MAX;
      for (unsigned j=0; j<ks2.Size(); j++)
      {
        if (ks2[j]->size > 0 &&
            ks2[j]->p.x-HALF_WIN_SIZE >= 0 && ks2[j]->p.y-HALF_WIN_SIZE >= 0 && 
            ks2[j]->p.x+HALF_WIN_SIZE < img2->width && ks2[j]->p.y+HALF_WIN_SIZE < img2->height)
        {
          k2 = ks2[j];
          if (k2->p.y >= y1 && k2->p.y <= y2)
          {
            d = CompareSURFDescriptors( ks2[j]->vec, ks1[i]->vec, dmin, ks2[j]->size ); 
            if (d < dmin)
            {
              dmin = d;
              idx = j;
            }
          }
        }
      }
      if (dmin < MAX_DIST_SIFT)
      {
        k1->fw = ks2[idx];
        ks2[idx]->bw = k1;
      }
    }
  }

  RefineAffine(img1, img2, ks2, 100000);
}*/

/**
 * Set 3d points to matched keys using the difference of the x-coordinates (disparity)
 */
void DShapeCore::ReconstructPoints(Array<KeypointDescriptor*> &ks)
{
  double p3[4];

  for (unsigned i=0; i<ks.Size(); i++)
  {
    if (ks[i]->fw!=0)
    {
      double d = ks[i]->p.x - ks[i]->fw->p.x;
      assert(fpclassify(d) != FP_ZERO);
      double tx = cvmGet(camT12, 0, 0);
      p3[0] = ks[i]->p.x - cvmGet(cameraMatrix1, 0, 2);
      p3[1] = ks[i]->p.y - cvmGet(cameraMatrix1, 1, 2);
      p3[2] = cvmGet(cameraMatrix1, 0, 0);
      p3[3] = -d/tx;
      // SVS calibration uses mm, we want m -> divide by 1000
      p3[3] *= 1000.;
      p3[0] /= p3[3];
      p3[1] /= p3[3];
      p3[2] /= p3[3];

      ks[i]->Set3D(p3);
    }
  }
}

/**
 * Fit least squares planes
 */
void DShapeCore::FitPlanes3D(Array<Plane*> &planes)
{
  Vector3 nz = Vector3(0.,0.,1.);
  Plane *plane;
  SPlane3D lsp;
  Array<Vector3> points;
  KeypointDescriptor *key;
  Hull2D chull;
  Array<int> hull;
  int hnum;

  for (unsigned i=0; i<planes.Size(); i++)
  {
    lsp.Clear();
    plane = planes[i];

    for (unsigned j=0; j<plane->keys.Size(); j++)
      if (plane->keys[j]->bw!=0)
      {
        key = (KeypointDescriptor*)plane->keys[j]->bw;
        if (key->Have3D())
          lsp.Insert(key->pos->data.db);
      }
    
    if (lsp.Size()>3)
    {
      //get plane normal and project 3d points to plane
      lsp.GetMean(&plane->p.x);
      lsp.Insert(&plane->p.x);

      points.Resize(lsp.Size());
      lsp.ProjectPointsToPlane3D(&plane->n.x, &points[0].x);
      plane->p = points.Last();   //center point
      if (Dot(nz, plane->n) > 0.) 
        plane->n = -plane->n;
      

      //get convex hull
      chull.Clear();
      for (unsigned j=0; j<plane->keys.Size(); j++)
        if (plane->keys[j]->bw!=0)
        {
          key = (KeypointDescriptor*)plane->keys[j]->bw;
          if (key->Have3D())
            chull.Insert(&key->p.x);
        }

      hull.Resize(chull.Size());
      chull.ConvexHull(&hull[0],hnum);

      plane->contour.Clear();
      for (int j=0; j<hnum; j++)
        plane->contour.PushBack(points[hull[j]]);
      
      //get area of convex hull
      SPolygon3D::ComputeArea(plane->contour, plane->area);      
      plane->oid = 0;
    }
  }
}

/**
 * compute relative angle shape descriptor
 */
void DShapeCore::ComputeRAShapeDescriptor(Array<Plane*> &planes, RASDescriptor &ras,
  int sizeAngle, int sizeScale)
{
  //test angles
  double relAngle, relScale;
  Vector3 l1, l2;

  ras.Set(sizeAngle, sizeScale);
  ras.Clear();  

  for (unsigned i=0; i<planes.Size(); i++)
  {
    Plane *p1 = planes[i];
    #ifdef DEBUG
    cout<<p1->id<<" area="<<p1->area*100.*100.<<"cm²: ";
    #endif
    for (unsigned j=0; j<planes.Size(); j++)
    {
      if (i!=j)
      {
        relAngle = ScaleAngle_0_pi(acos(Dot(p1->n,planes[j]->n)));

        /* for now forget about convex/concave distinction.
           first we need to be able to reliably identify where is inside
           and outside
        l1 = p1->p - planes[j]->p;
        l2 = (p1->p+p1->n) - (planes[j]->p+planes[j]->n);
        if (Length(l1) > Length(l2)) relAngle+=pi;*/

        relScale = p1->area/planes[j]->area;
        ras.Insert(relAngle, relScale);
      }
      #ifdef DEBUG
      else relAngle=relScale=0.;
      cout<<relAngle*180./M_PI<<" ";
      #endif
    }
    #ifdef DEBUG
    cout<<endl;
    #endif
  }

  ras.Normalise();
}




/*************************** PUBLIC **************************/

/**
 * Operate
 * @param img1 Left image
 * @param img2 Right image
 * @param ras Resulting histogram angles between planes
 * @param mask1 mask of the object to reconstruct (img1, 0..background, 255..foreground)
 */
void DShapeCore::Operate(IplImage* img1, IplImage *img2, RASDescriptor &ras,
  int sizeAngle, int sizeScale, IplImage *mask1)
{
  DeletePlanes(planes);
  InitImages(img1, img2, mask1);

  DetectKeypoints(imgRect1, keys1, MIN_DISTANCE, rmask1);
  TrackKeypoints(imgRect1, imgRect2, keys1, keys2);    //scan lines (templates)
  //TrackKeypoints2(imgRect1, imgRect2, keys1, keys2); //match points (templates)
  //TrackKeypoints3(imgRect1, imgRect2, keys1, keys2); //match points (surf)
  //TrackKeypoints4(imgRect1, imgRect2, keys1, keys2); //match points (sift)
  ReconstructPoints(keys1);

  splane.DetectPlanes(keys2, planes);
  FitPlanes3D(planes);
  ComputeRAShapeDescriptor(planes, ras, sizeAngle, sizeScale);


  #ifdef DEBUG
  if (dbg1!=0) Draw(dbg1, keys1, 0);
  if (dbg2!=0) Draw(dbg2, keys2, 0);
  if (dbg2!=0) splane.DrawPlanes(dbg2,planes);
  cout<<"Number of points: "<<keys1.Size()<<"/"<<keys2.Size()<<endl;
  #endif
}




/**
 * Set camera parameter (format see OpenCV)
 * @param C1 Intrinsic camera parameter of left camera
 * @param C2 Intrinsic camera parameter of right camera
 * @param D1 Distortion parameter of left camera
 * @param D2 Distortion parameter of right camera
 * @param R,T Baseline of the camera (extrinsic parameter)
 */
void DShapeCore::SetCameraParameter(CvMat *C1, CvMat *C2, CvMat *D1, CvMat *D2, CvMat *R, CvMat *T)
{
  if (cameraMatrix1==0) cameraMatrix1 = cvCreateMat( 3, 3, CV_64F );
  if (cameraMatrix2==0) cameraMatrix2 = cvCreateMat( 3, 3, CV_64F );
  if (distCoeffs1==0) distCoeffs1 = cvCreateMat( 5,1, CV_64F );
  if (distCoeffs2==0) distCoeffs2 = cvCreateMat( 5,1, CV_64F );
  if (camR12==0) camR12 = cvCreateMat( 3, 3, CV_64F );
  if (camT12==0) camT12 = cvCreateMat( 3, 1, CV_64F );

  cvConvert(C1, cameraMatrix1);
  cvConvert(C2, cameraMatrix2);
  cvConvert(R, camR12);
  cvConvert(T, camT12);
  cvConvert(D1, distCoeffs1);
  cvConvert(D2, distCoeffs2);
}

/**
 * Set debug image
 */
void DShapeCore::SetDebugImage(IplImage *img1, IplImage *img2)
{
  dbg1 = img1;
  dbg2 = img2;
}

/***
 * Draw tracks
 */
void DShapeCore::Draw(IplImage *img, Array<KeypointDescriptor*> &keys, unsigned detail)
{
  Keypoint *k;

  for (unsigned i=0; i<keys.Size(); i++)
  {
    SDraw::DrawCross(img,keys[i]->p.x,keys[i]->p.y,CV_RGB(255,255,255),1);

    if (detail>0 && keys[i]->bw!=0)
    {
      k = keys[i]->bw;
      SDraw::DrawLine(img,keys[i]->X(),keys[i]->Y(),k->X(),k->Y(), CV_RGB(255,255,255),1);
      if (detail>1)
      {
        while(k->bw!=0)
        {
          SDraw::DrawLine(img,k->X(),k->Y(),k->bw->X(),k->bw->Y(), CV_RGB(255,255,255),1);
          k = k->bw;
        }
      }
    }
  }
}



/****************************************************************************
 * Copy current scene information for drawing
 * Just trace back each object a pre-defined number of keyframes for drawing
 */
void DShapeCore::GetScene(Scene3D &scene)
{
  double *dat;
  Plane *plane;
  KeypointDescriptor *key;

  scene.cs.Clear();
  scene.ids.Clear();
  scene.contours.Clear();

  for (unsigned i=0; i<planes.Size(); i++)
  {
    plane = planes[i];
    scene.ids.PushBack(plane->id);
    scene.cs.PushBack(Array<Vector3>());
    scene.contours.PushBack(plane->contour);
    for (unsigned j=0; j<plane->keys.Size(); j++)
    {
      if (plane->keys[j]->bw!=0)
      {
        key = (KeypointDescriptor*)plane->keys[j]->bw;
        if (key->Have3D())
        {
          dat = key->pos->data.db;
          scene.cs.Last().PushBack(Vector3(dat[0],dat[1],dat[2]));
        }
      }
    }
  }

}


}

