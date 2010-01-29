/**
 * $Id$
 *
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 *
 * TODO:
 * - now we assume undistorted images 
 * -> introduce distortion paramter
 */


#include "ODetect3D.hh"

namespace P 
{





/********************** ODetect3D ************************
 * Constructor/Destructor
 */
ODetect3D::ODetect3D()
 : dbg(0)
{
  cameraMatrix=0;
  distCoeffs=0;
}

ODetect3D::~ODetect3D()
{
  if (cameraMatrix!=0) cvReleaseMat(&cameraMatrix);
  if (distCoeffs!=0) cvReleaseMat(&distCoeffs);
}

/**
 * DeletePairs
 */
void ODetect3D::DeletePairs(Array<KeyClusterPair*> &matches)
{
  for (unsigned i=0; i<matches.Size(); i++)
    delete matches[i];
  matches.Clear();
}


/**
 * Match keypoints with codebook
 * check second nearest neighbour
 */
void ODetect3D::MatchKeypoints2(Array<KeypointDescriptor *> &keys, Array<CodebookEntry *> &cb, Array<KeyClusterPair* > &matches)
{
  float dist, dist1, dist2;
  unsigned idx;
  KeypointDescriptor *key;
  float minDist = Def::DO_MIN_DESC_DIST * Def::DO_MIN_DESC_DIST;

  for (unsigned i=0; i<keys.Size(); i++)
  {
    if (keys[i]->id != 0)
    {
      dist1 = dist2 = FLT_MAX;
      key = (KeypointDescriptor*)keys[i];
      for (unsigned j=0; j<cb.Size(); j++)
      {
        if(key->GetType() == cb[j]->model->GetType())
        {
          dist = key->DistSqr(key->GetVec(), cb[j]->model->GetVec(), key->GetSize());
          if (dist < dist1)
          {
            dist2=dist1;
            dist1=dist;
            idx=j;
          }
          else
          {
            if (dist<dist2) dist2=dist;
          }
        }
      }
      if (100.*dist1 < minDist*dist2)
        matches.PushBack(new KeyClusterPair(key, cb[idx]));
    }
  } 
}

/**
 * Match keypoints with codebook
 * (use threshold)
 */
void ODetect3D::MatchKeypoints(Array<KeypointDescriptor *> &keys, Array<CodebookEntry *> &cb, Array<KeyClusterPair* > &matches)
{
  float dist, dist1;
  unsigned idx;
  KeypointDescriptor *key;

  for (unsigned i=0; i<keys.Size(); i++)
  {
    if (keys[i]->id != 0)                //upppps HACK!!!!
    {
      dist1=FLT_MAX;
      key = (KeypointDescriptor*)keys[i];
      for (unsigned j=0; j<cb.Size(); j++)
      {
        dist = key->Match(cb[j]->model);
        if (dist < dist1)
        {
          dist1=dist;
          idx=j;
        }
      }
      if (!IsEqual(dist1,FLT_MAX))
        matches.PushBack(new KeyClusterPair(key, cb[idx], dist1));
    }
  } 
}

/**
 * GetRandIdx
 */
void ODetect3D::GetRandIdx(unsigned size, unsigned num, P::Array<unsigned> &idx)
{
  unsigned temp;
  idx.Clear();
  for (unsigned i=0; i<num; i++)
  {
    do{
      temp = rand()%size;
    }while(idx.Contains(temp));
    idx.PushBack(temp);
  }
}

/**
 * Count inlier
 */
void ODetect3D::GetInlier(Array<KeyClusterPair*> &matches, Pose &pose, int &inl)
{
  inl=0;
  Vector2 p;
  Vector p3d;
  CodebookEntry *cbe;
  KeypointDescriptor *k;
  
  for (unsigned i=0; i<matches.Size(); i++)
  {
    cbe = matches[i]->c;
    k = matches[i]->k;
    
    for (unsigned j=0; j<cbe->Size(); j++)
    {
      p3d = pose.R*cbe->occurrences[j]->pos + pose.t;
      ProjectPoint2Image(p3d(1),p3d(2),p3d(3), C, p.x, p.y);

      if (Distance(p,k->p) < Def::DO_RANSAC_INL_DIST)
      {
        inl++;
        break;
      }
    }
  }
}


/**
 * uses opencv posit to compute 3d rotation and translation
 */
void ODetect3D::GetPose(CvMat *modelPoints, CvMat *imgPoints, CvMat *R, CvMat *t)
{
  CvMat *rod = cvCreateMat(3,1,CV_32F);
  
  cvFindExtrinsicCameraParams2(modelPoints, imgPoints, cameraMatrix, distCoeffs, rod, t);
  cvRodrigues2(rod,R);

  cvReleaseMat(&rod);
}

/**
 * Verify object model
 */
void ODetect3D::FitModelRANSAC(Array<KeyClusterPair*> &matches, Pose &pose)
{
  if (matches.Size()<4)
    return;

  int k=0;
  double eps = 4./(double)matches.Size();
  int inl, inls = 0;
  Array<unsigned> idx, idx2;
  CvMat *modelPoints = cvCreateMat( 4, 3, CV_32F );
  CvMat *imgPoints = cvCreateMat( 4, 2, CV_32F );
  srand(time(NULL));
  KeyClusterPair* kp;
  CvMat *R = cvCreateMat( 3, 3, CV_32F );
  CvMat *t = cvCreateMat( 3, 1, CV_32F );
  Pose tempPose;

  while (pow(1. - pow(eps,4),k) >= Def::DO_RANSAC_ETA0 && k<Def::DO_MAX_RANSAC_TRIALS)
  {
    GetRandIdx(matches.Size(), 4, idx);

    for (unsigned i=0; i<4; i++)
    {
      kp = matches[idx[i]];
      Vector &pos = kp->c->occurrences[rand()%kp->c->Size()]->pos;
      cvmSet( modelPoints, i, 0, pos(1) );
      cvmSet( modelPoints, i, 1, pos(2) );
      cvmSet( modelPoints, i, 2, pos(3) );
      cvmSet( imgPoints, i, 0, kp->k->p.x);
      cvmSet( imgPoints, i, 1, kp->k->p.y);
    }        
    
    GetPose(modelPoints, imgPoints, R, t);  

    OpenCV2Pose(R,t,tempPose);
    GetInlier(matches, tempPose, inl);
    
    if (inl > inls)
    {
      inls = inl;
      eps = (double)inls / (double)matches.Size();
      OpenCV2Pose(R,t,pose);
    }

    k++;
  }

  #ifdef DEBUG
  cout<<"Num RANSAC triels: "<<k<<endl;
  #endif

  cvReleaseMat(&R);
  cvReleaseMat(&t);
  cvReleaseMat(&modelPoints);
  cvReleaseMat(&imgPoints);
}


/**
 * Get best supporting corner of a matching codebook entry
 */
bool ODetect3D::GetBestCorner(Pose &pose, KeypointDescriptor *k, CodebookEntry *cbe, Vector &pos, double &minDist)
{
  Vector2 p;
  Vector p3d;

  double dist;
  minDist = DBL_MAX;

  for (unsigned j=0; j<cbe->Size(); j++)
  {
    p3d = pose.R*cbe->occurrences[j]->pos + pose.t;
    ProjectPoint2Image(p3d(1),p3d(2),p3d(3), C, p.x, p.y);

    dist = Distance(p,k->p);
    if (dist < minDist)
    {
      minDist = dist;
      pos = cbe->occurrences[j]->pos;
    }
  }

  if (minDist < Def::DO_RANSAC_INL_DIST)
    return true;
  return false;
}

/**
 * Compute least squares pose
 */
void ODetect3D::RefinePoseLS(Array<KeyClusterPair*> &matches, Pose &pose, unsigned &inl, double &err)
{
  Vector pos;
  double dist;
  Array<Vector> modelPoints;
  Array<Vector2> imgPoints;
  CvMat *modelPointsCv; 
  CvMat *imgPointsCv; 
  CvMat *R = cvCreateMat( 3, 3, CV_32F );
  CvMat *t = cvCreateMat( 3, 1, CV_32F );
  err=0;

  for (unsigned i=0; i<matches.Size(); i++)
  {
    if (GetBestCorner(pose, matches[i]->k, matches[i]->c, pos, dist))
    {
      err+=dist;
      modelPoints.PushBack(pos);
      imgPoints.PushBack(matches[i]->k->p);
    }
  }

  inl = modelPoints.Size();
  err/=(double)modelPoints.Size();

  if (inl>4)
  {
    modelPointsCv = cvCreateMat( modelPoints.Size(), 3, CV_32F );
    imgPointsCv = cvCreateMat( imgPoints.Size(), 2, CV_32F );

    for (unsigned i=0; i<modelPoints.Size(); i++)
    {
      cvmSet( modelPointsCv, i, 0, modelPoints[i](1) );
      cvmSet( modelPointsCv, i, 1, modelPoints[i](2) );
      cvmSet( modelPointsCv, i, 2, modelPoints[i](3) );
      cvmSet( imgPointsCv, i, 0, imgPoints[i].x);
      cvmSet( imgPointsCv, i, 1, imgPoints[i].y);
    }

    GetPose(modelPointsCv, imgPointsCv, R, t);  
    OpenCV2Pose(R,t,pose);

    cvReleaseMat(&modelPointsCv);
    cvReleaseMat(&imgPointsCv); 
  }

  cvReleaseMat(&R);
  cvReleaseMat(&t);
}

/**
 * ComputeConfidence
 */
void ODetect3D::ComputeConfidence(Array<KeypointDescriptor *> &keys, unsigned &numInl, Object3D &object)
{
  Vector2 p;
  Vector p3d;
  Array<Vector2> cs;
  CodebookEntry *cbe;
  unsigned numKeys=0;

  for (unsigned i=0; i<object.codebook.Size(); i++)
  {
    cbe=object.codebook[i];
    for (unsigned j=0; j<cbe->occurrences.Size(); j++)
    {
      p3d = object.pose.R * cbe->occurrences[j]->pos + object.pose.t;
      ProjectPoint2Image(p3d(1),p3d(2),p3d(3), C, p.x, p.y);
      cs.PushBack(p);
    }
  }

  ChainHull2D(cs, object.contour.v);

  for (unsigned i=0; i<keys.Size(); i++)
    if (object.contour.Inside(keys[i]->p))
      numKeys++;

  if (numKeys>4 && numInl>=4)
    object.conf = (double)numInl / (double)numKeys;
  else
    object.conf = 0.;
}




/******************************** PUBLIC **************************/
/**
 * Simple RANSAC based 3d object detector
 */
bool ODetect3D::Detect(Array<KeypointDescriptor *> &keys, Object3D *object)
{
  #ifdef DEBUG
  struct timespec start1, end1, start2, end2, start3, end3;
  clock_gettime(CLOCK_REALTIME, &start1);
  #endif

  unsigned numInl=0;
  Array<KeyClusterPair*> matches;

  if (Def::DO_USE_SECOND_MATCH)              // match keypoints with codebook
    MatchKeypoints2(keys, object->codebook, matches);
  else
    MatchKeypoints(keys, object->codebook, matches);

  #ifdef DEBUG
  if (dbg!=0)
    for (unsigned i=0; i<matches.Size(); i++)
      KeypointDescriptor::Draw(dbg,*matches[i]->k,CV_RGB(255,0,0));
  clock_gettime(CLOCK_REALTIME, &start2);
  #endif

  FitModelRANSAC(matches, object->pose);    // ransac pose
  
  #ifdef DEBUG
  clock_gettime(CLOCK_REALTIME, &end2);
  clock_gettime(CLOCK_REALTIME, &start3);
  #endif

  RefinePoseLS(matches, object->pose, numInl, object->err);     // least squares pose

  ComputeConfidence(keys, numInl, *object);

  DeletePairs(matches);

  #ifdef DEBUG
  clock_gettime(CLOCK_REALTIME, &end3);
  clock_gettime(CLOCK_REALTIME, &end1);
  cout<<"id="<<object->id<<", conf="<<object->conf<<", err="<<object->err<<", numInl="<<numInl
      <<", cbSize="<<object->codebook.Size()<<endl;
  cout<<"Time for RANSAC [s]: "<<P::timespec_diff(&end2, &start2)<<endl;
  cout<<"Time for least squares pose [s]: "<<P::timespec_diff(&end3, &start3)<<endl;
  cout<<"Time for object detection (matching and RANSAC) [s]: "<<P::timespec_diff(&end1, &start1)<<endl;
  #endif

  if (numInl>=4)
    return true;

  object->conf=0.;
  object->err=DBL_MAX;

  return false;
}

/**
 
* Set camera intrinsic parameter
 * We assume undistort images!!!
 * Keypoints must be undistort!!!!
 * @param _C Camera intrinsic matrix of undistort images
 */
void ODetect3D::SetCameraParameter(Matrix &_C)
{
  C = _C;
 
  cameraMatrix = cvCreateMat( 3, 3, CV_32F );
  distCoeffs = cvCreateMat( 4, 1, CV_32F );

  cvmSet( cameraMatrix, 0, 0, C(1,1) );
  cvmSet( cameraMatrix, 0, 1, C(1,2) );
  cvmSet( cameraMatrix, 0, 2, C(1,3) );
  cvmSet( cameraMatrix, 1, 0, C(2,1) );
  cvmSet( cameraMatrix, 1, 1, C(2,2) );
  cvmSet( cameraMatrix, 1, 2, C(2,3) );
  cvmSet( cameraMatrix, 2, 0, C(3,1) );
  cvmSet( cameraMatrix, 2, 1, C(3,2) );
  cvmSet( cameraMatrix, 2, 2, C(3,3) );

  cvmSet( distCoeffs, 0, 0, 0. );
  cvmSet( distCoeffs, 1, 0, 0. );
  cvmSet( distCoeffs, 2, 0, 0. );
  cvmSet( distCoeffs, 3, 0, 0. );
}



/************************************ DEBUG METHODES *********************************/



}

