/**
 * $Id$
 */

#define SP3D_MIN_FEATURES 3
#define SP3D_RANSAC_ETA0 0.01
#define SP3D_PLANE_INL_DIST 0.01   //todo

#include "SPlane3D.hh"

namespace P 
{



SPlane3D::SPlane3D()
{
}

SPlane3D::~SPlane3D()
{
}


/*************************** PRIVATE *********************************/

/**
 * Compute Least Squares plane
 */
void SPlane3D::FitPlaneLS(vector<cv::Vec4d> &p3d, double n[3], double p[3])
{
  if (p3d.size()<4)
    throw runtime_error ("SPlane3D::FitPlaneLS : Too few points!");

  CvMat matA = cvMat(p3d.size(),4,CV_64F,&p3d[0][0]);
  CvMat *matW = cvCreateMat(p3d.size(), 4, CV_64F);
  CvMat *matU = cvCreateMat(p3d.size(), 4, CV_64F);
  CvMat *matV = cvCreateMat(4, 4, CV_64F);

  cvSVD(&matA,matW,matU,matV);

  n[0] = cvmGet(matV,0,3);
  n[1] = cvmGet(matV,1,3);
  n[2] = cvmGet(matV,2,3);

  double d = cvmGet(matV,3,3)/PVec::Norm3(n);
  PVec::Normalise3(n,n);

  double p1[3], p2[3];
  PVec::Mul3(n,-d,p1);

  PVec::Sub3(&p3d[0][0],p1,p2);
  PVec::Mul3(n,PVec::Dot3(p2,n),p2);
  PVec::Sub3(&p3d[0][0],p2, p);

  cvReleaseMat(&matW);
  cvReleaseMat(&matU);
  cvReleaseMat(&matV);
}

/**
 * Compute Least Squares plane and return points projected to the plane
 */
void SPlane3D::ProjectPointsToPlane3D(vector<cv::Vec4d> &p3d, double n[3], double *points)
{
  if (p3d.size()<4)
    throw runtime_error ("SPlane3D::ProjectPointsToPlane3D : Too few points!");

  CvMat matA = cvMat(p3d.size(),4,CV_64F,&p3d[0][0]);
  CvMat *matW = cvCreateMat(p3d.size(), 4, CV_64F);
  CvMat *matU = cvCreateMat(p3d.size(), 4, CV_64F);
  CvMat *matV = cvCreateMat(4, 4, CV_64F);

  cvSVD(&matA,matW,matU,matV);

  n[0] = cvmGet(matV,0,3);
  n[1] = cvmGet(matV,1,3);
  n[2] = cvmGet(matV,2,3);

  double d = cvmGet(matV,3,3)/PVec::Norm3(n);
  PVec::Normalise3(n,n);

  double p1[3], p2[3];
  PVec::Mul3(n,-d,p1);

  for (unsigned i=0; i<p3d.size(); i++)
  {
    PVec::Sub3(&p3d[i][0],p1,p2);
    PVec::Mul3(n,PVec::Dot3(p2,n),p2);
    PVec::Sub3(&p3d[i][0],p2, &points[i*3]);
  }

  cvReleaseMat(&matW);
  cvReleaseMat(&matU);
  cvReleaseMat(&matV);
}

/**
 * GetRandIdx
 */
void SPlane3D::GetRandIdx(unsigned size, unsigned num, P::vector<unsigned> &idx)
{
  unsigned temp;
  idx.clear();
  for (unsigned i=0; i<num; i++)
  {
    do{
      temp = rand()%size;
    }while(Contains(idx,temp));
    idx.push_back(temp);
  }
}

/**
 * Get the plane inlier
 */
void SPlane3D::GetInlierPlane3D(vector<cv::Vec4d> &p3d, cv::Vec4d &p1, cv::Vec4d &p2, cv::Vec4d &p3, int &inl)
{
  double a, b, c, d;
  double dist;

  Plane3dExp2Imp(&p1[0], &p2[0], &p3[0], a,b,c,d);

  inl=0;
  for (unsigned i=0; i<p3d.size(); i++)
  {
    Plane3dImpPointDist(a, b, c, d, &p3d[i][0], dist);

    if (dist < SP3D_PLANE_INL_DIST)
      inl++;
  }
}

/**
 * RANSAC a 3d plane
 */
bool SPlane3D::FitPlaneRANSAC(vector<cv::Vec4d> &p3d, cv::Vec4d &p1, cv::Vec4d &p2, cv::Vec4d &p3)
{
  if (p3d.size() < SP3D_MIN_FEATURES)   //3)
    return false;

  int k=0;
  double eps = 3./(double)p3d.size();
  int inl, inls = 0;
  vector<unsigned> idx, idx2;
  srand(time(NULL));

  while (pow(1. - pow(eps,3),k) >= SP3D_RANSAC_ETA0)
  {
    GetRandIdx(p3d.size(), 3, idx);

    GetInlierPlane3D(p3d, p3d[idx[0]], p3d[idx[1]], p3d[idx[2]], inl);

    if (inl > inls)
    {
      inls = inl;
      eps = (double)inls / (double)p3d.size();
      idx2 = idx;
    }

    k++;
  }

  p1 = p3d[idx2[0]];
  p2 = p3d[idx2[1]];
  p3 = p3d[idx2[2]];

  #ifdef DEBUG
  cout<<"numTrielsPlane3D="<<k<<", inls="<<inls<<endl;
  #endif
  return true;
}

/**
 * Least squares fit of an 3d plane
 */
bool SPlane3D::RefinePlane3dLS(vector<cv::Vec4d> &p3d, cv::Vec4d &p1, cv::Vec4d &p2,  cv::Vec4d &p3,
                               vector<cv::Point3d> &inPlane, double n[3], double p[3])
{
  if (p3d.size() < SP3D_MIN_FEATURES) //3)
    return false;

  double a, b, c, d;
  double dist;
  vector<cv::Vec4d> inl;

  Plane3dExp2Imp(&p1[0], &p2[0], &p3[0], a,b,c,d);

  inl.clear();
  outl.clear();
  for (unsigned i=0; i<p3d.size(); i++)
  {
    Plane3dImpPointDist(a, b, c, d, &p3d[i][0], dist);

    if (dist < SP3D_PLANE_INL_DIST)
    {
      inl.push_back(p3d[i]);
      outl.push_back(0);
    }
    else outl.push_back(1);
  }

  if (inl.size() > SP3D_MIN_FEATURES)
  {
    inPlane.resize(inl.size());
    ProjectPointsToPlane3D(inl, n, &inPlane[0].x);
    //FitPlaneLS(inl, n, p);
    return true;
  }
  return false;
}






/**************************** PUBLIC **********************************/
/**
 * compute mean vector
 */
void SPlane3D::GetMean(double mean[3])
{
  if (points4.size()==0)
    throw runtime_error("SPlane3D::GetMean : Empty data structure!");

  mean[0]=0.;
  mean[1]=0.;
  mean[2]=0.;

  for (unsigned i=0; i<points4.size(); i++)
  {
    mean[0] += points4[i][0];
    mean[1] += points4[i][1];
    mean[2] += points4[i][2];
  }

  mean[0] /= ((double)points4.size());
  mean[1] /= ((double)points4.size());
  mean[2] /= ((double)points4.size());
}

/**
 * compute median vector
 */
void SPlane3D::GetMedian(double median[3])
{
  if (points4.size()<3)
    throw runtime_error("SPlane3D::GetMedian : Too few points!");
  
  unsigned halfsize = points4.size()/2;
  vector<double> data(points4.size());

  for (unsigned i=0; i<points4.size(); i++)
    data[i]=points4[i][0];

  sort(data.begin(),data.end());
  median[0] = data[halfsize];

  for (unsigned i=0; i<points4.size(); i++)
    data[i]=points4[i][1];

  sort(data.begin(),data.end());
  median[1] = data[halfsize];

  for (unsigned i=0; i<points4.size(); i++)
    data[i]=points4[i][2];

  sort(data.begin(),data.end());
  median[2] = data[halfsize];
}

/**
 * Compute Least Squares plane
 */
void SPlane3D::FitPlaneLS(double n[3], double p[3])
{
  FitPlaneLS(points4, n, p);
}

/**
 * Compute Least Squares plane and return points projected to the plane
 */
void SPlane3D::ProjectPointsToPlane3D(double n[3], double *points)
{
  ProjectPointsToPlane3D(points4, n, points);
}

bool SPlane3D::ProjectPointsToPlane3DRobust(double n[3], double *points, unsigned &num)
{
  cv::Vec4d p1, p2, p3;
  vector<cv::Point3d> inl;
  double p[3];

  if (FitPlaneRANSAC(points4, p1, p2, p3))
  {
    RefinePlane3dLS(points4, p1, p2, p3, inl, n, p);

    num = inl.size();
    for (unsigned i=0; i<inl.size(); i++)
    {
      points[i*3+0] = inl[i].x;
      points[i*3+1] = inl[i].y;
      points[i*3+2] = inl[i].z;
    }

    return true;
  }
  return false;
}

/**
 * RANSAC based plane detection and least squares fit
 */
bool SPlane3D::FitPlaneRobustLS(double n[3], double p[3])
{
  cv::Vec4d p1, p2, p3;
  vector<cv::Point3d> inl;

  if (FitPlaneRANSAC(points4, p1, p2, p3))
  {
    RefinePlane3dLS(points4, p1, p2, p3, inl, n, p);
    return true;
  }
  return false;
}

}

