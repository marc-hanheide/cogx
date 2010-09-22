/**
 * $Id$
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_POSE_CV_HH
#define P_POSE_CV_HH

#include <limits.h>
#include <map>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <iostream>
#include "PNamespace.hh"
#include "Array.hh"


namespace P
{


class PoseCv
{
private:    
  

public:
  CvMat *R;
  CvMat *t;
  CvMat *n;

  PoseCv();
  ~PoseCv();
  inline void GetHom(CvMat *mat);
  inline void SetHom(CvMat *mat);
};

void InitializePoseCv(PoseCv &pose);
void DeletePoseCv(Array<PoseCv*> &ps);

inline void CopyPoseCv(PoseCv &in, PoseCv &out);
inline void Rot2Quat(CvMat *M, double *q);
inline void Rot2Vec3(CvMat *R, double *d);
inline void Quat2Rot(double *q, CvMat *M);
inline void Vec32Rot(double *d, CvMat *R);
inline void InvPoseCv(PoseCv &in, PoseCv &out);
inline void MulPoseCv(PoseCv &in1, PoseCv &in2, PoseCv &out);
inline void Pose2ProjMat(PoseCv &pose, CvMat *C, CvMat *P);
inline void RT2ProjMat(double R[9], double t[3], double C[9], double P[12]);




/*********************************** INLINE *********************************/
inline void PoseCv::GetHom(CvMat *mat)
{
  CvMat sub;
  cvGetSubRect(mat, &sub, cvRect(0,0,3,3));
  cvCopy(R, &sub);
  cvGetSubRect(mat, &sub, cvRect(3,0,1,3));
  cvCopy(t, &sub);
  cvmSet(mat,3,0,0.); cvmSet(mat,3,1,0.); cvmSet(mat,3,2,0.); cvmSet(mat,3,3,1.);
}

inline void PoseCv::SetHom(CvMat *mat)
{
  CvMat sub;
  cvGetSubRect(mat, &sub, cvRect(0,0,3,3));
  cvCopy(&sub, R);
  cvGetSubRect(mat, &sub, cvRect(3,0,1,3));
  cvCopy(&sub, t);
}

inline void CopyPoseCv(PoseCv &in, PoseCv &out)
{ 
  cvCopy(in.R, out.R);
  cvCopy(in.t, out.t);
  cvCopy(in.n, out.n);
}

inline void Rot2Quat(CvMat *M, double *q)
{
  double *R = M->data.db;

  double tmp[4];
  double mag;
  unsigned maxpos;
  tmp[0]=1.0 + R[0] + R[4] + R[8];
  tmp[1]=1.0 + R[0] - R[4] - R[8];
  tmp[2]=1.0 - R[0] + R[4] - R[8];
  tmp[3]=1.0 - R[0] - R[4] + R[8];

  mag=-1.0;
  for(unsigned i=0; i<4; i++){
    if(tmp[i]>mag){
      mag=tmp[i];
      maxpos=i;
    }
  }
  if(maxpos==0){
    q[0]=sqrt(tmp[0])*0.5;
    q[1]=(R[7] - R[5])/(4.0*q[0]);
    q[2]=(R[2] - R[6])/(4.0*q[0]);
    q[3]=(R[3] - R[1])/(4.0*q[0]);
  }
  else if(maxpos==1){
    q[1]=sqrt(tmp[1])*0.5;
    q[0]=(R[7] - R[5])/(4.0*q[1]);
    q[2]=(R[3] + R[1])/(4.0*q[1]);
    q[3]=(R[2] + R[6])/(4.0*q[1]);
  }
  else if(maxpos==2){
    q[2]=sqrt(tmp[2])*0.5;
    q[0]=(R[2] - R[6])/(4.0*q[2]);
    q[1]=(R[3] + R[1])/(4.0*q[2]);
    q[3]=(R[7] + R[5])/(4.0*q[2]);
  }
  else if(maxpos==3){
    q[3]=sqrt(tmp[3])*0.5;
    q[0]=(R[3] - R[1])/(4.0*q[3]);
    q[1]=(R[2] + R[6])/(4.0*q[3]);
    q[2]=(R[7] + R[5])/(4.0*q[3]);
  }
  else
  {
    cout<<"komisch"<<endl;
  }
  // enforce unit length
  mag=q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];

  if(!IsZero(mag-1.))
  {

    mag=1.0/sqrt(mag);
    q[0]*=mag; q[1]*=mag; q[2]*=mag; q[3]*=mag;
  }
}

inline void Rot2Vec3(CvMat *R, double *d)
{
  double t[4];
  Rot2Quat(R,t);

  double mag=sqrt(Sqr(t[0]) + Sqr(t[1]) + Sqr(t[2]) + Sqr(t[3]));
  double sg=(t[0]>=0.0)? 1.0 : -1.0;
  mag=sg/mag;
  d[0] = t[1]*mag;
  d[1] = t[2]*mag;
  d[2] = t[3]*mag;
}

inline void Quat2Rot(double *q, CvMat *M)
{
  double *R = M->data.db;

  // ensure unit length
  double mag = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
  if(!IsZero(mag-1.0))
  {
    mag=1.0/sqrt(mag);
    q[0]*=mag; q[1]*=mag; q[2]*=mag; q[3]*=mag;
  }

  R[0]=q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3];
  R[1]=2*(q[1]*q[2]-q[0]*q[3]);
  R[2]=2*(q[1]*q[3]+q[0]*q[2]);

  R[3]=2*(q[1]*q[2]+q[0]*q[3]);
  R[4]=q[0]*q[0]+q[2]*q[2]-q[1]*q[1]-q[3]*q[3];
  R[5]=2*(q[2]*q[3]-q[0]*q[1]);

  R[6]=2*(q[1]*q[3]-q[0]*q[2]);
  R[7]=2*(q[2]*q[3]+q[0]*q[1]);
  R[8]=q[0]*q[0]+q[3]*q[3]-q[1]*q[1]-q[2]*q[2];
}

inline void Vec32Rot(double *d, CvMat *R)
{
  double q[4];
  q[0] = sqrt(1.0 - Sqr(d[0]) - Sqr(d[1])- Sqr(d[2]));
  q[1] = d[0];
  q[2] = d[1];
  q[3] = d[2];

  Quat2Rot(q,R);  
}

inline void InvPoseCv(PoseCv &in, PoseCv &out)
{
  cvTranspose( in.R, out.R );
  cvGEMM( out.R, in.t, -1, 0, 0, out.t, 0 );
}

inline void MulPoseCv(PoseCv &in1, PoseCv &in2, PoseCv &out)
{
  double dat1[16], dat2[16], dat3[16];
  CvMat pose1 = cvMat(4,4,CV_64F,dat1);
  CvMat pose2 = cvMat(4,4,CV_64F,dat2);
  CvMat pose3 = cvMat(4,4,CV_64F,dat3);

  in1.GetHom(&pose1);
  in2.GetHom(&pose2);

  cvMatMul(&pose1, &pose2, &pose3);

  out.SetHom(&pose3);
}

/**
 * Compute projection matrix
 */
inline void RT2ProjMat(double R[9], double t[3], double C[9], double P[12])
{
  double tmpC[12], tmpRt[16];

  CvMat tmpCvC = cvMat(3, 4, CV_64F, tmpC);
  CvMat tmpCvRt = cvMat(4, 4, CV_64F, tmpRt);
  CvMat cvP = cvMat(3, 4, CV_64F, P);

  tmpC[0]=C[0], tmpC[1]=C[1], tmpC[2]=C[2],  tmpC[3]=0.; 
  tmpC[4]=C[3], tmpC[5]=C[4], tmpC[6]=C[5],  tmpC[7]=0.; 
  tmpC[8]=C[6], tmpC[9]=C[7], tmpC[10]=C[8], tmpC[11]=0.; 

  tmpRt[0]=R[0], tmpRt[1]=R[1], tmpRt[2]=R[2],  tmpRt[3]=t[0]; 
  tmpRt[4]=R[3], tmpRt[5]=R[4], tmpRt[6]=R[5],  tmpRt[7]=t[1]; 
  tmpRt[8]=R[6], tmpRt[9]=R[7], tmpRt[10]=R[8], tmpRt[11]=t[2];
  tmpRt[12]=0.,  tmpRt[13]=0.,  tmpRt[14]=0.,   tmpRt[15]=1.;
 
  cvMatMul(&tmpCvC, &tmpCvRt, &cvP);
}

/**
 * Compute projection matrix
 */
inline void Pose2ProjMat(PoseCv &pose, CvMat *C, CvMat *P)
{
  CvMat *tmpC = cvCreateMat(3, 4, CV_64F);
  CvMat *tmpRt = cvCreateMat(4, 4, CV_64F);
  cvZero(tmpRt);
  cvZero(tmpC);

  for (unsigned i=0; i<3; i++)
    for (unsigned j=0; j<3; j++)
    {
      cvmSet(tmpRt,i, j, cvmGet(pose.R,i,j));
      cvmSet(tmpC, i, j, cvmGet(C, i, j));
    }
  for (unsigned i=0; i<3; i++)
    cvmSet(tmpRt, i, 3, cvmGet(pose.t, i, 0));
  cvmSet(tmpRt,3,3,1.);

  cvMatMul(tmpC,tmpRt, P);

  cvReleaseMat(&tmpRt);
  cvReleaseMat(&tmpC);
}


} //--THE END--

#endif

