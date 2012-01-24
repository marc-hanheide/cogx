/**
 * $Id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_KEYPOINT_DESCRIPTOR_HH
#define P_KEYPOINT_DESCRIPTOR_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <opencv/cv.h>
#include "PNamespace.hh"
#include "Keypoint.hh"
#include "Array.hh"
#include "Vector2.hh"
#include "SDraw.hh"
#include "Color.hh"

namespace P
{

//#define SAVE_PATCHES
#define PATCH_MASK_SIZE 16 


class KeypointDescriptor : public Keypoint
{
public:
  enum Type
  {
    DOG_SIFT,
    LOWE_DOG_SIFT,
    MSER_SIFT,
    UVMSER_SIFT,
    HESLAP_SIFT,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };

  static float DOG_SIFT_THR;
  static float LOWE_DOG_SIFT_THR;
  static float MSER_SIFT_THR;
  static float COL_THR;
  static float HESLAP_SIFT_THR;


public:
  static const unsigned DELETE;             //delete occurrence
  static const unsigned INSERT;             //insert to codebook
  static const unsigned DETECTED;           //keypoint detected flag

  unsigned flags;     //flags

  Type type;
  float *vec;
  unsigned size;

  CvMat *pos;            // 3d location
  IplImage *patch;       //normalised grey scale patch

  float error;
  unsigned cntError;

  KeypointDescriptor();
  KeypointDescriptor(Type t);
  KeypointDescriptor(KeypointDescriptor *k);
  KeypointDescriptor(Type t, double x, double y, float s, float a);
  KeypointDescriptor(Type t, double x,double y,float s,float a, float _m11,float _m12,float _m21,float _m22);
  ~KeypointDescriptor();

  static const char* TypeName(Type t);
  static Type EnumType(const char *type_name);
  bool GetVoteCenter(KeypointDescriptor *current, Vector2 &v, double &delta_angle, double &delta_scale);
  void SavePatch(IplImage *img);
  void ProjectPatch(KeypointDescriptor *model,IplImage *img);
  void ReadKeypoint( ifstream &in );
  void ReadDescriptor( ifstream &in, unsigned size_in);
  void WriteKeypoint( ofstream &out );
  void WriteDescriptor(ofstream &out );

  void Copy(KeypointDescriptor *k);
  void AllocVec(unsigned s);
  void CopyVec(KeypointDescriptor *k);
  inline float* GetVec(){return vec;}
  inline float GetVec(unsigned i){return (i<size?vec[i]:0);}
  inline Type GetType(){return type;}
  inline unsigned GetSize(){return size;} 
  inline bool Have3D(){return (pos != 0 ? true:false);}
  inline void Set3D(double p[3]);
  inline void Release3D();
  float DistSqr(float *desc1, float *desc2, unsigned cnt);
  void Add(float *desc);
  void Div(float num);
  void Mul(float num);
  void SetZero();
 
  static void LoadAll(ifstream &is, KeypointDescriptor &k);
  static void SaveAll(ofstream &os, const KeypointDescriptor &k);
  static void Draw(IplImage *img, KeypointDescriptor &k, CvScalar col);  
};


void CopyVec(float *svec, float *dvec, unsigned size);
float MatchKeypoint(KeypointDescriptor *k1, KeypointDescriptor *k2);
void WriteKeypoints(P::Array<KeypointDescriptor*> &ks, const char* file, int format=0);
void LoadKeypoints(const char* file, P::Array<KeypointDescriptor*> &ks, int format=0);
void LoadLoweKeypoints(const char* file, P::Array<Keypoint*> &ks, int format=0);
void LoadLoweKeypoints(const char* file, P::Array<KeypointDescriptor*> &ks, int format=0);
void DeleteKeypoints(Array<KeypointDescriptor*> &keys);
void DeleteKeypoint(KeypointDescriptor* key);
void CopyKeypoints(Array<KeypointDescriptor*> &src, Array<KeypointDescriptor*> &dst);








/*************************** INLINE METHODES **************************/
inline void KeypointDescriptor::Release3D()
{
  cvReleaseMat(&pos);
  pos=0;
}

inline void KeypointDescriptor::Set3D(double p[3])
{
  if (pos==0) pos = cvCreateMat(3,1,CV_64F); 
  pos->data.db[0] = p[0];
  pos->data.db[1] = p[1];
  pos->data.db[2] = p[2];
}

inline void KeypointDescriptor::AllocVec(unsigned s)
{
  if (vec!=0) delete[] vec;
  size=s;
  vec = new float[size];
}

inline void KeypointDescriptor::CopyVec(KeypointDescriptor *k)
{
  if (k->vec!=0){
    AllocVec(k->size);
    for (unsigned i=0; i<size; i++){
      vec[i]=k->vec[i];
    }
  }
}

inline void KeypointDescriptor::Copy(KeypointDescriptor *k)
{
  p=k->p;
  if (k->Have3D())
  {
    if (!Have3D()) pos = cvCreateMat(3,1, CV_64F);
    cvCopy(k->pos,pos);
  }
  scale=k->scale;
  angle=k->angle;

  mi11=k->mi11;
  mi12=k->mi12;
  mi21=k->mi21;
  mi22=k->mi22;
  
  type=k->type;
  CopyVec(k);
  
  if (k->patch!=0){ 
    if (patch==0) patch=cvCreateImage( cvSize(PATCH_MASK_SIZE,PATCH_MASK_SIZE), IPL_DEPTH_8U, 1 );
    cvCopy(k->patch, patch);
  }
  error = k->error;
  cntError = k->cntError;
  flags = k->flags;
  id = k->id;
}

/**
 * Return squared distance between two keypoint descriptors.
 */
inline float KeypointDescriptor::DistSqr(float *desc1, float *desc2, unsigned cnt)
{
    if (cnt==0) return FLT_MAX;

    register unsigned i;
    float dif;
    float distsq = 0;

    for (i = 0; i < cnt; i++) {
      dif = *desc1++ - *desc2++;
      distsq += dif * dif;
    }

    return distsq;
}

/**
 * Add two descriptors
 */
inline void KeypointDescriptor::Add(float *desc)
{
  register unsigned i;
  register float *d = GetVec();

  for (i = 0; i < GetSize(); i++) {
    *d++ += *desc++;
  }
}

/**
 * Set a descriptor to 0
 */
inline void KeypointDescriptor::SetZero()
{
  register float *d=GetVec();
  register unsigned z;

  for (z=0; z< GetSize(); z++){
    *d++ = 0.;
  }
}

/**
 * Devide a descriptor by ..
 */
inline void KeypointDescriptor::Div(float num)
{
  register float *d=GetVec();
  register unsigned z;

  for (z=0; z< GetSize(); z++){
    *d++ /= num;
  }
}

/**
 * Multiply a descriptor with ..
 */
inline void KeypointDescriptor::Mul(float num)
{
  register float *d=GetVec();
  register unsigned z;

  for (z=0; z< GetSize(); z++){
    *d++ *= num;
  }
}



/**
 * only copies a vector of floats
 */
inline void CopyVec(float *svec, float *dvec, unsigned size)
{
  for (register unsigned i=0; i<size; i++){
    *dvec++=*svec++;
  }
}

inline float MatchKeypoint(KeypointDescriptor *k1, KeypointDescriptor *k2)
{
  if (k1->GetType() != k2->GetType())
    return FLT_MAX;
  return k1->DistSqr(k1->vec, k2->vec, k1->GetSize()); 
}


} //--END--

#endif

