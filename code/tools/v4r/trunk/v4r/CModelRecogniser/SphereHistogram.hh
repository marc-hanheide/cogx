/**
 * $Id$
 */

#ifndef P_SPHERE_HISTOGRAM_HH
#define P_SPHERE_HISTOGRAM_HH

#include <set>
#include <vector>
#include <opencv2/core/core.hpp>
#include <iostream>
#include "v4r/PGeometry/SPlane3D.hh"
#include "ConfValues.hh"

namespace P
{

class SFace;


class SphereHistogram
{
private:

  unsigned edgeWalk, numVertices;
  vector<unsigned> start, end, midpoint; //just temp container 

  void InitIcosahedron();
  void Subdevide();
  unsigned SearchMidpoint (unsigned idxStart, unsigned idxEnd);
  void ComputeNormals();
  void CopyFace(SFace *src, SFace *dst, vector<SFace*> &subdiv);
  void DeepCopy(const SphereHistogram *src, SphereHistogram *dst);
  bool FindMatch(cv::Point3d &n, vector<SFace*> *in, vector<SFace*> **out, SFace **match);
  void SetNeighbours();



public:
  double sumWeight;
  vector<cv::Point3d> vertices;
  vector<SFace*> startFaces;     // 20 icosahedron faces
  vector<SFace*> subdivFaces;    // subdevided faces depending on init

  SphereHistogram(unsigned subdevisions=1);
  ~SphereHistogram();

  void Release();
  void Init(unsigned subdevisions);
  void Insert(cv::Point3d n, double weight, unsigned id);
  void InsertMax(unsigned idx, cv::Point3d vr, ConfValues &pred);
  double GetMax();
  void Normalise(double norm);
  void Normalise();
  void GetMin(cv::Point3d vr, double &min);
  SFace* GetFace(cv::Point3d n);
  void GetViewRays(vector<cv::Point3d> &vr, double minScore, double maxScore);
  void Clear();

  void copyTo(SphereHistogram &dst);

  SphereHistogram& operator=(const SphereHistogram &sphere);
};



class SFace
{
public:
  unsigned vs[3];                 // vertices
  vector<SFace*> subFaces;        // links to 4 subdivided faces
  SFace* neighbours[3];

  cv::Point3d n;                  // normal
  double weight;                  // what ever you want to accumulate
  unsigned idx;                   // link to something...
  set<unsigned> views;            // can be links to something

  SFace() : subFaces(0), weight(0.), idx(UINT_MAX) {};
  ~SFace(){
    for (unsigned i=0; i<subFaces.size(); i++)
      delete subFaces[i];
  }
};


/*********************** INLINE METHODES **************************/






}

#endif

