/**
 * $Id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_KD_VERTEX_HH
#define P_KD_VERTEX_HH

#include "PNamespace.hh"
#include "Vector2.hh"
#include "Keypoint.hh"
#include "KeypointDescriptor.hh"
#include "Array.hh"

namespace P
{


class KDVertex
{
public:
  KeypointDescriptor *kp;   //link to a pair of keypoints
  Vector2 p;          //location of the current keypoint (kp->k2)

  Array<KDVertex*> links;
  Array<double > dist;

  int nb;
  static int nbcnt;

  int ncl;
  const static int CL_DP = 0;              //dominant plane marker
  const static int CL_NOT_SET = -1;        //cluster id not set

  KDVertex();
  KDVertex(KeypointDescriptor* k);
  KDVertex(KeypointDescriptor* k, int _ncl);
  KDVertex(KeypointDescriptor *k, unsigned n);
  KDVertex(double x, double y);
  ~KDVertex();

  void Cluster(Array<KeypointDescriptor *> &mps, double thr);

  inline double X(){return p.x;}
  inline double Y(){return p.y;}
};

/*********************** INLINE METHODES **************************/


}

#endif

