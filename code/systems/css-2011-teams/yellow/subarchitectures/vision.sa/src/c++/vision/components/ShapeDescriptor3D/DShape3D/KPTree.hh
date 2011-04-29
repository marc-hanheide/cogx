/**
 * $Id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_KP_TREE_HH
#define P_KP_TREE_HH

#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include "Vector2.hh"
#include "Array.hh"
#include "PNamespace.hh"
#include "limits.h"
#include "KDVertex.hh"
#include "Keypoint.hh"
#include "KeypointDescriptor.hh"


namespace P
{

//  Delaunay-tree
//
//  Copyright (c) 1993  by  INRIA Prisme Project 
//  2004 route des Lucioles,
//  BP 93 06902 Sophia Antipolis Cedex 
//  All rights reserved.
//  Olivier.Devillers@sophia.inria.fr
//  (33) 93 65 77 63
//  Fax (33) 93 65 76 43
//  http://www.inria.fr:/prisme/personnel/devillers/devillers.html
// 
//
//  GEOMETRIC OBJECT :
//
//    The Delaunay tree is a randomized geometric data structure computing the
//    Delaunay triangulation 
//    This structure holds insertion and queries. Deletions are not supported 
//    in this version.
//
//  Reference:
//                                                              
//    Jean-Daniel Boissonnat, Monique Teillaud,
//    On the randomized construction of the Delaunay tree,
//    Theoretetical Computer Science,
//    Volume 112, pages339-354, 1993.
//
//    Olivier Devillers, Stefan Meiser, Monique Teillaud,
//    Fully dynamic Delaunay triangulation in logarithmic expected 
//    time per operation,
//    Computational Geometry: Theory and Applications,
//    Volume 2, Number 2, pages 55-80, 1992.
//
//    Olivier Devillers,
//    Robust and efficient implementation of the Delaunay tree,
//   INRIA Research Report 1619, 1992.
//

class KPTree;
class KPNode;
class KPList;
typedef unsigned char ind;  // used for flag and ind in array


class KPTree
{
  public :
    // number of the current operation
    int nb;

    // the root of the delaunay_tree
    KPNode* root;

  public :
    KPTree ( void );                        // initialization as empty
    ~KPTree ( void );                       // destructor
    KPTree& operator+=(KDVertex*);             // insertion
    void GetGroups(Array<Array<KeypointDescriptor*> > &mpsGroups);
    void SetVertexLinks();
    inline void SetNB(KDVertex *kdv, int ncl);
};



#ifndef DT_FLAG
#define DT_FLAG
class DTFlag
{
  //private :
  public: 
    ind f;
    DTFlag ( void ) { f = (ind) 0 ; }
    void infinite ( int i ) { f |= (ind) i ; }
    void last_finite ( void ) { f |= 8;}
    void kill ( void ) { f |= 16;}
    ind is_infinite ( void ) { return f & 7;}
    ind is_last_finite ( void )   { return f & 8;}
    ind is_dead ( void )         { return f & 16;}
    ind is_degenerate ( void )    { return f & 32;}
  public :
    friend class KPNode;
    friend class KPTree;
};
#endif

class KPList
{
  //private :
  public:
    KPList *next;
    KPNode *key;
    KPList ( KPList* l, KPNode* k ) {next=l; key=k;}
    ~KPList ( void );
  public :
    friend class KPNode;
    friend class KPTree;
};

class KPNode
{
  //private :
  public:
      // the first vertex is the creator, that is finite
      // except for the root and its neighbors
    DTFlag      flag;
    int nb;
    int ncl;
    KDVertex* vertices[3];
    double length[3]; 
    KPNode*        neighbors[3];
    KPList*    sons;
    KPNode ( void );          // initialize the root
    KPNode(KPNode*, ind);  // initialize nowhere
    KPNode(KPNode*, KDVertex*, ind);
            // father, creator, direction of stepfather
    ind conflict(KDVertex *); 
    KPNode* find_conflict(KDVertex* p);
    ind cw_neighbor_ind(KDVertex *p);
    ind neighbor_ind(KPNode *n);

    void GetNextNode(KPNode* &node, int ncl);

    void Cluster(P::Array<KDVertex*> &out, int ncl);
    void SetVertexLinks();

    inline bool TestNB(KDVertex *kdv, int ncl);

  public :
    friend class KPTree;
};


/***************** INLINE METHODES **********************************************/
inline ind KPNode::cw_neighbor_ind(KDVertex *p)
{ 
  return ((p==vertices[0]) ? 2 : ((p==vertices[1]) ? 0 : 1)); 
}

inline ind KPNode::neighbor_ind(KPNode *n)
{ 
  return ( (neighbors[0]==n)?0:((neighbors[1]==n)?1:2) );
}

inline bool KPNode::TestNB(KDVertex *kdv, int ncl)
{
    if (kdv->kp->nb==ncl)
      return true;
    else
      return false;
}

inline void KPTree::SetNB(KDVertex *kdv, int ncl)
{
  kdv->kp->nb=ncl;
}

}

#endif

