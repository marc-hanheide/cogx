/**
 * $Id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */


#include "KDVertex.hh"


namespace P 
{

int KDVertex::nbcnt=0;

/********************** KDVertex ************************
 * Constructor/Destructor
 */
KDVertex::KDVertex()
 : nb(0), ncl(INT_MIN)
{
}

KDVertex::KDVertex(KeypointDescriptor *k)
 : nb(0), ncl(INT_MIN)
{
  kp = k;
  p = k->p;
}

KDVertex::KDVertex(KeypointDescriptor *k, int _ncl)
 : nb(0), ncl(_ncl)
{
  k->nb = _ncl;
  p = k->p;

  kp = k;
}

KDVertex::KDVertex(KeypointDescriptor *k, unsigned n)
 : kp(k), nb(n), ncl(INT_MIN)
{
  kp = k;
  p = k->p;
}

KDVertex::KDVertex(double x, double y)
 : nb(0), ncl(INT_MIN)
{
  p = P::Vector2(x,y);
}


KDVertex::~KDVertex()
{
}

/**
 * Cluster vertices depending on distance
 */
void KDVertex::Cluster(Array<KeypointDescriptor *> &mps, double thr)
{
  mps.PushBack(kp);

  for (unsigned i=0; i<links.Size(); i++)
    if (links[i]->ncl == CL_NOT_SET && dist[i]<thr)
    {
      links[i]->ncl = ncl;
      links[i]->Cluster(mps, thr);
    }
}



}

