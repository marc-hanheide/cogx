/**
 * $Id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */


#include "Plane.hh"


namespace P 
{

unsigned Plane::idcnt=0;

/********************** Plane ************************
 * Constructor/Destructor
 */
Plane::Plane()
 : id(UINT_MAX),oid(UINT_MAX), thrDist(DBL_MAX)
{
  H = cvCreateMat( 3, 3, CV_64F );
}

Plane::~Plane()
{
  cvReleaseMat(&H);
}

void CopyParameter(Plane* src, Plane* dst)
{
  dst->id = src->id;
  dst->oid = src->oid;
  dst->thrDist = src->thrDist;
}

/**
 * Delete keypoints of planes
 * If keypoints are only stored in the plane it is necessary to release the tracking links of
 * keypoints before deleting the plane.
 * If this is the case this method has to be called before deleting the planes
 */
void ReleaseKeypoints(Array<Plane*> &planes)
{
  for (unsigned i=0; i<planes.Size(); i++)
    DeleteKeypoints(planes[i]->keys);
}


/**
 * DeletePlanes
 */
void DeletePlanes(Array<Plane*> &planes)
{
  for (unsigned i=0; i<planes.Size(); i++)
    delete planes[i];
  planes.Clear();
}

}

