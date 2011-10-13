/**
 * $Id$
 */


#include "Plane.hh"




namespace P 
{

unsigned Plane::idcnt=0;

/************************************************************************************
 * Constructor/Destructor
 */

Plane::Plane(unsigned _id)
 : id(_id), col(cv::Scalar(0))
{ 
}

Plane::~Plane()
{
}


/***************************************************************************************/

/**
 * Deep copy a view
 */
void Plane::copyTo(cv::Ptr<Plane> &dst)
{
  if (dst.empty()) dst = new Plane();

  dst->id = id;
  H.copyTo(dst->H);

  //copy keypoint
  dst->keys.resize(keys.size());
  for (unsigned i=0; i<keys.size(); i++)
  {
    dst->keys[i] = new PKeypoint(*keys[i]);
  }

  dst->lastKeys.resize(lastKeys.size());
  for (unsigned i=0; i<lastKeys.size(); i++)
  {
    dst->lastKeys[i] = new PKeypoint(*lastKeys[i]);
  }

  descriptors.copyTo(dst->descriptors);

  dst->center = center;
  dst->haveMotion = haveMotion;
}




}












