/**
 * $Id$
 */


#include "PKeypointMRF.hh"




namespace P 
{


/************************************************************************************
 * Constructor/Destructor
 */

PKeypointMRF::PKeypointMRF()
{ 
}

PKeypointMRF::PKeypointMRF(const PKeypoint &k)
{
  id=k.id;
  pt=k.pt;
  size=k.size;
  angle=k.angle;
  response=k.response;
  pos=k.pos;
}

PKeypointMRF::PKeypointMRF(const cv::KeyPoint &k)
{
  id = k.class_id;
  pt = cv::Point2d(k.pt.x,k.pt.y);
  size = k.size; 
  angle = k.angle; 
  response = k.response;
  pos.x = badPoint;
}


PKeypointMRF::~PKeypointMRF()
{
}


/***************************************************************************************/



}












