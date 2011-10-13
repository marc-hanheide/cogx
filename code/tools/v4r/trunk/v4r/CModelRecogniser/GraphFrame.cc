/**
 * $Id$
 */


#include "GraphFrame.hh"




namespace P 
{


/************************************************************************************
 * Constructor/Destructor
 */
GraphFrame::GraphFrame()
{ 
}

GraphFrame::GraphFrame(std::vector<cv::Ptr<Plane> > &pl, unsigned _id, unsigned _idx)
 : id(_id), idx(_idx)
{
  planes = pl;
  for (unsigned i=0; i<planes.size(); i++)
    planes[i]->idx = i;
}

GraphFrame::~GraphFrame()
{
}


/***************************************************************************************/





}












