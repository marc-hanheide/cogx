/**
 * $Id$
 * Builds a graph of which links planes depending on common motion based on the fundamental matrix
 */


#include "MergePlanesHF.hh"

#define DEBUG


namespace P 
{


/************************************************************************************
 * Constructor/Destructor
 */

MergePlanesHF::MergePlanesHF(Parameter p)
 : param(p)
{ 
  INV_SQR_SIGMA2_F = 1./(2*PMath::Sqr(param.sigmaDistF));
  CONST_GAUSS_F = 1. / (param.sigmaDistF*sqrt(2*M_PI));
  INV_SQR_SIGMA2_H = 1./(2*PMath::Sqr(param.sigmaDistH));
}

MergePlanesHF::~MergePlanesHF()
{
}



/**
 * GetPoints
 */
bool MergePlanesHF::GetPoints(const Plane &plane, std::vector<cv::Point2f> &pts1, std::vector<cv::Point2f> &pts2)
{
  if (plane.keys.size()!=plane.lastKeys.size())
    return false;

  double mot=0;
  for (unsigned i=0; i<plane.keys.size(); i++)
  {
    const PKeypoint &key1 = *plane.lastKeys[i];
    pts1.push_back(cv::Point2f(key1.pt.x, key1.pt.y));
    const PKeypoint &key2 = *plane.keys[i];
    pts2.push_back(cv::Point2f(key2.pt.x, key2.pt.y));
    mot += PVec::Distance2(&key1.pt.x,&key2.pt.x);
  }

  mot /= (double)pts1.size();

  return (mot>2?true:false);
}

/**
 * ComputeError (fundamental matrix)
 */
void MergePlanesHF::ComputeErrorF(std::vector<cv::Point2f> &pts, std::vector<cv::Vec3f> &lines, double &error)
{
  if (pts.size()!=lines.size())
    return;

  error=0.;
  for (unsigned i=0; i<pts.size(); i++)
  {
    //error +=  CONST_GAUSS_F*exp( -(PMath::Sqr(SLine2D::AbsDistPointToLine(pts[i],lines[i]))*INV_SQR_SIGMA2_F) );
    error +=  exp( -(PMath::Sqr(SLine2D::AbsDistPointToLine(pts[i],lines[i]))*INV_SQR_SIGMA2_F) );
//cout<<1-exp( -(PMath::Sqr(SLine2D::AbsDistPointToLine(pts[i],lines[i]))*INV_SQR_SIGMA2_F) )<<" ";
  }

  //error = (error>1?0:1-error);
}

/**
 * ComputeErrorH (homography border)
 */
void MergePlanesHF::ComputeErrorH(const Plane &plane1, const Plane &plane2, double &error)
{
  error = DBL_MAX;
  if (plane1.lastKeys.size()!=plane1.keys.size() || plane2.lastKeys.size()!=plane2.keys.size())
    return;

  unsigned idxi1, idxj1, idxi, idxj;
  double dist, min1, min=DBL_MAX;

  for (unsigned i=0; i<plane1.lastKeys.size(); i++)
  {
    min1 = DBL_MAX;
    for (unsigned j=0; j<plane2.lastKeys.size(); j++)
    {
      dist = PVec::DistanceSqr2(&plane1.lastKeys[i]->pt.x, &plane2.lastKeys[j]->pt.x);
      if ( dist < min1)
      {
        idxi1 = i;
        idxj1 = j;
        min1 = dist;
      }
    }
    if (min1<min)
    {
      idxi = idxi1;
      idxj = idxj1;
      min = min1;
    }
  }
 
  cv::Point2d pt1, pt2, mid;
  
  PVec::MidPoint2(&plane1.lastKeys[idxi]->pt.x, &plane2.lastKeys[idxj]->pt.x, &mid.x);
  PHom::MapPoint(&mid.x, &plane1.H(0,0), &pt1.x);
  PHom::MapPoint(&mid.x, &plane2.H(0,0), &pt2.x);

  error = PVec::Distance2(&pt1.x, &pt2.x);
}


/**
 * ComputeProbF
 */
void MergePlanesHF::ComputeProbF(const Plane &plane1, const Plane &plane2, double &prob)
{
  cv::Mat F;
  std::vector<uchar> status;
  std::vector<cv::Vec3f> lines;
  std::vector<cv::Point2f> pts1, pts2;
  bool ok;

  ok = GetPoints(plane1, pts1, pts2);
  ok &= GetPoints(plane2, pts1, pts2);

  prob = 0.;
  if (ok)
  {
    F = cv::findFundamentalMat(cv::Mat(pts1), cv::Mat(pts2), status, cv::FM_RANSAC, 1., 0.99 );
    cv::computeCorrespondEpilines( cv::Mat(pts1), 1, F, lines );
    ComputeErrorF(pts2, lines, prob);
  }
}





/***************************************************************************************/


/**
 * Operate
 */
void MergePlanesHF::Operate(std::vector< cv::Ptr<Plane> > &planes)
{
  double error;
  CvPoint2D32f pt;
  CvMemStorage* storage;
  CvSubdiv2D* subdiv;

  storage = cvCreateMemStorage(0);
  subdiv = cvCreateSubdiv2D( CV_SEQ_KIND_SUBDIV2D, sizeof(*subdiv), sizeof(CvSubdiv2DPoint),
                             sizeof(CvQuadEdge2D), storage );
  cvInitSubdivDelaunay2D( subdiv, cvRect(0,0,param.width+10, param.height+10));

  // insert points
  for (unsigned i=0; i<planes.size(); i++)
  {
    pt = cvPoint2D32f((float)(planes[i]->center.x), (float)(planes[i]->center.y));
    cvSubdivDelaunay2DInsert( subdiv, pt )->id = i;
  }

  // create graph
  double pF;
  CvSeqReader  reader;
  CvSubdiv2DPoint* ptOrg;
  CvSubdiv2DPoint* ptDst;
  int i, total = subdiv->edges->total;
  int elem_size = subdiv->edges->elem_size;

  cvStartReadSeq( (CvSeq*)(subdiv->edges), &reader, 0 );

  for( i = 0; i < total; i++ )
  {
      CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);

      if( CV_IS_SET_ELEM( edge ))
      {
        ptOrg = cvSubdiv2DEdgeOrg((CvSubdiv2DEdge)edge);
        ptDst = cvSubdiv2DEdgeDst((CvSubdiv2DEdge)edge);
        if (ptOrg && ptDst && ptOrg->id!=-1 && ptDst->id!=-1)
        {
          ComputeErrorH(*planes[ptOrg->id], *planes[ptDst->id], error);
          pF = exp( -(PMath::Sqr(error))*INV_SQR_SIGMA2_H) ;
          planes[ptOrg->id]->motLink[planes[ptDst->id]] = pF;
          planes[ptDst->id]->motLink[planes[ptOrg->id]] = pF;
          //ComputeProbF(*planes[ptOrg->id], *planes[ptDst->id], plMerges.back().pF);

          #ifdef DEBUG
          //if (!dbg.empty()) cv::line(dbg, planes[ptOrg->id]->center, planes[ptDst->id]->center, CV_RGB(255,255,255),1);
          //cv::Point2d c;
          //PVec::MidPoint2(&planes[ptOrg->id]->center.x, &planes[ptDst->id]->center.x, &c.x);
          //cv::putText(dbg, toString(plMerges.back().pH), c, cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,255,255), 2);
          #endif
        }
      }

      CV_NEXT_SEQ_ELEM( elem_size, reader );
  }

  cvReleaseMemStorage( &storage );
}



/**
 * Draw line
 */
void MergePlanesHF::DrawLine(cv::Mat &img, cv::Vec3f &l)
{
  cv::Point2f pt1(0,0), pt2(2000,0);

  pt1.y = -(l[2] + l[0]*pt1.x) / l[1];
  pt2.y = -(l[2] + l[0]*pt2.x) / l[1];

  cv::line(img, pt1, pt2, CV_RGB(255,255,255));
}


}












