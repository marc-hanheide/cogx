/**
 * $Id$
 * Builds a graph of which links planes depending on common motion based on the fundamental matrix
 */


#include "MergePlanesHF.hh"




namespace P 
{


/************************************************************************************
 * Constructor/Destructor
 */

MergePlanesHF::MergePlanesHF(Parameter p)
 : param(p)
{ 
  INV_SQR_SIGMA2 = 1./(2*PMath::Sqr(param.sigmaDistF));
}

MergePlanesHF::~MergePlanesHF()
{
}



/**
 * GetPoints
 */
bool MergePlanesHF::GetPoints(const Plane &plane, vector<cv::Point2f> &pts1, vector<cv::Point2f> &pts2)
{
  if (plane.keys.size()!=plane.lastKeys.size())
    return false;

  for (unsigned i=0; i<plane.keys.size(); i++)
  {
    const PKeypoint &key1 = *plane.lastKeys[i];
    pts1.push_back(cv::Point2f(key1.pt.x, key1.pt.y));
    const PKeypoint &key2 = *plane.keys[i];
    pts2.push_back(cv::Point2f(key2.pt.x, key2.pt.y));
  }

  return true;
}

/**
 * ComputeError (fundamental matrix)
 */
void MergePlanesHF::ComputeErrorF(vector<cv::Point2f> &pts, vector<cv::Vec3f> &lines, double &error)
{
  if (pts.size()!=lines.size())
    return;

  double prob;
  error=0.;
  
  for (unsigned i=0; i<pts.size(); i++)
  {
    prob =  1.-exp( -(PMath::Sqr(SLine2D::AbsDistPointToLine(pts[i],lines[i]))*INV_SQR_SIGMA2) );
    error+=prob;
  }
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

cv::line(dbg, plane1.lastKeys[idxi]->pt, plane2.lastKeys[idxj]->pt, CV_RGB(255,255,255));
cv::circle(dbg, plane1.lastKeys[idxi]->pt, 1, CV_RGB(255,255,255), 2);
cv::circle(dbg, plane2.lastKeys[idxj]->pt, 1, CV_RGB(255,255,255), 2);
cv::line(dbg, mid, pt1, CV_RGB(255,0,0));
cv::line(dbg, mid, pt2, CV_RGB(255,0,0));

 
  error = PVec::Distance2(&pt1.x, &pt2.x);
}






/***************************************************************************************/


/**
 * Operate
 */
void MergePlanesHF::Operate(const vector< cv::Ptr<Plane> > &planes)
{
  cv::Mat F;
  vector<uchar> status;
  vector<cv::Vec3f> lines;
  vector<cv::Point2f> pts1, pts2;
  bool havePoints;
  double error;

cv::Mat img;
dbg.copyTo(img);
  for (unsigned i=0; i<planes.size(); i++)
  {
    for (unsigned j=i+1; j<planes.size(); j++)
    {
      /*if (planes[i]->haveMotion && planes[j]->haveMotion)
      { 
        pts1.clear(), pts2.clear();
        havePoints = GetPoints(*planes[i], pts1, pts2);
        havePoints &= GetPoints(*planes[j], pts1, pts2);

        if (havePoints)
        {
          F = cv::findFundamentalMat(cv::Mat(pts1), cv::Mat(pts2), status, cv::FM_RANSAC, 1., 0.99 );
          cv::computeCorrespondEpilines( cv::Mat(pts1), 1, F, lines );
          ComputeErrorF(pts2, lines, error);
          
for (unsigned k=0; k<lines.size(); k++)
{
DrawLine(dbg,lines[k]);
cv::circle(dbg, pts2[k], 2, CV_RGB(0,0,255), 2);
}
cout<<planes[i]->id<<"/"<<planes[j]->id<<": error="<<error<<endl;

cv::imshow("Image",dbg);
cv::waitKey(0);
img.copyTo(dbg);
        }
      }*/

      ComputeErrorH(*planes[i], *planes[j], error);
cout<<"error="<<error<<endl;
cv::imshow("Image",dbg);
cv::waitKey(0);
img.copyTo(dbg);
     
    }
  }
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












