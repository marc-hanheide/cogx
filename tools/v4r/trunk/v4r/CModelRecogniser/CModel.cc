/**
 * $Id$
 */


#include "CModel.hh"




namespace P 
{



/************************************************************************************
 * Constructor/Destructor
 */

CModel::CModel(int subdivHist)
 : center(cv::Point3d(0.,0.,0.))
{
  if (subdivHist>0)
    viewHist = new SphereHistogram(subdivHist);
}

CModel::~CModel()
{
}


/***************************************************************************************/
/**
 * Deep copy a model
 */
void CModel::copyTo(cv::Ptr<CModel> &dst)
{
  if (dst.empty()) dst = new CModel();

  dst->id = id;

  dst->views.resize(views.size());

  for (unsigned i=0; i<views.size(); i++)
    views[i]->copyTo(dst->views[i]);

  if (!viewHist.empty())
  {
    dst->viewHist = new SphereHistogram();
    *dst->viewHist = *viewHist;
  }

  dst->center = center;
  dst->pose = pose;
}

/**
 * clear()
 */
void CModel::clear()
{
  views.clear();
  if (!viewHist.empty()) viewHist->Clear();  
}


/**
 * stream to file
 */
void CModel::save(ofstream &os)
{
  os<<id<<'\n';
  os<<center.x<<' '<<center.y<<' '<<center.z<<'\n';
  os<<views.size()<<'\n';

  for (unsigned i=0; i<views.size(); i++)
  {
    views[i]->id = i;
    views[i]->save(os);
  }

  os<<points.size()<<'\n';
  for (unsigned i=0; i<points.size(); i++)
  {
    os<<points[i]->idx<<' '<<points[i]->pt.x<<' '<<points[i]->pt.y<<' '<<points[i]->pt.z<<' '<<points[i]->projs.size()<<'\n';
    for (unsigned j=0; j<points[i]->projs.size(); j++)
      os<<points[i]->projs[j].first<<' '<<points[i]->projs[j].second<<' ';
    os<<'\n';
  }
}

/**
 * stream from file
 */
void CModel::load(ifstream &is)
{
  unsigned tmp, tmp2;
  
  is>>id;
  is>>center.x>>center.y>>center.z;
  is>>tmp;
  views.resize(tmp);

  for (unsigned i=0; i<views.size(); i++)
  {
    views[i] = new View();
    views[i]->load(is);
  }

  is>>tmp;
  points.resize(tmp);
  for (unsigned i=0; i<points.size(); i++)
  {
    points[i] = new Point3dProjs();
    is>>points[i]->idx>>points[i]->pt.x>>points[i]->pt.y>>points[i]->pt.z;
    is>>tmp2;
    points[i]->projs.resize(tmp2);
    for (unsigned j=0; j<tmp2; j++)
    {
      is>>points[i]->projs[j].first>>points[i]->projs[j].second;
      //link 3d point to keypoints
      views[ points[i]->projs[j].first ]->keys[ points[i]->projs[j].second ]->pos = points[i];
    }
  }

}


}












