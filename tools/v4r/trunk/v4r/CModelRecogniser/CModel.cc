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

    
  //save links
  for (unsigned j=0; j<views.size(); j++)
  {
    View &view = *views[j];
    set<PKeypoint*>::iterator it;
    for (unsigned i=0; i<view.keys.size(); i++)
    {
      os<<view.keys[i]->links.size()<<' ';
      for (it=view.keys[i]->links.begin(); it!=view.keys[i]->links.end(); it++)
      {
        if ((*it)->HaveView())
          os<<(*it)->view->id<<' '<<(*it)->id<<' ';
        else
          os<<UINT_MAX<<' ';
      }
      os<<'\n';
    }
  }
  
}

/**
 * stream from file
 */
void CModel::load(ifstream &is)
{
  unsigned vid, kid, tmp;
  
  is>>id;
  is>>center.x>>center.y>>center.z;
  is>>tmp;
  views.resize(tmp);
  for (unsigned i=0; i<views.size(); i++)
  {
    views[i] = new View();
    views[i]->load(is);
  }

  //load links
  for (unsigned j=0; j<views.size(); j++)
  {
    View &view=*views[j];
    for (unsigned i=0; i<view.keys.size(); i++)
    {
      view.keys[i]->view = views[j];
      is>>tmp;

      for (unsigned k=0; k<tmp; k++)
      {
        is>>vid;
        if (vid!=UINT_MAX)
        {
          is>>kid;
          view.keys[i]->InsertLink(*views[vid]->keys[kid]);
        }
      }
    }
  }
}


}












