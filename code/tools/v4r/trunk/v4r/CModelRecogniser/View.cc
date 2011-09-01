/**
 * $Id$
 */


#include "View.hh"




namespace P 
{



/************************************************************************************
 * Constructor/Destructor
 */

View::View()
 : id(UINT_MAX)
{ 
}

View::~View()
{
}


/***************************************************************************************/

/**
 * Deep copy a view
 */
void View::copyTo(cv::Ptr<View> &dst)
{
  if (dst.empty()) dst = new View();

  dst->id = id;
  dst->idx = idx; 
  dst->time = time;

  dst->pose = pose;
  dst->vr = vr;
  dst->center = center;

  vector< cv::Ptr<PKeypoint> > keys;

  //copy keypoint
  dst->keys.resize(keys.size());
  for (unsigned i=0; i<keys.size(); i++)
  {
    keys[i]->id = i;
    dst->keys[i] = new PKeypoint(*keys[i]);
    dst->keys[i]->view = dst;
  }
  //copy links
  set<PKeypoint*>::iterator it;
  for (unsigned i=0; i<keys.size(); i++)
  {
    for (it=keys[i]->links.begin(); it!=keys[i]->links.end(); it++)
      dst->keys[i]->InsertLink(*dst->keys[(*it)->id]);
  }

  descriptors.copyTo(dst->descriptors);

  dst->pointcloud = pointcloud;
}

/**
 * stream to file
 */
void View::save(ofstream &os)
{
  os<<id<<' '<<idx<<' '<<time<<' '<<pose<<' '<<vr.x<<' '<<vr.y<<' '<<vr.z<<' '<<center.x<<' '<<center.y<<'\n';

  //save keypoints
  os<<keys.size()<<'\n';
  for (unsigned i=0; i<keys.size(); i++)
  {
    os<<*keys[i]<<'\n';
    keys[i]->id = i;
  }

  //save descriptors
  os<<descriptors.rows<<' '<<descriptors.cols<<'\n';
  float *d = descriptors.ptr<float>();
  for (unsigned i=0; i<descriptors.rows*descriptors.cols; i++,d++)
    os<<(*d)<<' ';
  os<<'\n';

  // save point cloud
  os<<pointcloud.size()<<'\n';
  for (unsigned i=0; i<pointcloud.size(); i++)
    os<<pointcloud[i][0]<<' '<<pointcloud[i][1]<<' '<<pointcloud[i][2]<<' '<<pointcloud[i][3]<<' ';
  os<<'\n';
}

/**
 * stream to file
 */
void View::load(ifstream &is)
{
  unsigned tmp, tmp2;

  is>>id>>idx>>time;
  is>>pose>>vr.x>>vr.y>>vr.z>>center.x>>center.y;

  //save keypoints
  is>>tmp;
  keys.resize(tmp);
  for (unsigned i=0; i<keys.size(); i++)
  {
    keys[i] = new PKeypoint();
    is>>(*keys[i]);
  }

  //load descriptors
  unsigned rows, cols;

  is>>rows>>cols;
  if (rows>0 && cols>0)
  {
    descriptors = cv::Mat_<float>(rows,cols);
    float *d = descriptors.ptr<float>();

    for (unsigned i=0; i<descriptors.rows*descriptors.cols; i++,d++)
      is>>(*d);
  }

  //load point cloud
  is>>tmp;
  if (tmp>0)
  {
    pointcloud.resize(tmp);
    for (unsigned i=0; i<pointcloud.size(); i++)
      is>>pointcloud[i][0]>>pointcloud[i][1]>>pointcloud[i][2]>>pointcloud[i][3];
  }
}


}












