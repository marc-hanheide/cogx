/** @file CameraProj.cpp
 *  @brief Camera projection.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#include <fstream>
#include <opencv/cv.h>
#include "CameraProj.h"
#include "vision/components/common/SystemUtils/Common.h"

using namespace std;
using namespace Common;
using namespace Geom;

template<class T> inline T Sqr(T x) {return x*x;}

// when camera pose was calibrated using KNI, and now we use Golem, coordinate
// systems have to be adjusted
//#define CORRECT_CAM_POSE


static void SetCvMat(CvMat *M, int i, int j, double d) {
  if(i < 0 || i >= M->rows || j < 0 || j >= M->cols)
    throw user_error(__HERE__, "invalid matrix indices: %d %d", i, j);
  M->data.db[i*M->cols + j] = d;
}


static double GetCvMat(CvMat *M, int i, int j)
{
  if(i < 0 || i >= M->rows || j < 0 || j >= M->cols)
    throw user_error(__HERE__, "invalid matrix indices: %d %d", i, j);
  return M->data.db[i*M->cols + j];
}


static void PrintCvMat(CvMat *M)
{
  int i, j;
  for(i = 0; i < M->rows; i++)
  {
    for(j = 0; j < M->cols; j++)
      printf(" %.3f", M->data.db[i*M->cols + j]);
    printf("\n");
  }
}

// Transform a direction from object coordinates to world coordinates.
Vector3D TransformDirToWorld(const Pose3D &pose, const Vector3D &p)
{
  return Rotate(pose.m_orientation, p);
}


/**
 * w = R o + t
 * o = R^T w - R^T t
 */
Pose3D InvertPose3D(const Pose3D &pose)
{
  Pose3D inv;
  inv.m_orientation = -pose.m_orientation;
  inv.m_position = -Rotate(inv.m_orientation, pose.m_position);
  return inv;
}


CameraProj::CameraProj() {
  this->bInitialized = false;
}

CameraProj::~CameraProj() 
{
    std::cout << "Destructing a CameraProj\n"; 
}


CameraProj& CameraProj::operator=(const CameraProj& src) 
{
    if (this == &src)
	return(*this);
    
    bInitialized = true;
    m_num = src.m_num;
    m_pose = src.m_pose;
    m_width = src.m_width;
    m_height = src.m_height;
    m_fx = src.m_fx;
    m_fy = src.m_fy;
    m_cx = src.m_cx;
    m_cy = src.m_cy;
    m_k1 = src.m_k1;
    m_k2 = src.m_k2;
    m_k3 = src.m_k3;
    m_t1 = src.m_t1;
    m_t2 = src.m_t2;
    return(*this);
}

Vector3D CameraProj::projectImagePointToGroundplane(Vector2D p) {
  Vector3D e;  // eye point = origin of line of sight
  Vector3D d;  // direction of line of sight
  Vector3D g;  // point on ground plane
  double l;    // length of line of sight until intersection
  
  // first find direction in camera coordinates
  d.x(p.x - m_cx);
  d.y(p.y - m_cy);
  // note: i assume that fx is (nearly) equal to fy, so it does not matter
  // whether we use fx or fy
  d.z(m_fx);

  // transform dir and eye point to world coordinates
  d = TransformDirToWorld(this->m_pose, d);
  e = m_pose.m_position;

  // find intersection with z = 0 plane
  l = -e.z()/d.z();
  g.x(e.x() + l*d.x());
  g.y(e.y() + l*d.y());
  g.z(0.);

  return g;
}




void CameraProj::setDefault(int _num) {
  this->m_num = _num;  
  this->m_pose.setZero();
  this->m_width = 0;
  this->m_height = 0;
  this->m_fx = this->m_fy = 1.;
  this->m_cx = this->m_cy = 0.;
  this->m_k1 = this->m_k2 = this->m_k3 = 0.;
  this->m_t1 = this->m_t2 = 0.;
}
 

void CameraProj::readCamParms(const Vision::Camera &campose) {
    m_pose.m_position.set(campose.m_pose.m_position.m_x,
			  campose.m_pose.m_position.m_y,
			  campose.m_pose.m_position.m_z);
    
    m_pose.m_orientation.set(campose.m_pose.m_orientation.m_x,
			     campose.m_pose.m_orientation.m_y,
			     campose.m_pose.m_orientation.m_z);
    
    //    cout << "position: " << m_pose.m_position << endl;
    //    cout << "orientation: " << m_pose.m_orientation << endl;


    this->m_width = campose.m_width;
    this->m_height = campose.m_height;
    this->m_fx = campose.m_fx;
    this->m_fy = campose.m_fy;
    this->m_cx = campose.m_cx;
    this->m_cy = campose.m_cy;
    this->m_k1 = campose.m_k1;
    this->m_k2 = campose.m_k2;
    this->m_k3 = campose.m_k3;
    this->m_t1 = campose.m_t1;
    this->m_t2 = campose.m_t2;

}

void CameraProj::readCamParms(char filename[]) {
  ifstream in;
  in.open(filename);

  if(!(in >> this->m_pose))
    throw user_error(__HERE__, "failed to read cam pose");
  if(!(in >> this->m_width))
    throw user_error(__HERE__, "failed to read cam width");
  if(!(in >> this->m_height))
    throw user_error(__HERE__, "failed to read cam height");
  if(!(in >> this->m_fx))
    throw user_error(__HERE__, "failed to read cam fx");
  if(!(in >> this->m_fy))
    throw user_error(__HERE__, "failed to read cam fy");
  if(!(in >> this->m_cx))
    throw user_error(__HERE__, "failed to read cam cx");
  if(!(in >> this->m_cy))
    throw user_error(__HERE__, "failed to read cam cy");
  if(!(in >> this->m_k1))
    throw user_error(__HERE__, "failed to read cam k1");
  if(!(in >> this->m_k2))
    throw user_error(__HERE__, "failed to read cam k2");
  if(!(in >> this->m_k3))
    throw user_error(__HERE__, "failed to read cam k3");
  if(!(in >> this->m_t1))
    throw user_error(__HERE__, "failed to read cam t1");
  if(!(in >> this->m_t2))
    throw user_error(__HERE__, "failed to read cam t2");

#ifdef CORRECT_CAM_POSE
  CvMat *R = cvCreateMat(3, 3, CV_64F);
  CvMat *S = cvCreateMat(3, 3, CV_64F);
  CvMat *r = cvCreateMat(3, 1, CV_64F);

  SetCvMat(r, 0, 0, this->m_pose.m_orientation.x);
  SetCvMat(r, 1, 0, this->m_pose.m_orientation.y);
  SetCvMat(r, 2, 0, this->m_pose.m_orientation.z);

  cvRodrigues2(r, R, (CvMat*)0);

  // rotate coordinate system:
  // x = -y
  SetCvMat(S, 0, 0, -GetCvMat(R, 0, 1));
  SetCvMat(S, 1, 0, -GetCvMat(R, 1, 1));
  SetCvMat(S, 2, 0, -GetCvMat(R, 2, 1));
  // y = -x
  SetCvMat(S, 0, 1, -GetCvMat(R, 0, 0));
  SetCvMat(S, 1, 1, -GetCvMat(R, 1, 0));
  SetCvMat(S, 2, 1, -GetCvMat(R, 2, 0));
  // z = -z
  SetCvMat(S, 0, 2, -GetCvMat(R, 0, 2));
  SetCvMat(S, 1, 2, -GetCvMat(R, 1, 2));
  SetCvMat(S, 2, 2, -GetCvMat(R, 2, 2));
  
  cvRodrigues2(S, r, (CvMat*)0);

  this->m_pose.m_orientation.x = GetCvMat(r, 0, 0);
  this->m_pose.m_orientation.y = GetCvMat(r, 1, 0);
  this->m_pose.m_orientation.z = GetCvMat(r, 2, 0);

  cvReleaseMat(&R);
  cvReleaseMat(&S);
  cvReleaseMat(&r);
#endif

  // note: pose in file is pose of world w.r.t. camera, 
  // therefore we have to invert.
#ifdef INVERSE
  this->m_pose = InvertPose3D(this->m_pose);
#endif


  printf("camera %ld config: pose %f %f %f\n",
	 m_num,
	 m_pose.m_position.x(),
	 m_pose.m_position.y(),
	 m_pose.m_position.z());

  in.close();
}

void CameraProj::projectRoiToWorld(CvBox2D &roi, 
				   BBox3D &bbox,
				   Pose3D &pose) 
{
  Vector2D p, r;
  Vector3D g, t;
  Vector3D d;

  // bottom left of bounding box
  p.x = roi.center.x - (roi.size.width/2.);
  p.y = roi.center.y + (roi.size.height/2.);
  g = this->projectImagePointToGroundplane(p);

  // bottom right of bounding box
  r.x = roi.center.x + (roi.size.width/2.);
  r.y = roi.center.y + (roi.size.height/2.);
  t = this->projectImagePointToGroundplane(r);

  d = (g-t);
  double W = d.norm();
  double H = (W*roi.size.height/roi.size.width);
  
  // get 3D sizes
  //w = this->projectDistanceToGroundplane(p, roi.size.width);
  //h = this->projectDistanceToGroundplane(p, roi.size.height);

  bbox.centroid((g.x() + t.x())/2., (g.y() + t.y())/2., H/2.);
  bbox.size(W, W, H);

  pose.position(bbox.centroid());
  pose.orientation(Vector3D(0,0,0));

  // other attributes (color, shape, label, surfaces) are untouched
}



/**
 * First find point on z = 0 groundplane for a given image point, then projects
 * given distance to 3D distance at that point.
 * @param cam  camera parameters
 * @param p  image point in pixel coordinates
 * @param s  distance in pixel coordinates
 * Note: Does not do image undistortion!
 */
double CameraProj::projectDistanceToGroundplane(Vector2D p, double s)
{
  Vector3D e;  // eye point = origin of line of sight
  Vector3D d;  // direction of line of sight
  Vector3D g;  // point on ground plane
  double L;    // distance on line of sight to intersection
  double l;    // distnace on line of sight to image plane

  // first find direction in camera coordinates
  d.set(p.x-m_cx, p.y-m_cy, m_fx);
  // note: i assume that fx is (nearly) equal to fy, so it does not matter
  // whether we use fx or fy

  // transform dir and eye point to world coordinates
  d = TransformDirToWorld(m_pose, d);
  e = m_pose.m_position;

  // find intersection with z = 0 plane
  L = -e.z()/d.z();
  // S/s = L/l
  l = sqrt(Sqr(m_fx) + Sqr(d.x()));
  return s*L/l;
}



Vector2D CameraProj::projectToImage(Vector3D p3) {
 
  Pose3D inv_pose = InvertPose3D(m_pose);
  
  //Pose3D inv_pose = m_pose;

    Vector3D p3_rot = Rotate(inv_pose.m_orientation, p3);
    //Vector3D p3_rot_trans = Translate(m_pose.m_position, p3_rot);
    Vector3D p3_rot_trans = inv_pose.m_position + p3_rot;
    
//     CvMat *K = cvCreateMat(3, 3, CV_64F);
//     SetCvMat(K, 0, 0, m_fx);    
//     SetCvMat(K, 1, 0, 0);    
//     SetCvMat(K, 2, 0, 0);

//     SetCvMat(K, 0, 1, 0);    
//     SetCvMat(K, 1, 1, m_fy);    
//     SetCvMat(K, 2, 1, 0);

//     SetCvMat(K, 0, 2, m_cx);    
//     SetCvMat(K, 1, 2, m_cy);    
//     SetCvMat(K, 2, 2, 1);
 
//     CvMat *P = cvCreateMat(3, 1, CV_64F);
//     SetCvMat(P, 0, 0, p3_rot_trans.x); 
//     SetCvMat(P, 1, 0, p3_rot_trans.y);    
//     SetCvMat(P, 2, 0, p3_rot_trans.z);

    Vector3D p(p3_rot_trans.x()*m_fx + p3_rot_trans.z()*m_cx , 
	       p3_rot_trans.y()*m_fy + p3_rot_trans.z()*m_cy ,
	       p3_rot_trans.z() );

    Vector2D p_img( p.x()/p.z() , p.y()/p.z() );

    return (p_img);
}
