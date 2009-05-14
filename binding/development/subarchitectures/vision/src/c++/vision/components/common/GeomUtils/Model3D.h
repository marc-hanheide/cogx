/** @file Model3D.h
 *  @brief Models of a 3D pose and 3D bounding box.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#ifndef _MODEL_3D_H_
#define _MODEL_3D_H_

#include <iostream>
#include <vector>
#include "Vector3D.h"

using namespace std;

namespace Geom {

using namespace std;

class BBox3D {
 public:
  BBox3D();
  ~BBox3D();

  void getBottomFace(vector<Vector3D> &pnts);
  void getTopFace(vector<Vector3D> &pnts);
  Vector3D centroid() const {return m_centroid;};
  Vector3D size() const {return m_size;};

  void centroid(float x, float y, float z) {m_centroid.set(x,y,z);}
  void size(float x, float y, float z) {m_size.set(x,y,z);}

  friend ostream& operator<<(ostream &os, BBox3D &bbox);  

 private:
  Vector3D m_centroid;
  Vector3D m_size;
};  


class Pose3D {
 public:
  Pose3D();
  ~Pose3D();

  Vector3D position() const {return m_position;};
  Vector3D orientation() const {return m_orientation;};
  void setZero();
  Pose3D& operator=(const Pose3D& src);
  void position(Vector3D pos) {m_position = pos;}
  void orientation(Vector3D orient) {m_orientation = orient;}

  friend istream& operator>>(istream &is, Pose3D &pose);  
  
 public:
  Vector3D m_position;
  Vector3D m_orientation;
};

}

#endif 
