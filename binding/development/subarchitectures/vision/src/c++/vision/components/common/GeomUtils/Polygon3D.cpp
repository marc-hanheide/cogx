/** @file Polygon3D.cpp
 *  @brief A 3D region implemented as a 3D polygon.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#include "Polygon3D.h" 

namespace Geom {

using namespace std;

Polygon3D::Polygon3D(void) {

}

Polygon3D::Polygon3D(const vector<Vector3D> contour3D) {
  init(contour3D);
}


Polygon3D::~Polygon3D(void) {
  poly_3D.clear();
}




void Polygon3D::init(const vector<Vector3D> contour3D) {
  unsigned int idx;

  poly_3D.clear();
  for (idx=0; idx < contour3D.size(); idx++) {    
    poly_3D.push_back(contour3D[idx]);
  }
}


Vector3D Polygon3D::compute_center(void) {
  Vector3D ctr;
  float totalpnts;

  list<Vector3D>::const_iterator iter;
  for (iter=poly_3D.begin(); iter != poly_3D.end(); iter++)  {
    ctr = ctr + (*iter);
  }
  
  totalpnts = (float) poly_3D.size();
  totalpnts = 1/totalpnts;

  ctr = ctr*totalpnts;

  return (ctr);
}


Polygon3D& Polygon3D::operator=(const Polygon3D &src) {
  if( this == &src ) 
      return (*this);

  poly_3D.clear();
  list<Vector3D>::const_iterator iter;
  for (iter=src.poly_3D.begin(); iter != src.poly_3D.end(); iter++)  {
      poly_3D.push_back(*iter); 
  }
  return (*this);

}


ostream& operator<<(ostream &out, Polygon3D &p) {
  p.print(&out);
  return out;
}


void Polygon3D::print(ostream *os) {
  list<Vector3D>::const_iterator iter;
  int i=0;

  for (iter=poly_3D.begin(); iter != poly_3D.end(); iter++)  {
    *os << *iter << "----" << i << endl ;
    i++;
  }

}


}
