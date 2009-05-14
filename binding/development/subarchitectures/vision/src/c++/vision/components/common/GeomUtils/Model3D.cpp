/** @file Model3D.cpp
 *  @brief Models of a 3D pose and 3D bounding box.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#include "Model3D.h" 
#include "vision/components/common/SystemUtils/Common.h"

namespace Geom {

    using namespace Common;

BBox3D::BBox3D() {  
}

BBox3D::~BBox3D() {
}

void BBox3D::getBottomFace(vector<Vector3D> &pnts) 
{
    pnts.clear();
    pnts.push_back(Vector3D(m_centroid.x() - (m_size.x()/2),
			    m_centroid.y() - (m_size.y()/2),
			    m_centroid.z() - (m_size.z()/2)));
    pnts.push_back(Vector3D(m_centroid.x() + (m_size.x()/2),
			    m_centroid.y() - (m_size.y()/2),
			    m_centroid.z() - (m_size.z()/2)));
    pnts.push_back(Vector3D(m_centroid.x() + (m_size.x()/2),
			    m_centroid.y() + (m_size.y()/2),
			    m_centroid.z() - (m_size.z()/2)));
    pnts.push_back(Vector3D(m_centroid.x() - (m_size.x()/2),
			    m_centroid.y() + (m_size.y()/2),
			    m_centroid.z() - (m_size.z()/2)));
}

void BBox3D::getTopFace(vector<Vector3D> &pnts) 
{
     pnts.clear();
     pnts.push_back(Vector3D(m_centroid.x() - (m_size.x()/2),
			     m_centroid.y() - (m_size.y()/2),
			     m_centroid.z() + (m_size.z()/2)));
     
     pnts.push_back(Vector3D(m_centroid.x() + (m_size.x()/2),
			     m_centroid.y() - (m_size.y()/2),
			     m_centroid.z() + (m_size.z()/2)));

     pnts.push_back(Vector3D(m_centroid.x() + (m_size.x()/2),
			     m_centroid.y() + (m_size.y()/2),
			     m_centroid.z() + (m_size.z()/2)));

     pnts.push_back(Vector3D(m_centroid.x() - (m_size.x()/2),
			     m_centroid.y() + (m_size.y()/2),
			     m_centroid.z() + (m_size.z()/2)));
}




ostream& operator<<(ostream &os, BBox3D &bbox) 
{
    try
    {
	vector<Vector3D> bPnts;
	vector<Vector3D> tPnts;
	bbox.getBottomFace(bPnts);
	bbox.getTopFace(tPnts);
	os << bbox.centroid();  
	user_assert(bPnts.size()==4, 
		    __HERE__, "A bbox face must have 4 corners");
	for (int i=0; i<4; i++)
	    os << bPnts[i];
	user_assert(tPnts.size()==4,
		    __HERE__, "A bbox face must have 4 corners");
	for (int i=0; i<4; i++)
	    os << tPnts[i];
	os << endl;
    }
    catch (exception& e)
    {
	cout << e.what() << endl;
	// nothing to clean up
    }
    
    return os;
}



Pose3D::Pose3D() {
}

Pose3D::~Pose3D() {
}

void Pose3D::setZero() {
  m_position.set(0.0, 0.0, 0.0);
  m_orientation.set(0.0, 0.0, 0.0);
}

Pose3D& Pose3D::operator=(const Pose3D& src) {
    if (this == &src)
	return(*this);
    
    m_position = src.position();
    m_orientation = src.orientation();
    return (*this);
}


istream& operator>>(istream &is, Pose3D &pose) {
    return is >> pose.m_position >> pose.m_orientation;
}

}
