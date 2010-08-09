/**
 * @file Plane.h
 * @author Andreas Richtsfeld
 * @date August 2010
 * @version 0.1
 * @brief Class for creating a dominant plane.
 */


#ifndef Z_PLANE_HH
#define Z_PLANE_HH

#include "Vector.hh"
#include <stdio.h>
#include <vector>
#include <VisionData.hpp>

namespace Z
{

/**
 * @brief Class for dominant plane calculations
 */
class Plane
{
private:
	std::vector<VisionData::Vertex> p_vertices;				///< list of vertices, relativ to position
	std::vector<VisionData::Face> p_faces;						///< list of faces

	std::vector<Vector3> points;											///< points of the plane, relativ to position

	double radius;																		///< radius from center to farthest point

public:
	Vector3 position;																	///< point on the plane
	Vector3 normal;																		///< normal of the plane

	Plane(Vector3 pos, double rad, std::vector<Vector3> p);
	void TriangulatePolygon();
	bool PointOnSameSide(Vector3 p1, Vector3 p2, Vector3 a, Vector3 b);
  bool GetVisualObject(VisionData::VisualObjectPtr &obj);
	bool IntersectPlaneAndRay(Vector3 &isct, Vector3 point, Vector3 dir);						/// TODO sollte in eine Vector Hilfsklasse (schneiden Gerade/Ebene! für Vector3)
	bool PointInPlane(Vector3 point);
};

}

#endif
