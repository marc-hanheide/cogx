/**
 * @file Plane.cpp
 * @author Andreas Richtsfeld
 * @date August 2010
 * @version 0.1
 * @brief Class for creating a dominant plane.
 */

#include "Plane.h"

namespace Z
{

/**
 * @brief Constructor of class plane.
 */
Plane::Plane(Vector3 pos, double rad, std::vector<Vector3> p)
{
	position = pos;
	radius = rad;
	
	for(unsigned i=0; i<p.size(); i++)
		points.push_back(p[i]);
	
	TriangulatePolygon();
}

/**
 * @brief Create polygon from point sequence.
 */
void Plane::TriangulatePolygon()
{
	int i,s;
	int idx = 0;//m_vertices.size();						/// TODO number of vertices (should be 0)
	bool pointInTriangle;
	bool pointIsConvex = false;
	Vector3 e1, e2, n, poly_normal;
	Vector3 v0, v1, v2;
	VisionData::Vertex v;
	VisionData::Face f;
	std::vector<VisionData::Vertex> vertices;
	std::vector<VisionData::Vertex>::iterator v_act, v_pre, v_post, v_in;
	
	s=points.size();
	if(s<3){
		printf("Plane::TriangulatePolygon: warning: too few points for polygon: %d\n", s);
		return;
	}
	
	for(i=0; i<s; i++)
	{
		// Calculate normal
		v0 = points[i];
		v1 = points[(i+1)%s];
		v2 = points[(i+s-1)%s];
		e1 = Normalise(v1-v0);
		e2 = Normalise(v2-v0);
		n = Cross(e1,e2);
		poly_normal = poly_normal + n;	// polygon normal = sum of all normals
		v.pos.x = v0.x;
		v.pos.y = v0.y;
		v.pos.z = v0.z;
		vertices.push_back(v);
	}
	normal = Normalise(poly_normal);	// normalize polygon normal
	v_act = vertices.begin();
	while(vertices.size()>2)
	{
		if(v_act==vertices.end()-1)
		{
			v_pre		= v_act-1;
			v_post	= vertices.begin();
		}
		else if(v_act==vertices.begin())
		{
			v_pre		= vertices.end()-1;
			v_post	= v_act+1;
		}
		else
		{
			v_pre 	= v_act-1;
			v_post	= v_act+1;
		}

		// Test if triangle is convex
		v0.x = (*v_act).pos.x;
		v0.y = (*v_act).pos.y;
		v0.z = (*v_act).pos.z;		
		v1.x = (*v_post).pos.x;
		v1.y = (*v_post).pos.y;
		v1.z = (*v_post).pos.z;		
		v2.x = (*v_pre).pos.x;
		v2.y = (*v_pre).pos.y;
		v2.z = (*v_pre).pos.z;
		e1 = Normalise(v1-v0);
		e2 = Normalise(v2-v0);
		Vector3 zw0;
		zw0 = Cross(e1, e2);
		zw0 = Normalise(zw0);
		(*v_act).normal.x = zw0.x;
		(*v_act).normal.y = zw0.y;
		(*v_act).normal.z = zw0.z;

		if(Dot(zw0, normal) > 0.0)
			pointIsConvex = true;
		
		// 	Test if any other point of remaining polygon lies in this triangle
		pointInTriangle = false;
		if(pointIsConvex)
		{
			for(v_in=vertices.begin(); v_in<vertices.end() && vertices.size()>3; v_in++)
			{
				if(v_in!=v_act && v_in!=v_pre && v_in!=v_post)
				{
					Vector3 in, post, act, pre;
					in.x = (*v_in).pos.x;
					in.y = (*v_in).pos.y;
					in.z = (*v_in).pos.z;
					post.x = (*v_post).pos.x;
					post.y = (*v_post).pos.y;
					post.z = (*v_post).pos.z;		
					act.x = (*v_act).pos.x;
					act.y = (*v_act).pos.y;
					act.z = (*v_act).pos.z;		
					pre.x = (*v_pre).pos.x;
					pre.y = (*v_pre).pos.y;
					pre.z = (*v_pre).pos.z;
					
					if( PointOnSameSide(in, post, act, pre) && 
							PointOnSameSide(in, act, pre, post) &&
							PointOnSameSide(in, pre, post, act))
					{
						pointInTriangle = true;
					}
				}
			}
		}

		if(pointIsConvex && !pointInTriangle)
		{
			// Generate face
			f.vertices.clear();

			(*v_pre).normal.x = normal.x;
			(*v_pre).normal.y = normal.y;
			(*v_pre).normal.z = normal.z;
			(*v_act).normal.x = normal.x;
			(*v_act).normal.y = normal.y;
			(*v_act).normal.z = normal.z;
			(*v_post).normal.x = normal.x;
			(*v_post).normal.y = normal.y;
			(*v_post).normal.z = normal.z;

			p_vertices.push_back(*v_pre);  f.vertices.push_back(idx); idx++;
			p_vertices.push_back(*v_act);  f.vertices.push_back(idx); idx++;
			p_vertices.push_back(*v_post); f.vertices.push_back(idx); idx++;

// 			f.normal = poly_normal;		// The "VisionData::Face" has no normal!
			p_faces.push_back(f);
			vertices.erase(v_act);
		}
		else
		{
			v_act = v_post;
		}
	}
}

/**
 * @brief Check if point is on same side.
 * @param p1 
 * @param p2
 * @param a
 * @param b
 * @return Returns true, if point is on same side.
 */
bool Plane::PointOnSameSide(Vector3 p1, Vector3 p2, Vector3 a, Vector3 b)
{
	Vector3 cp1, cp2;
	cp1 = Cross(b-a, p1-a);
	cp2 = Cross(b-a, p2-a);
	if( (Dot(cp1, cp2)) >= 0.0)
		return true;
	return false;
}


/**
 * @brief Check if point is on same side.
 * @param obj Visual object pointer for plane.
 * @return True for success.
 */
bool Plane::GetVisualObject(VisionData::VisualObjectPtr &obj)
{
	obj->model = new VisionData::GeometryModel;

	for(unsigned i=0; i<p_vertices.size(); i++)
		obj->model->vertices.push_back(p_vertices[i]);
	for(unsigned i=0; i<p_faces.size(); i++)
		obj->model->faces.push_back(p_faces[i]);

	return true;
}

/**
 * @brief Find intersection point of plane and straight line.
 * p = ray.start + ray.dir * z (1)
 * p*n + d = 0   ; point p must be on plane (with normal vector n and distance d) (2)
 * (1) into (2) and solve!
 * @param isct The calculated intersection point
 * @return True, if the intersection point is inside the plane.
 */
bool Plane::IntersectPlaneAndRay(Vector3 &isct, Vector3 point, Vector3 dir)
{
	double d = - Dot(position, normal);		// distance of plane form origin 
	double z = -(Dot(point, normal) + d) / (Dot(dir, normal));
	isct = point + dir*z;

	// return false, when point is behind ground plane!
	if(z > 0) return false;
	
	// test if point lies inside plane hull
	return(PointInPlane(isct));
}

/**																																					/// TODO TODO TODO TODO TODO Funktioniert nicht richtig: ist radius richtig? distance?
 * @brief Returns true, if point is within plane radius.
 * @return Returns true, when point is inside plane radius.
 */
bool Plane::PointInPlane(Vector3 point)
{
// 	printf("Plane::PointInPlane: Test new object point!\n");
// 	printf("Plane::PointInPlane: point: %4.2f / %4.2f / %4.2f\n", point.x, point.y, point.z);
		
	double distance = Norm(point - position);
// 	printf("  => point: %4.2f / %4.2f / %4.2f\n", point.x, point.y, point.z);
// 	printf("  => position: %4.2f / %4.2f / %4.2f\n", position.x, position.y, position.z);
// 	printf("  => distance < radius: %4.2f < %4.2f\n", distance, radius);
	if(distance < 1.5*radius) return true;
	else return false;
}


}




