/**
 * @file Object.cpp
 * @author Andreas Richtsfeld
 * @date August 2010
 * @version 0.1
 * @brief Class for objects.
 */

#include "Object.h"

namespace Z
{

/**
 * @brief Constructor of class object.
 */
Object::Object(Vector3 pos, VisionData::VertexSeq vertices, VisionData::FaceSeq faces)
{
	position = pos;
	for(unsigned i=0; i<vertices.size(); i++)
		o_vertices.push_back(vertices[i]);
	for(unsigned i=0; i<faces.size(); i++)
		o_faces.push_back(faces[i]);
}


/**
 * @brief Project the faces to the ground plane and create object.
 * @param plane The plane, on which we want to project.
 */
void Object::ProjectToPlane(Z::Plane *plane)
{
	sucProj = true;
	for(unsigned i=0; i<o_vertices.size(); i++)
	{
		Vector3 isct, point, dir;
		point.x = o_vertices[i].pos.x + position.x;		// we need absolute position
		point.y = o_vertices[i].pos.y + position.y;
		point.z = o_vertices[i].pos.z + position.z;
		dir.x = plane->normal.x;
		dir.y = plane->normal.y;
		dir.z = plane->normal.z;
		if(!plane->IntersectPlaneAndRay(isct, point, dir)) sucProj = false;
		
		VisionData::Vertex vtx;
		vtx.pos.x = isct.x - position.x;
		vtx.pos.y = isct.y - position.y;
		vtx.pos.z = isct.z - position.z;
		o_vertices_p.push_back(vtx);
		
		
	}
}

/**
 * @brief Get the original visual object.
 * @param obj Visual object pointer for object.
 * @return True for success.
 */
bool Object::GetVisualObject(VisionData::VisualObjectPtr &obj)
{
	obj->model = new VisionData::GeometryModel;

	for(unsigned i=0; i<o_vertices.size(); i++)
		obj->model->vertices.push_back(o_vertices[i]);
	for(unsigned i=0; i<o_faces.size(); i++)
		obj->model->faces.push_back(o_faces[i]);

	return true;
}

/**
 * @brief Get the visual object, received from projected faces.
 * @param obj Visual object pointer for object.
 * @return Returns true for success.
 */
bool Object::GetVisualObjectProjected(VisionData::VisualObjectPtr &obj)
{
	if(!sucProj) return false;
	
	obj->model = new VisionData::GeometryModel;

	// save original and projected vertices
	for(unsigned i=0; i<o_vertices.size(); i++)
	{
		obj->model->vertices.push_back(o_vertices[i]);
		obj->model->vertices.push_back(o_vertices_p[i]);
	}
	for(unsigned i=0; i<o_faces.size(); i++)
	{
		VisionData::Face f;

		// top surface
		for(unsigned j=0; j<o_faces[i].vertices.size(); j++)
			f.vertices.push_back(o_faces[i].vertices[j]*2);
		obj->model->faces.push_back(f);
		f.vertices.clear();

		// bottom surface
		for(unsigned j=0; j<o_faces[i].vertices.size(); j++)
			f.vertices.push_back(o_faces[i].vertices[j]*2 + 1);
		obj->model->faces.push_back(f);
		f.vertices.clear();		

		// the rest of the surfaces
		for(unsigned j=0; j<o_faces[i].vertices.size(); j++)
		{
			f.vertices.push_back(o_faces[i].vertices[j]*2);
			f.vertices.push_back(o_faces[i].vertices[j]*2 + 1);
			if(j == o_faces[i].vertices.size()-1)
			{
				f.vertices.push_back(o_faces[i].vertices[0]*2 + 1);
				f.vertices.push_back(o_faces[i].vertices[0]*2);
			}
			else
			{
				f.vertices.push_back(o_faces[i].vertices[j+1]*2 + 1);
				f.vertices.push_back(o_faces[i].vertices[j+1]*2);
			}
			obj->model->faces.push_back(f);
			f.vertices.clear();		
		}
	}
	return true;
}
}




