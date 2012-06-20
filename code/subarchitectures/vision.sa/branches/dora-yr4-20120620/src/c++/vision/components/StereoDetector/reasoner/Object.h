/**
 * @file Object.h
 * @author Andreas Richtsfeld
 * @date August 2010
 * @version 0.1
 * @brief Class for objects.
 */


#ifndef Z_OBJECT_HH
#define Z_OBJECT_HH

#include <stdio.h>
#include <vector>
#include <VisionData.hpp>
#include "Vector.hh"
#include "Plane.h"

namespace Z
{

/**
 * @brief Class for objects
 */
class Object
{
private:	
	Vector3 position;																	///< Position of the object
	
	std::vector<VisionData::Vertex> o_vertices;				///< List of vertices, relativ to position
	std::vector<VisionData::Face> o_faces;						///< List of faces

	bool sucProj;																			///< True, when sucessfully projected to ground plane
	std::vector<VisionData::Vertex> o_vertices_p;			///< List of vertices on ground plane
	
public:
	Object(Vector3 pos, VisionData::VertexSeq vertices, VisionData::FaceSeq faces);
	void ProjectToPlane(Z::Plane *plane);
	bool GetVisualObject(VisionData::VisualObjectPtr &obj);
	bool GetVisualObjectProjected(VisionData::VisualObjectPtr &obj);

};

}

#endif
