 /**
 * @file tgModel.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Defining a model for rendering.
 */

#ifndef TG_MODEL
#define TG_MODEL

#include <stdio.h>
#include <vector>

#include "tgMathlib.h"

namespace TomGine{

/**
* @brief Class tgModel
*/
class tgModel{	
public:

	struct Vertex{
		vec3 pos;				///< 3D position of vertex
		vec3 normal;		///< normal vector of vertex
		vec2 texCoord;	///< texture coordinate of vertex
	};
	struct Face{
		std::vector<int> vertices;		///< list of vertex-indices
		vec3 normal;									///< normal vector of face
	};
	
	std::vector<Vertex>		m_vertices;	///< list of vertices
	std::vector<Face>			m_faces;		///< list of faces
	
	virtual void DrawFaces();
	virtual void DrawNormals(float normal_length);
	void ComputeNormals();
	void PrintInfo();
};

} // namespace TomGine

#endif
