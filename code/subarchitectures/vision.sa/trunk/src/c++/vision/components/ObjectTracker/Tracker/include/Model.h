

#ifndef __MODEL_H__
#define __MODEL_H__

#include "headers.h"
#include "mathlib.h"

class Model
{
public:
	struct Vertex {
		vec3 pos;					// vertex position
		vec3 normal;				// vertex normal
		vec2 texCoord;				// texture coordinates u,v
	};
	
	struct Face {
		std::vector<int> v;            	// vertex-index list
		vec3 normal;
	};
	
	std::vector<Vertex> 	m_vertexlist;
	std::vector<Face> 		m_facelist;
	
	virtual void computeNormals();
	virtual void computeFaceNormals();
	
	virtual void drawNormals(){}
	virtual void drawFaces(){}
};

#endif