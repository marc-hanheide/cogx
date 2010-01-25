 /**
 * @file Model.h
 * @author Thomas Mörwald
 * @date January 2010
 * @version 0.1
 * @brief Basic 3D geometry
 * @namespace Tracking
 */

#ifndef _MODEL_H_
#define _MODEL_H_

#include "headers.h"
#include "mathlib.h"
namespace Tracking{

/** @brief struct Vertex */
struct Vertex {
	vec3 pos;					// vertex position
	vec3 normal;				// vertex normal
	vec2 texCoord;				// texture coordinates u,v
};

/** @brief struct Face */
struct Face {
	std::vector<int> v;            	// vertex-index list
	vec3 normal;
};

/** @brief class Model */
class Model
{
protected:
	std::vector<Vertex>  	m_vertexlist;		///< Vector holding vertices of the model
	std::vector<Face> 		m_facelist;			///< Vector holding faces of the model
	
public:

	friend class TrackerModel;

	void push_back(Vertex v){ m_vertexlist.push_back(v); }
	void push_back(Face f){ m_facelist.push_back(f); }
	
	/** @brief Compute normals of vertices using cross product of faces */
	virtual void computeNormals();
	
	/** @brief Compute normals of faces using cross product of faces */
	virtual void computeFaceNormals();
	
	/** @brief Draws the normals of the vertices */
	virtual void drawNormals(){}
	
	/** @brief Draws the faces of the model */
	virtual void drawFaces(){}
	
	virtual void print();

};

} // namespace Tracking

#endif