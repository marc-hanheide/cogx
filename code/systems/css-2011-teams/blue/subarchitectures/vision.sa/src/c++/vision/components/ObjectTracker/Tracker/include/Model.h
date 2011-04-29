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
	vec4 texCoord;				// texture coordinates u,v
};

/** @brief struct Face */
struct Face {
	std::vector<int> v;            	// vertex-index list
	vec3 normal;
};

typedef std::vector<Vertex> VertexList;
typedef std::vector<Face>  FaceList;

/** @brief class Model */
class Model
{
protected:
	VertexList  m_vertexlist;		///< Vector holding vertices of the model
	FaceList 		m_facelist;			///< Vector holding faces of the model
	
public:

	friend class TrackerModel;

	void push_back(Vertex v){ m_vertexlist.push_back(v); }
	void push_back(Face f){ m_facelist.push_back(f); }
	
	Vertex	getVertex(int i){ if(i<m_vertexlist.size() && i>=0) return m_vertexlist[i]; }
	Face		getFace(int i){ if(i<m_facelist.size() && i>=0) return m_facelist[i]; }
	int			getVertexSize(){ return m_vertexlist.size(); }
	int			getFaceSize(){ return m_facelist.size(); }
	
	/** @brief Compute normals of vertices using cross product of faces */
	virtual void computeNormals();
	
	/** @brief Compute normals of faces using cross product of faces */
	virtual void computeFaceNormals();
	
	/** @brief Draws the normals of the vertices */
	virtual void drawNormals(){}
	
	/** @brief Draws the faces of the model */
	virtual void drawFaces(){}
	
	virtual void print();
	
	virtual void clear();

};

} // namespace Tracking

#endif