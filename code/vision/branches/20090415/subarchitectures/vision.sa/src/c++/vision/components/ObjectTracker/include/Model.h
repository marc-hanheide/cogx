
#ifndef __MODEL_H__
#define __MODEL_H__

#include <stdlib.h>
#include <stddef.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <vector>

#include "Texture.h"
#include "mathlib.h"

using namespace std;

class Model
{
    
public:
	typedef struct Vertex {
		vec3 pos;				// vertex position
		vec3 normal;			// vertex normal
		vec2 texCoord;		// texture coordinates u,v
	} Vertex;
	
	typedef struct Face {
		vector<int> v;            	// pointer to memory holding the vertex-index list
	} Face;
	
	typedef struct Edge {
		int start;      		 	// start vertex index
		int end;         			// end vertex index
	} Edge;

	vector<Vertex> m_vertexlist;
	vector<Face> m_facelist;
	vector<Edge> m_edgelist;

    
    Model();
    Model(const Model& m);
    ~Model();
    Model& operator=(const Model& m2);
    
    Texture* getTexture(){ return m_texture; }
    void setTexture(Texture* tex){ m_texture = tex; }
    void setOriginalTexture(Texture* tex){ m_tex_original = tex; }
    
    void computeEdges();
    void computeNormals();
    
    void drawFaces();
    void drawEdges();
    
    void restoreTexture();

protected:
	
	GLint edgeDisplayList;
	Texture* m_tex_original;		// original texture of model (not modified by tracker)
    Texture* m_texture;				// texture of model modified by tracker (edge-texture)
	
    // Functions
    void print();
    bool isRedundant(Edge* e1, int k);
    void genEdgeDisplayList();
    
};


#endif
