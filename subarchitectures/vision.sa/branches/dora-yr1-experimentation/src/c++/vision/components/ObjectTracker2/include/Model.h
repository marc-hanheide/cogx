
#ifndef __MODEL_H__
#define __MODEL_H__
class Model;

#include <stdlib.h>
#include <stddef.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <vector>

#include "Texture.h"
#include "Shader.h"
#include "mathlib.h"
#include "Pose.h"

#ifndef FN_LEN
#define FN_LEN 256
#endif


using namespace std;

class Model
{
    
public:
	typedef struct Vertex {
		vec3 pos;					// vertex position
		vec3 normal;				// vertex normal
		vec2 texCoord;				// texture coordinates u,v
	} Vertex;
	
	typedef struct Face {
		Face(){ max_pixels=0; }
		vector<int> v;            	// vertex-index list
		int max_pixels;				// number of visible pixel in best view
	} Face;
	
	typedef struct Edge {
		Edge(){ start=0; end=0; }
		int start;      		 	// start vertex index
		int end;         			// end vertex index
	} Edge;
	
	typedef struct Pass {			// Renderpass
		mat4 modelviewprojection;	// Modelview and projection matrix for texCoords
		Texture* texture;			// Texture to use
		vector<int> f;				// Faces to draw with this pass
		Pass(){ texture = new(Texture); }
		~Pass(){ delete(texture); }
	} Pass;

	vector<Vertex> 		m_vertexlist;		// vertices in 3D space with normals and texture coordinates
	vector<Face> 		m_facelist;			// faces of model (indices of vertexlist)
	vector<Edge> 		m_edgelist;			// edges of model (indices of vertexlist)
	
	vector<Pass*>		m_passlist;	
    
    Model();
    Model(const Model& m);
    ~Model();
    Model& operator=(const Model& m2);
    
    bool isTextured(){ return m_textured; }
    
    Texture* getTexture(){ return m_texture; }
    Texture* getOriginalTexture(){ return m_tex_original; }
    vector<Face> getFacelist(){ return m_facelist; }
    vector<Vertex> getVertexlist(){ return m_vertexlist; }
    mat4 getModelviewProjection(){ return m_modelviewprojection; }
    vector<Pass*> getPasslist(){ return m_passlist; }
    
    void enableTexture(bool val){ m_textured = val; }
    
    void setTexture(Texture* tex){ m_texture = tex; }
    void setOriginalTexture(Texture* tex){ m_tex_original = tex; }
    void setModelviewProjection(mat4 m){ m_modelviewprojection = m; }
    void setPasslist(vector<Pass*> p){ m_passlist = p; }
    
    void computeEdges();
    void computeNormals();
    
    void drawPass(Shader* shadeTexturing);
    void drawFaces();
    void drawFace(int i);
    void drawEdges();
    
    void restoreTexture();
    
    vector<int> getFaceUpdateList(Pose* p_max);
    void textureFromImage(unsigned char* image, int width, int height, Pose* p_max);

protected:
	char m_modelname[FN_LEN];
	
	bool m_textured;		
	
	mat4 m_modelviewprojection;		// Transformation matrix from model to camera to image -space
	
	GLint edgeDisplayList;
	Texture* m_tex_original;		// original texture of model (not modified by tracker)
    Texture* m_texture;				// texture of model modified by tracker (edge-texture)
	
    // Functions
    void print();
    bool isRedundant(Edge* e1, int k);
    void genEdgeDisplayList();
    
};


#endif
