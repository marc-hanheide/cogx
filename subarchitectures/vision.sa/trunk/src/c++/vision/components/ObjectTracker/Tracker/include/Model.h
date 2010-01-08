
#ifndef __MODEL_H__
#define __MODEL_H__
class Model;

#include "headers.h"
#include "Texture.h"
#include "Shader.h"
#include "mathlib.h"

#ifndef FN_LEN
#define FN_LEN 256
#endif


class Particle;

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
		vec3 normal;
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
	    
	Model();
	Model(const Model& m);
	~Model();
	Model& operator=(const Model& m2);
	
	void print();
	bool isTextured(){ return m_textured; }
	
	Texture* getTexture(){ return m_texture; }
	Texture* getOriginalTexture(){ return m_tex_original; }
	vector<Face> getFacelist(){ return m_facelist; }
	vector<Vertex> getVertexlist(){ return m_vertexlist; }
	vector<Pass*> getPasslist(){ return m_passlist; }
	
	mat4 getModelviewProjection(){ return m_modelviewprojection; }
	int getUntexturedFaces(){return ( m_facelist.size() - m_texturedfaces.size() ); }
	
	void enableTexture(bool val){ m_textured = val; }
	
	void setTexture(Texture* tex){ m_texture = tex; }
	void setOriginalTexture(Texture* tex){ m_tex_original = tex; }
	void setModelviewProjection(mat4 m){ m_modelviewprojection = m; }
	void setPasslist(vector<Pass*> p){ m_passlist = p; }
	
	void computeEdges();
	void computeNormals();
	void computeFaceNormals();
	void flipNormals();
	
	void UpdateDisplayLists();
	void drawNormals();
	void drawTexturedFaces();
	void drawUntexturedFaces();
	void drawPass();
	void drawFaces();
	void drawEdges();
	
	void drawFace(int i);
	
	void restoreTexture();
	
	vector<int> getFaceUpdateList(Particle* p_max, vec3 view, float minTexGrabAngle=3.0*PI/4.0);
	void textureFromImage(Texture* image, int width, int height, Particle* p_max, vec3 view, float minTexGrabAngle);
	
// PUBLIC
	vector<Vertex> 	m_vertexlist;		// vertices in 3D space with normals and texture coordinates
	vector<Face> 		m_facelist;			// faces of model (indices of vertexlist)
	vector<Edge> 		m_edgelist;			// edges of model (indices of vertexlist)
	vector<Pass*>		m_passlist;
	
protected:
	char m_modelname[FN_LEN];
	
	bool m_textured;
	
	GLint m_dlTexturedFaces;
	GLint m_dlUntexturedFaces;
	GLint m_dlPass;
	GLint m_dlFaces;
	GLint m_dlEdges;
	GLint m_dlNormals;
	
	Shader* m_shadeTexturing;
	
	mat4 m_modelviewprojection;		// Transformation matrix from model to camera to image -space
	
// 	GLint edgeDisplayList;
	vector<int> m_texturedfaces;
	Texture* m_tex_original;		// original texture of model (not modified by tracker)
	Texture* m_texture;				// texture of model modified by tracker (edge-texture)

	// Functions
	bool isRedundant(Edge* e1);
	
	void genListTexturedFaces();
	void genListUntexturedFaces();
	void genListPass();
	void genListFaces();
	void genListEdges();
	void genListNormals(float normal_length);
};


#endif
