
#ifndef __TRACKER_MODEL_H__
#define __TRACKER_MODEL_H__
class TrackerModel;

#include "headers.h"
#include "Model.h"
#include "Texture.h"
#include "Shader.h"
#include "mathlib.h"
#include "Particles.h"

#ifndef FN_LEN
#define FN_LEN 256
#endif

class TrackerModel : public Model
{
public:

	struct Edge {
		Edge(){ start=0; end=0; }
		int start;      		 	// start vertex index
		int end;         			// end vertex index
	};
	
	struct Pass {			// Renderpass
		mat4 modelviewprojection;	// Modelview and projection matrix for texCoords
		Texture* texture;			// Texture to use
		std::vector<int> f;				// Faces to draw with this pass
		Pass(){ texture = new(Texture); }
		~Pass(){ delete(texture); }
	};
	
	// Variables
	std::vector<Edge> 		m_edgelist;			// edges of model (indices of vertexlist)
	std::vector<Pass*>		m_passlist;
	std::vector<int>			m_facepixellist;
	std::vector<int> 		m_texturedfaces;
	
	Texture* m_tex_original;		// original texture of model (not modified by tracker)
	Texture* m_texture;				// texture of model modified by tracker (edge-texture)
	bool m_textured;
	char m_modelname[FN_LEN];
	
	// Constructors
	TrackerModel();
	TrackerModel(const TrackerModel& m);
	TrackerModel(const Model& m);
	~TrackerModel();
	
	// operators
	TrackerModel& operator=(const TrackerModel& m2);
	
	
	// computes, updates
	void computeEdges();
	void Update();
	
	// draws
	void print();
	virtual void drawNormals();
	virtual void drawFaces();
	void drawFace(int i);
	void drawEdges();
	void drawTexturedFaces();
	void drawUntexturedFaces();
	void drawPass();
	
	std::vector<int> getFaceUpdateList(Particle* p_max, vec3 view, float minTexGrabAngle=3.0*PI/4.0);
	void textureFromImage(Texture* image, int width, int height, Particle* p_max, vec3 view, float minTexGrabAngle);
	
		// gets
	bool 			getTextured(){ return m_textured; }
	Texture* 	getTexture(){ return m_texture; }
	Texture* 	getOriginalTexture(){ return m_tex_original; }
	mat4 			getModelviewProjection(){ return m_modelviewprojection; }
	int 			getUntexturedFaces(){return ( m_facelist.size() - m_texturedfaces.size() ); }
	
	// sets
	void setTexture(Texture* tex){ m_texture = tex; }
	void setOriginalTexture(Texture* tex){ m_tex_original = tex; }
	void setModelviewProjection(mat4 m){ m_modelviewprojection = m; }
	void restoreTexture(){ m_texture=m_tex_original; }
	
protected:
	GLint m_dlTexturedFaces;
	GLint m_dlUntexturedFaces;
	GLint m_dlPass;
	GLint m_dlFaces;
	GLint m_dlEdges;
	GLint m_dlNormals;
	
	Shader* m_shadeTexturing;
	
	mat4 m_modelviewprojection;		// Transformation matrix from model to camera to image -space

	// Functions
	bool isRedundant(Edge* e1);
	void UpdateDisplayLists();
	
	// generate display lists
	void genListTexturedFaces();
	void genListUntexturedFaces();
	void genListPass();
	void genListFaces();
	void genListEdges();
	void genListNormals(float normal_length);
};


#endif
