
#ifndef __TRACKER_MODEL_H__
#define __TRACKER_MODEL_H__
class TrackerModel;

#include "headers.h"
#include "Model.h"
#include "Texture.h"
#include "Shader.h"
#include "mathlib.h"

#ifndef FN_LEN
#define FN_LEN 256
#endif


class Particle; // TODO TODO TODO TODO TODO TODO UGLY UGLY UGLY !!!!!

using namespace std;

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
		vector<int> f;				// Faces to draw with this pass
		Pass(){ texture = new(Texture); }
		~Pass(){ delete(texture); }
	};
	    
	TrackerModel();
	TrackerModel(const TrackerModel& m);
	~TrackerModel();
	TrackerModel& operator=(const TrackerModel& m2);
	
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
	void flipNormals();
	
	void Update();
	void UpdateDisplayLists();
	
	virtual void drawNormals();
	virtual void drawFaces();
	void drawTexturedFaces();
	void drawUntexturedFaces();
	void drawPass();
	
	void drawEdges();
	
	void drawFace(int i);
	
	void restoreTexture();
	
	vector<int> getFaceUpdateList(Particle* p_max, vec3 view, float minTexGrabAngle=3.0*PI/4.0);
	void textureFromImage(Texture* image, int width, int height, Particle* p_max, vec3 view, float minTexGrabAngle);
	
// PUBLIC
	vector<Edge> 		m_edgelist;			// edges of model (indices of vertexlist)
	vector<Pass*>		m_passlist;
	vector<int>			m_facepixellist;
	
	Texture* m_tex_original;		// original texture of model (not modified by tracker)
	Texture* m_texture;				// texture of model modified by tracker (edge-texture)
	bool m_textured;
	char m_modelname[FN_LEN];
	
protected:
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
