
#ifndef __TRACKER_MODEL_H__
#define __TRACKER_MODEL_H__

namespace Tracking{
	class TrackerModel;
}
#include "headers.h"
#include "Model.h"
#include "Texture.h"
#include "Shader.h"
#include "mathlib.h"
#include "Pose.h"

#ifndef FN_LEN
#define FN_LEN 256
#endif

namespace Tracking{


/** @brief 3D Model with special methods for tracking and texturing */
class TrackerModel : public Model
{
private:
	TrackerModel(const TrackerModel& m);
	TrackerModel& operator=(const TrackerModel& m);

public:
	TrackerModel();
	~TrackerModel();
	
	TrackerModel& operator=(const Model& m);
	
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
		~Pass(){ delete(texture);}
	};
	
	typedef std::vector<Pass*>	PassList;
	
	// Variables
	std::vector<Edge> 		m_edgelist;			// edges of model (indices of vertexlist)
	PassList							m_passlist;
	std::vector<int>			m_facepixellist;
	std::vector<int> 			m_texturedfaces;
	
	Texture* m_tex_original;		// original texture of model (not modified by tracker)
	Texture* m_texture;				// texture of model modified by tracker (edge-texture)
	bool m_textured;
	
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
	
	std::vector<int> getFaceUpdateList(Pose* p_max, vec3 view, float minTexGrabAngle=3.0*PI/4.0);
	void textureFromImage(Texture* image, int width, int height, Pose* p_max, vec3 view, float minTexGrabAngle);
	
		// gets
	bool 			getTextured(){ return m_textured; }
	Texture* 	getTexture(){ return m_texture; }
	Texture* 	getOriginalTexture(){ return m_tex_original; }
	mat4 			getModelviewProjection(){ return m_modelviewprojection; }
	int 			getUntexturedFaces(){return ( m_facelist.size() - m_texturedfaces.size() ); }
	
	// sets
	void setBFC(bool bfc){ m_bfc = bfc; }
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
	bool m_bfc;
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

} // namespace Tracking

#endif
