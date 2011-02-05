
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
#include "Camera.h"

#ifndef FN_LEN
#define FN_LEN 256
#endif

namespace Tracking{


/** @brief 3D Model with special methods for tracking and texturing */
class TrackerModel : public Model
{
private:
	TrackerModel(const TrackerModel& m);

public:
	TrackerModel();
	~TrackerModel();
	
	TrackerModel& operator=(const TrackerModel& m);
	TrackerModel& operator=(const Model& m);
	
	void releasePassList();
	
	struct Edge {
		Edge(){ start=0; end=0; }
		int start;      		 	// start vertex index
		int end;         			// end vertex index
	};
	
	struct Pass {												// Renderpass
		std::vector<int> f;								// Faces to draw with this pass
		mat4 modelviewprojection;					// Modelview and projection matrix for texCoords
		float x,y,w,h;										// Bounding box of SubTexture
		Texture* texture;									// Texture to use
		Pass(){ texture = new(Texture); glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); }
		~Pass(){ delete(texture);}
	};
	
	typedef std::vector<Pass*>	PassList;
	
	// Variables
	std::vector<Edge> 		m_edgelist;			// edges of model (indices of vertexlist)
	PassList							m_passlist;
	std::vector<int>			m_facepixellist;
	
	Texture* m_tex_original;		// original texture of model (not modified by tracker)
	Texture* m_texture;				// texture of model modified by tracker (edge-texture)
	bool m_textured;
	
	// computes, updates
	void computeEdges();
	void computeBoundingSphere();
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
	void drawCoordinates();
	
	
	std::vector<int> getFaceUpdateList(Pose& p_max, vec3 view, float minTexGrabAngle=3.0*PI/4.0, bool use_num_pixels=true);
	
	/** @brief capture texture from image */
	void textureFromImage(	Texture* image,
													int width, int height,
													Pose& p_max,
													vec3 view,
													float minTexGrabAngle,
													std::vector<int> faceUpdateList,
													std::vector<Vertex> &vertices,
													Camera* m_cam);
	
		// gets
	bool 			getTextured(){ return m_textured; }
	Texture* 	getTexture(){ return m_texture; }
	Texture* 	getOriginalTexture(){ return m_tex_original; }
	float			getBoundingSphereRadius(){ return m_boundingSphereRadius; }
	
	// sets
	void setBFC(bool bfc){ m_bfc = bfc; }
	void setTexture(Texture* tex){ m_texture = tex; }
	void setOriginalTexture(Texture* tex){ m_tex_original = tex; }
	void restoreTexture(){ m_texture=m_tex_original; }
	
		// generate display lists
	void genListTexturedFaces();
	void genListUntexturedFaces();
	void genListPass();
	void genListFaces();
	void genListEdges();
	void genListNormals(float normal_length);
	
protected:
	GLint m_dlTexturedFaces;
	GLint m_dlUntexturedFaces;
	GLint m_dlPass;
	GLint m_dlFaces;
	GLint m_dlEdges;
	GLint m_dlNormals;
	
	Shader* m_shadeTexturing;
	int m_shadeTexturingID;
	bool m_bfc;
	float m_boundingSphereRadius;

	// Functions
	bool isRedundant(Edge* e1);
	void UpdateDisplayLists();
};

} // namespace Tracking

#endif
