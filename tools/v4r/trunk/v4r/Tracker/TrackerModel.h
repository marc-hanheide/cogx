
#ifndef __TRACKER_MODEL_H__
#define __TRACKER_MODEL_H__

namespace Tracking{
	class TrackerModel;
}
#include "headers.h"
#include <v4r/TomGine/tgModel.h>
#include <v4r/TomGine/tgTexture.h>
#include <v4r/TomGine/tgShader.h>
#include <v4r/TomGine/tgMathlib.h>
#include <v4r/TomGine/tgPose.h>
#include <v4r/TomGine/tgCamera.h>

#ifndef FN_LEN
#define FN_LEN 256
#endif

namespace Tracking{


/** @brief 3D Model with special methods for tracking and texturing */
class TrackerModel : public TomGine::tgModel
{
private:
	TrackerModel(const TrackerModel& m);
	
	TrackerModel& operator=(const TomGine::tgModel& m);		// no implementation (should not be used)

public:
	TrackerModel();
	TrackerModel(const TomGine::tgModel& m);
	~TrackerModel();
	
	TrackerModel& operator=(const TrackerModel& m);
	
	void releasePassList();
	
	struct Pass {											// Renderpass
		std::vector<unsigned> f;							// Faces to draw with this pass
		TomGine::mat4 modelviewprojection;					// Modelview and projection matrix for texCoords
		float x,y,w,h;										// Bounding box of SubTexture
		TomGine::tgTexture2D texture;						// TomGine::tgTexture2D to use
	};
	
	typedef std::vector<Pass*>	PassList;
	
	// Variables
	PassList				m_passlist;
	std::vector<int>		m_facepixellist;
	
	TomGine::tgTexture2D* m_tex_original;		// original texture of model (not modified by tracker)
	TomGine::tgTexture2D*	m_texture;				// texture of model modified by tracker (edge-texture)
	bool m_textured;
	bool m_fulltextured;
	
	// computes, updates
	void computeEdges();
	void computeBoundingSphere();
	void Update();
	
	// draws
	virtual void Print() const;
	virtual void drawNormals(float normal_length=0.01);
	virtual void drawFaces(bool colorful=false);
	void drawFace(int i);
	void drawEdges();
	void drawTexturedFaces();
	void drawUntexturedFaces();
	void drawPass(bool colorful=false);
	void drawCoordinates();
	
	
	std::vector<unsigned> getFaceUpdateList(TomGine::tgPose& p_max, TomGine::vec3 view, float minTexGrabAngle=3.0*M_PI_4, bool use_num_pixels=true);
	
	void getBoundingBox2D( TomGine::tgPose& pose, TomGine::tgRect2Di &bb, bool pow2=false );
	
	/** @brief capture texture from image */
	void textureFromImage(	TomGine::tgTexture2D &image,
							int width, int height,
							TomGine::tgPose& p_max,
							TomGine::vec3 view,
							float minTexGrabAngle,
							std::vector<unsigned> faceUpdateList,
							std::vector<TomGine::tgVertex> &vertices,
							TomGine::tgCamera* m_cam);
	
	void useTexCoords(bool useTC);
	void unwarpTexturesBox_hacky(const char* name);

		// gets
	bool getTextured(){ return m_textured; }
	bool getFullTextured(){ return m_fulltextured; }
	TomGine::tgTexture2D* getTexture(){ return m_texture; }
	TomGine::tgTexture2D* getOriginalTexture(){ return m_tex_original; }
	float getBoundingSphereRadius(){ return m_boundingSphereRadius; }
	
	// sets
	void setBFC(bool bfc){ m_bfc = bfc; }
	void setTexture(TomGine::tgTexture2D* tex){ m_texture = tex; }
	void setOriginalTexture(TomGine::tgTexture2D* tex){ m_tex_original = tex; }
	void restoreTexture(){ m_texture=m_tex_original; }

protected:
	GLint m_dlTexturedFaces;
	GLint m_dlUntexturedFaces;
	GLint m_dlPass;
	GLint m_dlFaces;
	GLint m_dlEdges;
	GLint m_dlNormals;
	
	TomGine::tgShader* m_shadeTexturing;
	int m_shadeTexturingID;
	bool m_bfc;
	float m_boundingSphereRadius;

	// Functions
	bool isRedundant(TomGine::tgLine* e1);
	void UpdateDisplayLists();
};

} // namespace Tracking

#endif
