
#ifndef __TEXTURE_TRACKER_H__
#define __TEXTURE_TRACKER_H__

#include "Tracker.h"
#include "Lighting.h"

class TextureTracker : public Tracker
{
private:
	
	// Resources
	Shader* m_shadeTexturing;
	Lighting m_lighting;
	
	// Functions
	void model_processing(Pose* p);
	virtual void image_processing(unsigned char* image);
		
public:
	TextureTracker();
	
	virtual bool initInternal();
				
	virtual bool track(	unsigned char* image,
						Model* model,
						Camera* camera,
						Pose p_estimate,
						Pose& p_result,
						float time);
						
	virtual void textureFromImage(Pose &p);
						
	virtual void drawResult(Pose* p);
	

};

#endif
