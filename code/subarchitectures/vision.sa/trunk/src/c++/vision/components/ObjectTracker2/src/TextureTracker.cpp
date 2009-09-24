
#include "TextureTracker.h"

// *** PRIVATE ***

// Process camera image (gauss, sobel, thinning, spreading, rendering)
void TextureTracker::image_processing(unsigned char* image){
	// Load camera image to texture
	m_tex_frame->load(image, params.width, params.height);
	
	// Preprocessing for camera image
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDepthMask(0);
	glColor3f(1,1,1);
	m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);
	m_ip->gauss(m_tex_frame, m_tex_frame_ip[0]);
	m_ip->sobel(m_tex_frame_ip[0], m_tex_frame_ip[0]);
	m_ip->thinning(m_tex_frame_ip[0], m_tex_frame_ip[0]);
	glDepthMask(1);
}

// Draw Model to screen, extract modelview matrix, perform image processing for model
void TextureTracker::model_processing(Pose* p){
	
	// Render camera image as background
	//m_opengl.RenderSettings(true, true); 	// (color-enabled, depth-enabled)
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if(m_bfc) glEnable(GL_CULL_FACE);
	else glDisable(GL_CULL_FACE);
	
	// Render model to screen (=projection)
	m_cam_perspective->Activate();			// activate perspective view
	m_lighting.Activate();
	p->activateGL();
	glGetFloatv(GL_MODELVIEW_MATRIX, m_modelview);
	glGetFloatv(GL_PROJECTION_MATRIX, m_projection);
	m_modelviewprojection = m_projection * m_modelview;	
	m_ortho_search->set_modelviewprojection(m_modelviewprojection);
	
	m_model->restoreTexture();
	
	m_shadeTexturing->bind();
	m_shadeTexturing->setUniform("modelviewprojection", m_modelviewprojection, GL_FALSE);		
		m_model->drawPass(m_shadeTexturing);
	m_shadeTexturing->unbind();
	
	p->deactivateGL();
	m_lighting.Deactivate();
	
	
	// Copy screen to texture (=reprojection to model)
	m_tex_model->copyTexImage2D(params.width, params.height);
		
	// perform image processing with reprojected image
	glDepthMask(0);
	if(m_model->isTextured()){
		m_ip->gauss(m_tex_model, m_tex_model_ip[0]);
	}else{
		m_ip->copy(m_tex_model, m_tex_model_ip[0]);
	}
	m_ip->sobel(m_tex_model_ip[0], m_tex_model_ip[0]);
	m_ip->thinning(m_tex_model_ip[0], m_tex_model_ip[0]);
	glDepthMask(1);
	
	m_model->setTexture(m_tex_model_ip[0]);
}

// *** PUBLIC ***

TextureTracker::TextureTracker(){
	m_lock = false;
	m_showmodel = true;
	m_zero_pose = false;
	m_draw_edges = false;
	m_tracker_initialized = false;
	m_testflag = false;
	m_bfc = false;
		
	int id;
	// Shader
	if((id = g_Resources->AddShader("texturing", "texturing.vert", "texturing.frag")) == -1)
		exit(1);
	m_shadeTexturing = g_Resources->GetShader(id);

}

// Initialise function (must be called before tracking)
bool TextureTracker::initInternal()
{	
	return (m_tracker_initialized = true);
}

// Tracking function for texture tracking
bool TextureTracker::track(	unsigned char* image,	// camera image (3 channel, unsigned byte each)
							Model* model,			// tracking model (textured, vertexlist, facelist) 
							Camera* camera,			// extrinsic camera (defining pose and projection)
							Pose p_estimate,		// position estimate of model (R,t)
							Pose& p_result,			// storage to write tracked position
							float time)			
{
	// Start time measurement
	m_timer.Update();
	
	// Check if input is valid
	isReady(image, model, camera, &p_estimate);	
	
	// Process image from camera (edge detection)
	image_processing(m_image);
	
	// Process model (texture reprojection, edge detection)
	model_processing(&p_estimate);
	
	p_result = p_estimate;
	
	// Clear framebuffer and render image from camera
	if(!m_lock){
		// TODO TODO TODO
		// Ortho Search with Texture
		p_result = m_ortho_search->findPose(p_result, m_tex_frame_ip[0], model, m_tex_model, m_cam_perspective);
		
		// adjust number of particles according to tracking speed
	}
	
	// Copy result to output
	if(m_zero_pose){
		p_result = params.zP;
		m_zero_pose = false;
	}
	
	
	if(m_draw_edges)
		m_ip->render(m_tex_frame_ip[0]);
	else
		m_ip->render(m_tex_frame);
	
	time_tracking = m_timer.Update();
	
	//m_opengl.RenderSettings(true, true); 	// (color-enabled, depth-enabled)
	//m_lighting.getLightDirection(m_tex_frame, m_particles->getMax(), m_model, m_cam_perspective);
	
	
	
	return true;
}

// grabs texture from camera image and attaches it to faces of model
void TextureTracker::textureFromImage(Pose &p)
{
	Pose p_estimate = p;
	Pose p_result, p_old, p_new;	
	
	m_model->setOriginalTexture(0);
	m_model->setTexture(0);
	m_model->enableTexture(false);
	
	track(m_image, m_model, m_cam_perspective, p_estimate, p_result,0.0);
	drawResult(&p_result);
	swap();
	SDL_Delay(100);	
	
	m_model->textureFromImage(m_image, params.width, params.height, &p_result);
	m_shadeTexturing->bind();
	m_shadeTexturing->setUniform("useTexCoords", false);
	m_shadeTexturing->unbind();
	
	m_model->enableTexture(true);
}

// Draw result of texture tracking (particle with maximum likelihood)
void TextureTracker::drawResult(Pose* p){
	
	m_cam_perspective->Activate();
	m_lighting.Activate();
	p->activateGL();
	
	m_opengl.RenderSettings(true, false);
	glDisable(GL_DEPTH_TEST);
	
	if(m_showmodel){
		m_model->restoreTexture();
		m_shadeTexturing->bind();
		m_model->drawPass(m_shadeTexturing);
	}else{
		m_model->setTexture(m_tex_model_ip[1]);
		m_model->drawFaces();
	}
	 
	if(m_showmodel){
	 	m_shadeTexturing->unbind();
	}else{
	}
	
	glEnable(GL_DEPTH_TEST);
	m_opengl.RenderSettings(true, true);
	
	p->deactivateGL();
	m_lighting.Deactivate();
}

